import os 
import sys
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit('please declare "$SUMO_HOME" variable')
import traci
import traci.constants as tc
import numpy as np
import random
import timeit
import os

# phase codes based on environment.net.xml
PHASE_NS_GREEN = 0 
PHASE_NS_YELLOW = 1
PHASE_EW_GREEN = 2
PHASE_EW_YELLOW = 3


class Simulation:
    def __init__(self, Model, TrafficGen, sumo_cmd, max_steps, green_duration, yellow_duration, num_states, num_actions):
        self._Model = Model
        self._TrafficGen = TrafficGen
        self._step = 0
        self._sumo_cmd = sumo_cmd
        self._max_steps = max_steps
        self._green_duration = green_duration
        self._yellow_duration = yellow_duration
        self._num_states = num_states
        self._num_actions = num_actions
        self._reward_episode = []


    def run(self, episode):
        """
        Runs the testing simulation
        """
        start_time = timeit.default_timer()

        # first, generate the route file for this simulation and set up sumo
        self._TrafficGen.generate_routefile(seed=episode)
        traci.start(self._sumo_cmd)
        print("Simulating...")

        # inits
        self._step = 0
        self._waiting_times = {}
        old_total_wait = 0
        old_action = -1 # dummy init
        last_green_phase = -1
        cur_green_phase = 0
        while self._step < self._max_steps:
            # Flag represents the phase have been extended
            flag = 0
            # get current state of the intersection
            current_state = self._get_state()

            # calculate reward of previous action: (change in cumulative waiting time between actions)
            # waiting time = seconds waited by a car since the spawn in the environment, cumulated for every car in incoming lanes
            current_total_wait = self._get_waiting_times()
            reward = old_total_wait - current_total_wait

            # choose the light phase to activate, based on the current state of the intersection
            action = self._choose_action(current_state)

            # if the chosen phase is different from the last phase, activate the yellow phase
            if self._step != 0: 
                if action == 0: # action 0: extend the last phase
                    self._set_green_phase(last_green_phase)
                    self._simulate(self._green_duration)
                    flag = 1
                elif action == 1: # action 1: change phase
                    self._set_yellow_phase(last_green_phase)
                    self._simulate(self._yellow_duration)
                    if last_green_phase == 0:
                        cur_green_phase = 2
                    elif last_green_phase == 2:
                        cur_green_phase = 0

            # execute the phase selected before
            if flag == 0:
                self._set_green_phase(cur_green_phase)
                self._simulate(self._green_duration)

            # saving variables for later & accumulate reward
            old_action = action
            old_total_wait = current_total_wait
            last_green_phase = cur_green_phase
            self._reward_episode.append(reward)

        #print("Total reward:", np.sum(self._reward_episode))
        traci.close()
        simulation_time = round(timeit.default_timer() - start_time, 1)

        return simulation_time


    def _simulate(self, steps_todo):
        """
        Proceed with the simulation in sumo
        """
        if (self._step + steps_todo) >= self._max_steps:  # do not do more steps than the maximum allowed number of steps
            steps_todo = self._max_steps - self._step

        while steps_todo > 0:
            traci.simulationStep()  # simulate 1 step in sumo
            self._step += 1 # update the step counter
            steps_todo -= 1


    def _collect_speed(self):
        """
        Retrieve the speed of all car in the 50 meters away from traffic light from all directions
        """
        total_speed = 0
        traci.junction.subscribeContext('TL', tc.CMD_GET_VEHICLE_VARIABLE, 50, [tc.VAR_SPEED, tc.VAR_ROAD_ID])
        incoming_roads = ["E2TL", "N2TL", "W2TL", "S2TL"]
        vehicle_info = traci.junction.getContextSubscriptionResults('TL')
        for vehicle_id, vehicle_data in vehicle_info.items():
            speed = vehicle_data[tc.VAR_SPEED]
            road_id = vehicle_data[tc.VAR_ROAD_ID]
            if road_id in incoming_roads:
                total_speed += speed
        return total_speed


    def _choose_action(self, state):
        """
        Pick the best action known based on the current state of the env
        """
        return np.argmax(self._Model.predict_one(state))


    def _set_yellow_phase(self, last_green_phase):
        """
        Activate the correct yellow light combination in sumo
        """
        yellow_phase_code = last_green_phase + 1 # obtain the yellow phase code, based on the old action (ref on environment.net.xml)
        traci.trafficlight.setPhase("TL", yellow_phase_code)


    def _set_green_phase(self, green_phase):
        """
        Activate the correct green light combination in sumo
        """
        traci.trafficlight.setPhase("TL", green_phase)


    def _get_waiting_times(self):
        """
        Retrieve the waiting time of all car in 50 meters away from traffic light area from all directions
        """
        total_waiting_time = 0
        traci.junction.subscribeContext('TL', tc.CMD_GET_VEHICLE_VARIABLE, 50, [tc.VAR_WAITING_TIME, tc.VAR_ROAD_ID])
        incoming_roads = ["E2TL", "N2TL", "W2TL", "S2TL"]
        vehicle_info = traci.junction.getContextSubscriptionResults('TL')
        for vehicle_id, vehicle_data in vehicle_info.items():
            waiting_time = vehicle_data[tc.VAR_WAITING_TIME]
            road_id = vehicle_data[tc.VAR_ROAD_ID]
            if road_id in incoming_roads:
                total_waiting_time += waiting_time
        return total_waiting_time


    def _get_state(self):
        """
        State la cac feature sau cua moi lane, 50 met tinh tu nut giao
        * number of vehicle 
        * percentage of occupied area
        * average speed
        """
        traci.junction.subscribeContext('TL', tc.CMD_GET_VEHICLE_VARIABLE, 50, [tc.VAR_SPEED, tc.VAR_ROAD_ID])
        Route_W_E = [0, 0, 0] 
        Route_N_S = [0, 0, 0]
        Route_E_W = [0, 0, 0]
        Route_S_N = [0, 0, 0]
        vehicle_info = traci.junction.getContextSubscriptionResults('TL')
        if vehicle_info is not None:
            for vehicle_id, vehicle_data in vehicle_info.items():
                speed = vehicle_data[tc.VAR_SPEED]
                road = vehicle_data[tc.VAR_ROAD_ID]
                if road == 'W2TL':
                    Route_W_E[0] += 1
                    Route_W_E[1] += 5
                    Route_W_E[2] += speed
                elif road == 'N2TL':
                    Route_N_S[0] += 1
                    Route_N_S[1] += 5
                    Route_N_S[2] += speed
                elif road == 'E2TL':
                    Route_E_W[0] += 1
                    Route_E_W[1] += 5
                    Route_E_W[2] += speed
                elif road == 'S2TL':
                    Route_S_N[0] += 1
                    Route_S_N[1] += 5
                    Route_S_N[2] += speed

            if Route_W_E[0] != 0:
                Route_W_E[2] /= Route_W_E[0]
            if Route_N_S[0] != 0:
                Route_N_S[2] /= Route_N_S[0]
            if Route_E_W[0] != 0:
                Route_E_W[2] /= Route_E_W[0]
            if Route_S_N[0] != 0:
                Route_S_N[2] /= Route_S_N[0]
            Route_W_E[1] /= 50
            Route_N_S[1] /= 50
            Route_E_W[1] /= 50
            Route_S_N[1] /= 50
        state = np.array([Route_W_E, Route_N_S, Route_E_W, Route_S_N])
        state = np.reshape(state, [12, ])
        # print(state)
        return state

    @property
    def reward_episode(self):
        return self._reward_episode