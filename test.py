import h5py
model_path = "/Users/cicilian/Desktop/Môn học/Năm II - HK 2/AI/Do_an_CK/myTesting/models/model_4/trained_model.h5"
with h5py.File(model_path, 'r') as f:
    print(list(f.keys()))  # In ra các nhóm dữ liệu trong tệp HDF5