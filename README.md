# Dependencies
## Clone
```
git clone https://github.com/HesaiTechnology/HesaiLidarSDK.git --recursive
```

## Only Lidar
```
sudo apt install cmake libproj-dev libpcap-dev libboost-all-dev libyaml-cpp-dev libjpeg-dev libgdal-dev libpq-dev libvtk6-dev libvtk6-qt-dev libpcl-dev 
```

# build
```
mkdir build
cd build 
cmake ..  /
make -j4
make install; # Generate SDK DIR for HesaiLidarSDK, include (1) Headers (2) libhesaiLidarSDK.so in build/SDK
```

# Add to your project
```
add_subdirectory(<path to>HesaiLidarSDK)

target_link_libraries(${PROJECT_NAME}
  ...
  hesaiLidarSDK
  ...
}

```
