# Dependencies
## Clone
```
git clone https://github.com/HesaiTechnology/HesaiLidarSDK.git --recursive
```

## Only Lidar
```
sudo apt install cmake libproj-dev libpcap-dev libboost-all-dev libyaml-cpp-dev libjpeg-dev libgdal-dev libpq-dev libvtk6-dev libvtk6-qt-dev libpcl-dev 
```

## Camera + Lidar

```
sudo apt install cmake libproj-dev libcv-dev libpcap-dev libboost-all-dev libyaml-cpp-dev libjpeg-dev libgdal-dev libpq-dev libvtk6-dev libvtk6-qt-dev libpcl-dev 
```
# build
```
mkdir build
cd build 
 (Lidar )cmake ..  / cmake -DCamera_Enable=ON .. (Lidar + Camera)
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

## Note
```
1. Issue when compiling. Please Add '-DCamera_Enable=ON'
add 'add_definitions(-DHESAI_WITH_CAMERA)' to your CMakeLists.txt
cmake -DCamera_Enable=ON <your project DIR>.
```
