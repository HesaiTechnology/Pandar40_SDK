# HesaiLidar_General_SDK  

## About the project  
HesaiLidar_General_SDK project is the software development kit for:**Pandar40** LiDAR manufactured by Hesai Technology.
## Environment and Dependencies
**System environment requirement: Linux + G++ 7.0 or above**   
**Library Dependencies: libpcap-dev + libyaml-cpp-dev**  
```
$ sudo apt install libpcap-dev libyaml-cpp-dev
```
## Clone
```
$ git clone https://github.com/HesaiTechnology/Pandar40_SDK.git --recursive
```
## Build
```
$ cd Pandar40_SDK
$ mkdir build
$ cd build
$ cmake ..
$ make
```  
# Add to your project
```
	add_subdirectory(<path to>Pandar40_SDK)

	target_link_libraries(${YOUR_PROJECT_NAME}
		...
		Pandar40_SDK
		...
	)

```
