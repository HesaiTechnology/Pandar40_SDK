# Dependencies
## Clone
```
	git clone https://github.com/HesaiTechnology/HesaiLidar_SDK.git --recursive
```
# Build
```
	mkdir build
	cd build 
	cmake ..
	make
```

# Add to your project
```
	add_subdirectory(<path to>HesaiLidarSDK)

	target_link_libraries(${YOUR_PROJECT_NAME}
		...
		hesaiLidarSDK
		...
	)

```
