# Dependencies
## Clone
```
	git clone https://github.com/HesaiTechnology/Pandar40_SDK.git --recursive
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
	add_subdirectory(<path to>Pandar40_SDK)

	target_link_libraries(${YOUR_PROJECT_NAME}
		...
		Pandar40_SDK
		...
	)

```
