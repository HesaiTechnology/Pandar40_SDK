
add_library(pandar_rawdata rawdata.cc calibration.cc)
target_link_libraries(
	pandar_rawdata
)

set_target_properties(pandar_rawdata PROPERTIES
	COMPILE_FLAGS -std=c++11)


add_library(pandar_input input.cc)
target_link_libraries(pandar_input
)

add_library(pandora_utilities utilities.cc)
target_link_libraries(pandora_utilities
)

add_library(pandora_sdk_in hesaiLidarSDK_IN.cc)

target_link_libraries(pandora_sdk_in
	pandar_input
	pandar_rawdata
	pandora_utilities
	pandora_client
)



