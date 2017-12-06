#include_directories(
#	../../../include
#)

message("hello")

message("${INCLUDE_DIRECTORIES}")

add_library(pandar_rawdata rawdata.cc calibration.cc)
target_link_libraries(pandar_rawdata 
                      ${YAML_CPP_LIBRARIES}
                      ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
set_target_properties(pandar_rawdata PROPERTIES
	COMPILE_FLAGS -std=c++11)


add_library(pandar_input input.cc)
target_link_libraries(pandar_input
  ${libpcap_LIBRARIES}
)

add_library(angle angles.cc)
