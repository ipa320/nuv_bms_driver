cmake_minimum_required(VERSION 2.8.3)
project(nuv_bms_driver)

# add_compile_options(-std=c++11)

set(catkin_RUN_PACKAGES
	cob_msgs
	roscpp
)

set(catkin_BUILD_PACKAGES
	${catkin_RUN_PACKAGES}
)

find_package(catkin REQUIRED COMPONENTS
	${catkin_BUILD_PACKAGES}
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")

find_package(Modbus)


###################################
## catkin specific configuration ##
###################################
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES nuv_300_bms
	CATKIN_DEPENDS
		${catkin_RUN_PACKAGES}
	DEPENDS
		Modbus
)

###########
## Build ##
###########

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${MODBUS_INCLUDE_DIRS}
)

add_library(nuv_300_bms src/nuv_300_bms.cpp)
target_link_libraries(nuv_300_bms
	${catkin_LIBRARIES}
	${MODBUS_LIBRARIES}
)

add_executable(nuvation_nuv300_bms_driver_node src/nuvation_nuv300_bms_driver_node.cpp)
add_dependencies(nuvation_nuv300_bms_driver_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(nuvation_nuv300_bms_driver_node
	${catkin_LIBRARIES}
	${MODBUS_LIBRARIES}
)

#############
## Install ##
#############
install(TARGETS nuvation_nuv300_bms_driver_node nuv_300_bms
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
