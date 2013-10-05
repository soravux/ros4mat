cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Pour le phidgets
set(CMAKE_EXE_LINKER_FLAGS "-Wl,-L/usr/lib,-lphidget21")

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

rosbuild_add_executable(E_IMU src/phidgetsDriver.cpp)
rosbuild_add_executable(E_ADC src/adcDriver.cpp)
rosbuild_add_executable(E_Battery src/batDriver.cpp)
rosbuild_add_executable(E_Camera src/camDriver.cpp src/jpge.cpp)
rosbuild_add_executable(E_StereoCamera src/stereoCamDriver.cpp src/jpge.cpp)
rosbuild_add_executable(E_Computer src/computerInfoDriver.cpp)
rosbuild_add_executable(E_Kinect src/kinectDriver.cpp src/jpge.cpp)
rosbuild_add_executable(E_Serial src/serialDriver.cpp)
rosbuild_add_executable(C_ADC src/adcClient.cpp)
rosbuild_add_executable(C_IMU src/phidgetsClient.cpp)
rosbuild_add_executable(C_Battery src/batClient.cpp)
rosbuild_add_executable(C_Serial src/serialClient.cpp)
rosbuild_add_executable(C_GPS src/gpsClient.cpp)
rosbuild_add_executable(E_Logico src/ros4mat.cpp ../thirdparty/easyzlib.c)

target_link_libraries(E_IMU phidget21)
target_link_libraries(E_StereoCamera boost_signals)
target_link_libraries(E_Kinect boost_signals)
target_link_libraries(E_Logico pthread)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
