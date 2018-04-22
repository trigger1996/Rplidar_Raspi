cmake_minimum_required(VERSION 3.0.0)

project(rplidar)

set(CMAKE_CXX_STANDARD 11)

 ADD_DEFINITIONS(-DDEBUG)

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/install)

set(SRC
	# 自定文件
	${PROJECT_SOURCE_DIR}/main.cpp
	${PROJECT_SOURCE_DIR}/lidar_driver.cpp
	${PROJECT_SOURCE_DIR}/og_mapping.cpp
	${PROJECT_SOURCE_DIR}/icp_interface.cpp
	# ICP算法
	${PROJECT_SOURCE_DIR}/icp_opencv/icp-opencv/icp.c
	${PROJECT_SOURCE_DIR}/icp_opencv/third_party/kdtree/kdtree.c
	# Rplidar SDK
	sdk/src/rplidar_driver.cpp
        sdk/src/hal/thread.cpp
	sdk/src/arch/linux/net_serial.cpp
        sdk/src/arch/linux/net_socket.cpp
        sdk/src/arch/linux/timer.cpp 
	)

 INCLUDE_DIRECTORIES(
	# 工程库，采用相对地址	
	include
	# ICP算法位置
	icp_opencv/icp-opencv
	icp_opencv/third_party/kdtree
	# Rplidar SDK
	sdk
	sdk/include
	sdk/src
	sdk/arch/linux
	sdk/arch/hal
	# 系统库
	 /usr/local/include 
 )  

LINK_DIRECTORIES(                                  #相关库文件的目录  
	/usr/local/lib
	/usr/lib
)


add_executable(Rplidar ${SRC})

target_link_libraries(Rplidar
	# OpenCV
	opencv_calib3d	
	opencv_core
	opencv_contrib
	opencv_features2d
	opencv_flann
	opencv_highgui
	opencv_imgproc

	# 系统库
	pthread
	dl
	rt
	z
	)

