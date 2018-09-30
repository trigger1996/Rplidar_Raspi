cmake_minimum_required(VERSION 3.10)

##
# Lidar SDK Settings
set(lidar_sdk_dir
    lidar_sdk
)

set(lidar_sdk_src
    ${lidar_sdk_dir}/src/rplidar_driver.cpp
    ${lidar_sdk_dir}/src/arch/linux/net_serial.cpp
    ${lidar_sdk_dir}/src/arch/linux/net_socket.cpp
    ${lidar_sdk_dir}/src/arch/linux/timer.cpp
    ${lidar_sdk_dir}/src/hal/thread.cpp
)

set(lidar_sdk_include_dir
    ${lidar_sdk_dir}/include
    ${lidar_sdk_dir}/src
    ${lidar_sdk_dir}/src/arch/linux/
    ${lidar_sdk_dir}/src/hal
)

##
# opencv
find_package(OpenCV 2.4.9 REQUIRED)

##
# pthread
find_package(Threads)

##
# main
include_directories(
    ${lidar_sdk_include_dir}
    ${OpenCV_INCLUDE_DIRS}    

    icp_opencv/icp-opencv/
)

add_executable(lidar
    ${lidar_sdk_src}

    icp_opencv/icp-opencv/icp.c
    icp_opencv/third_party/kdtree/kdtree.c

    lidar_driver.cpp
    icp_interface.cpp
    grid_mapping.cpp
    main.cpp
)


target_link_libraries(lidar 
    ${CMAKE_THREAD_LIBS_INIT}
    ${OpenCV_LIBS}
)
