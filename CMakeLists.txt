cmake_minimum_required(VERSION 3.5)
project(can_test)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/can
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# 設定 lib 搜尋路徑
link_directories(${PROJECT_SOURCE_DIR}/can)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)

# ======================
# 主節點編譯
# ======================
add_executable(can_node src/Tangair_usb2can.cpp src/can_node.cpp)
target_link_libraries(can_node 
    pthread
    usb_can
    ${CMAKE_THREAD_LIBS_INIT}  
    ${CMAKE_DL_LIBS}
)

add_executable(can_node_one_motor
    src/usb2can_one_motor.cpp    # ← 改為新的 cpp 檔案名稱
    src/can_node_one_motor.cpp
)

target_link_libraries(can_node_one_motor
    pthread
    rt
    usb_can
)

add_executable(can_node_motor_imu
    src/can_node_motor_imu.cpp    # ← 改為新的 cpp 檔案名稱
    src/usb2can_motor_imu.cpp
)

target_link_libraries(can_node_motor_imu
    pthread
    rt
    usb_can
    unitree_sdk2
)

add_executable(reddog_hardware 
    src/reddog_hardware.cpp)
    
target_link_libraries(reddog_hardware  
    unitree_sdk2
)

SET(CMAKE_BUILD_TYPE Release)

