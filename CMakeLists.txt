cmake_minimum_required(VERSION 3.5)
project(can_test)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/can
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# 設定 lib 搜尋路徑
link_directories(${PROJECT_SOURCE_DIR}/can)

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

