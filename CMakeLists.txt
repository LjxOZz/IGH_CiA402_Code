cmake_minimum_required(VERSION 3.10)

project(ct_igh_sdk VERSION 0.1 LANGUAGES C CXX)
# cpp
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# c
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# 查找库
find_package(zenohc REQUIRED)
find_package(Protobuf REQUIRED)
find_package(spdlog REQUIRED)

find_package(nmxrt REQUIRED)

# 添加编译定义( 解决 CPU_ZERO/CPU_SET 宏未定义)


# add_compile_options(-O2 -g -Wall)

set(LIB_NAME igh_coe)

set(CORE_SOURCES 
    src/igh_coe_motor.c
    src/igh_rt_operation.c
)

add_library(${LIB_NAME} STATIC ${CORE_SOURCES})

target_compile_definitions(${LIB_NAME} PRIVATE _GNU_SOURCE)

target_link_libraries(${LIB_NAME} PUBLIC ethercat)

target_include_directories(${LIB_NAME} PUBLIC
    /include                        # igh库头文件路径
    ${PROJECT_SOURCE_DIR}/include   # 本地路径
)

# 进入tests目录构建测试程序
add_subdirectory(${PROJECT_SOURCE_DIR}/cpp ${PROJECT_BINARY_DIR}/cpp)
add_subdirectory(${PROJECT_SOURCE_DIR}/tests ${PROJECT_BINARY_DIR}/tests)
