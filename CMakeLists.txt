# [[    目标 = 库 或 可执行文件     ]]
cmake_minimum_required(VERSION 3.10)

project(ct_igh_sdk VERSION 0.1 LANGUAGES C CXX)
# cpp
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# c
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package(zenohc REQUIRED)
find_package(Protobuf REQUIRED)
find_package(spdlog REQUIRED)

find_package(nmxrt REQUIRED)

# add_compile_options(-O2 -g -Wall)
# [[    静态库:开始构建    ]]

set(LIB_NAME igh_lib)

# 核心库源码
set(CORE_SOURCES 
    src/igh_coe_motor.c
    src/igh_rt_operation.c
)

# 查找nmxrt库 (make install后)


# 构建核心静态库
add_library(${LIB_NAME} STATIC ${CORE_SOURCES})

# 添加编译定义( 解决 CPU_ZERO/CPU_SET 宏未定义)
target_compile_definitions(${LIB_NAME} PRIVATE _GNU_SOURCE)


# 头文件路径: 全局生效(指令之后的所有「目标」都继承这个路径)
# include_directories(
#     /include                        # igh库头文件路径
#     ${PROJECT_SOURCE_DIR}/include   # 本地路径
# )

# 头文件路径: 仅对指定「目标」生效，局部可控
target_include_directories(${LIB_NAME} PUBLIC
    /include                        # igh库头文件路径
    ${PROJECT_SOURCE_DIR}/include   # 本地路径
)

# [[    静态库:结束构建    ]]

# 进入tests目录构建测试程序
add_subdirectory(tests/test_c)
add_subdirectory(tests/test_cpp)
