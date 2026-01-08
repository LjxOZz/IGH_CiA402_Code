# 指定CMake最低版本要求
cmake_minimum_required(VERSION 3.10)

project(rt_posix_demo C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# -------------------------- 编译选项配置 --------------------------
add_compile_options(
    -O2
    -g
    -Wall
)

# -------------------------- 头文件路径配置 --------------------------
include_directories(
    .           # 当前目录
    ./include   # 包含test.hpp
    /include    # EtherCAT头文件路径
)

# -------------------------- 源文件配置（核心：包含所有文件） --------------------------
set(SOURCES
    ./main.c
    ./src/test.c
)

set(HEADERS
    ./main.c
    ./include/test.h
)

# -------------------------- 链接库配置 --------------------------
# link_directories(/usr/local/etherlab/lib) # 根据实际路径启用

# 要链接的库列表
set(LIBS
    ethercat  # -lethercat
    # pthread   # -lpthread
    # rt        # -lrt
)

# -------------------------- 生成可执行文件 --------------------------
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# 链接所需的库
target_link_libraries(${PROJECT_NAME} ${LIBS})

# -------------------------- 自定义目标 --------------------------
add_custom_target(info
    COMMAND echo "CFLAGS: ${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_RELEASE}"
    COMMAND echo "CXXFLAGS: ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE}" # 新增：显示C++编译选项
    COMMAND echo "LDFLAGS: ${CMAKE_EXE_LINKER_FLAGS}"
    COMMAND echo "Xenomai version:" && [ -x "$(command -v xeno-config)" ] && xeno-config --version || echo "xeno-config not found"
    COMMAND echo "EtherCAT library:" && ls -la /usr/local/ethercat/lib/libethercat.* 2>/dev/null || echo "EtherCAT library not found"
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Showing build information"
)