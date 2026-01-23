
set(TEST_NAME cpp_igh_test)


add_executable(${TEST_NAME} main.cpp)

#[[ 
1. 编译标准/选项自动继承, [头文件路径 核心库的target_include_directories已暴露(PUBLIC)]
]]

target_include_directories(${LIB_NAME} PUBLIC
    /nmxrt/build/
)


# 2. 链接用到的库: ethercat库, igh_coe_motor库
target_link_libraries(${TEST_NAME} PUBLIC
    igh_lib
    ethercat            # libethercat.so
    nmxrt
)

