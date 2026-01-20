set(TEST_NAME c_igh_test)

add_executable(${TEST_NAME} main.c)

#[[ 
1. 编译标准/选项自动继承, [头文件路径 核心库的target_include_directories已暴露(PUBLIC)]
]]

# 2. 链接用到的库: ethercat库, igh_coe_motor库
target_link_libraries(${TEST_NAME}
    igh_coe_motor
    ethercat        # libethercat.so
)

