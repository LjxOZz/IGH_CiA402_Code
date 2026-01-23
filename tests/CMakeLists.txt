
set(TESTS_SOURCE_DIR ${PROJECT_SOURCE_DIR}/tests)

add_executable(igh_test ${TESTS_SOURCE_DIR}/igh_coe_test.c)
target_link_libraries(igh_test PRIVATE igh_coe ethercat)
target_include_directories(igh_test PRIVATE ${PROJECT_BINARY_DIR})

# 去除cpp测试
# add_executable(cpp_test ${TESTS_SOURCE_DIR}/cpp_igh_test.cpp)
# target_link_libraries(cpp_test PRIVATE igh_coe nmxrt)
# target_include_directories(cpp_test PRIVATE ${PROJECT_BINARY_DIR} ${PROJECT_SOURCE_DIR}/cpp)


