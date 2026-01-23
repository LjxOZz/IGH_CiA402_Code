set(CPP_SOURCE_DIR ${PROJECT_SOURCE_DIR}/cpp)

add_executable(ct_torso ${CPP_SOURCE_DIR}/main.cpp ${CPP_SOURCE_DIR}/ct_torso_device.cpp)

target_link_libraries(ct_torso PRIVATE igh_coe nmxrt)

target_include_directories(ct_torso PRIVATE ${PROJECT_SOURCE_DIR}/cpp)
