set(CPP_SOURCE_DIR ${PROJECT_SOURCE_DIR}/cpp)
set(MSG_SOURCE_DIR ${PROJECT_SOURCE_DIR}/msg)

add_executable(ct_torso 
    ${CPP_SOURCE_DIR}/main.cpp 
    ${CPP_SOURCE_DIR}/ct_torso_device.cpp 
    ${MSG_SOURCE_DIR}/Joint.pb.cc)

target_link_libraries(ct_torso PRIVATE igh_coe nmxrt)

target_include_directories(ct_torso PRIVATE ${CPP_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/msg)
