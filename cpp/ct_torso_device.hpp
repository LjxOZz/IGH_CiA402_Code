/* 
CPP封装层
*/
#ifndef __CT_TORSO_DEVICE_HPP_
#define __CT_TORSO_DEVICE_HPP_

#include <iostream>
#include <string>

#include "igh_coe_motor.h"
#include "igh_rt_operation.h"

#include "nmxrt/publisher.hpp"

#include "Joint.pb.h"

/* nmxrt */


/* igh_coe */
extern S_EthercatMaster masters[D_MASTER_AMOUNT];
extern S_SlaveConfig slave_configs[];

class TcTorsoDevice 
{
public:

    nmx::msg::JointState State;
    nmx::rt::Publisher<nmx::msg::JointState> pub_msg;
    
    
    TcTorsoDevice():pub_msg("ct_motor0/JointState")
    {
        ecrt_init();
    }
    
    ~TcTorsoDevice() {}

    /**
     * @brief Start running thread
     *
     */
    void run(void);

    /**
     * @brief 位置步进
     * @param dir 示教方向
     * @param step 步进的距离
     * @param vel 速度
     * 
     * @return 
     */
    int teach(int dir, float step, float vel);

    /**
     * @brief 关节空间运动
     * @param pos 目标关节位置, 单位x
     * @param vel 目标关节速度, 
     * @param acc 目标关节加速度, 
     * @param blocking 是否阻塞等待运动完成
     * @param opts
     * 
     * @return 
     */
    int move_joint(float pos, float vel, float acc, bool blocking, int opts);

    /**
     * @brief Obtain joint status
     *
     * @return joint status
     */
    int get_joint_state(void);

private:
    // 可能的内部状态

};

extern TcTorsoDevice test;

typedef void* (*thread_func_ptr)(void*);

extern "C" {
    int create_custom_thread(thread_func_ptr thread_func);
}

#endif
