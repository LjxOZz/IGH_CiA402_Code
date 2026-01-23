/* 
CPP封装层

*/
#ifndef __MAIN_H
#define __MAIN_H

#include <iostream>
#include <string>

#include "igh_coe_motor.h"
#include "igh_rt_operation.h"

extern S_EthercatMaster masters[D_MASTER_AMOUNT];
extern S_SlaveConfig slave_configs[];


// class TorsoState {
// public:
//     std::string name;   // 关节名称
//     double pos;         // 关节反馈位置, 单位：rad or m
//     double vel;         // 关节反馈速度, 单位：rad/s or m/s
//     double acc;         // 关节反馈加速度, 单位：rad/s^2 or m/s^2
//     double effort;      // 关节反馈力矩, 单位：Nm
// };

class TcTorsoDevice {
private:
    // 可能的内部状态

public:
    TcTorsoDevice() {

        ecrt_init();

        //this->run();

    }
    
    ~TcTorsoDevice() {
        
    }

    void run(void) {
        // create_custom_thread();

        create_motor_thread();
    };

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
    void get_joint_state(void) {
        if (check_master_slave_state()) {
            get_status_flags = true;
            std::cout << "Spe:" << motor0_Speed << std::endl;
            std::cout << "Pos:" << motor0_Position << std::endl;
            std::cout << "Toq:" << motor0_Torque << std::endl;
        }
    };

};


#endif
