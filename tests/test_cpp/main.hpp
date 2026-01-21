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


class TorsoCmd {
public:
    std::string name;   // 关节名称
    double pos;         // 关节目标位置, 单位：rad or m
    double vel;         // 关节目标速度, 单位：rad/s or m/s
    double acc;         // 关节目标加速度, 单位：rad/s^2 or m/s^2
    double effort;      // 关节目标力矩, 单位：Nm
};

class TorsoState {
public:
    std::string name;   // 关节名称
    double pos;         // 关节反馈位置, 单位：rad or m
    double vel;         // 关节反馈速度, 单位：rad/s or m/s
    double acc;         // 关节反馈加速度, 单位：rad/s^2 or m/s^2
    double effort;      // 关节反馈力矩, 单位：Nm
};

class TcTorsoDevice {
private:
    // 可能的内部状态

public:
    TcTorsoDevice() {

        ecrt_init();
        rt_init();

    }
    
    ~TcTorsoDevice() {
        
    }
    
    /**
     * @brief 位置步进
     * @param dir 示教方向
     * @param step 步进的距离
     * @param vel 速度
     * 
     * @return 
     */
    int teach(TorsoCmd dir, float step, float vel);

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
    TorsoState get_joint_state(void);

};


#endif
