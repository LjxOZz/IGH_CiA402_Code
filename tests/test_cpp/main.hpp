/* 
CPP封装层

*/
#ifndef __MAIN_H
#define __MAIN_H

#include "igh_coe_motor.h"


extern S_EthercatMaster masters[D_MASTER_AMOUNT];
extern S_SlaveConfig slave_configs[];


class Torso {
private:
    // 可能的内部状态

public:
    Torso() {
        // 初始化逻辑
        ecrt_init();
        
    }
    
    ~Torso() {
        // 清理逻辑
    }
    
    int write(uint8_t value) {
        return write_pdo_u8(masters[0].slave_offsets[0].OperationMode, (uint8_t)value);
    }

};


#endif
