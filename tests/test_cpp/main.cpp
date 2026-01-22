/* 
CPP封装层
*/
#include <string>

#include "main.hpp"
#include "nmxrt/publisher.hpp"

int TcTorsoDevice::teach(TorsoCmd dir, float step, float vel) {

    
    return 0;
}
int TcTorsoDevice::move_joint(float pos, float vel, float acc, bool blocking, int opts) {


    return 0;
}

TorsoState TcTorsoDevice::get_joint_state(void) {
    TorsoState State;
    State.name = "test";
    State.pos = 1;

    return State;
}

int main() 
{
    TcTorsoDevice test;

    return 0;
}
