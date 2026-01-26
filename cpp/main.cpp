#include <string>
#include <sys/mman.h>

#include "ct_torso_device.hpp"

#include "Joint.pb.h"


TcTorsoDevice test;

nmx::rt::Publisher<std::string> pub1("ct_motor0/state/speed");
nmx::rt::Publisher<std::string> pub2("ct_motor0/state/postion");
nmx::rt::Publisher<std::string> pub3("ct_motor0/state/torque");

nmx::rt::Publisher<nmx::msg::JointState> pub_msg("nmx/rt/test/point");




int main(void) 
{   
    nmx::msg::JointState p;
    p.set_name("shoulder_joint");
    p.set_pos(1.57);
    p.set_vel(0.1);
    p.set_acc(0.0);
    p.set_effort(2.5);
    
    test.run();

    while (1) 
    {
        sleep(1);
    }

    return 0;
}
