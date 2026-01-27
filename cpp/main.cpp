#include <string>
#include <sys/mman.h>

#include "ct_torso_device.hpp"

TcTorsoDevice test;

int main(void) 
{   
    
    test.run();

    while (1) 
    {
        sleep(1);
    }

    return 0;
}
