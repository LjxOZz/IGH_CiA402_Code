/* 
CPP封装层
*/
#include <sys/mman.h>
#include <unistd.h>

#include "ct_torso_device.hpp"


void *custom_thread(void *arg) {

    struct timespec *setPeriod = (struct timespec *)arg;
    uint32_t sleep_us = setPeriod->tv_sec * 1000000 + setPeriod->tv_nsec / 1000;

    printf("\n========custom thread start========\n");
    
    
    while (1) {
        if (sPrintCount >= 5000) {

            test.get_joint_state();

            test.pub_msg.publish(test.State);

            if (check_master_slave_state()) {   // printf ns time
                printf("EnterCAT: period     %d ... %d us\n",
                    (int)(period_min_ns / 1000.0), (int)(period_max_ns / 1000.0));
                printf("EnterCAT: exec       %d ... %d us\n",
                    (int)(exec_min_ns / 1000.0), (int)(exec_max_ns / 1000.0));
                printf("EnterCAT: timeOutCount = %ld, continusTimeOut = %d, threadTimeOut = %d\n", 
                    timeOutCount, contTimeCount, threadConTimeOut);
                printf("EnterCAT: master0 slave0 = %d, statuscode = %d\n", 0, masters[0].slave_values[0].StatusWord);
                printf("CiA402: StatusWord  = %Xh\n", masters[0].slave_values[0].StatusWord);
                printf("CiA402: ControlWord = %Xh\n", masters[0].slave_values[0].ControlWord);
                printf("CiA402: ErrorStatus = %Xh\n", masters[0].slave_values[0].ErrorStatus);
            }
            sPrintCount = 0;
        }
        usleep(sleep_us);
    }
}

extern "C" {

    int create_custom_thread(thread_func_ptr thread_func) {

        int ret = 0;

        pthread_t customThread;
        pthread_attr_t customAttr;
        struct sched_param customPara = {};

        ret = pthread_attr_init(&customAttr);
        if (0 != ret) {
            printf("pthread_attr_init customAttr error ret = %d\n", ret);
            return -1;
        }

        ret = pthread_attr_setschedpolicy(&customAttr, SCHED_OTHER);
        if (0 != ret) {
            printf("pthread pthread_attr_setshedpolicy customAttr failed ret = %d\n", ret);
            return -1;
        }

        customPara.sched_priority = 0;
        ret = pthread_attr_setschedparam(&customAttr, &customPara);
        if (0 != ret) {
            printf("pthread pthread_attr_setschedparam customPara failed ret = %d\n", ret);
            return -1;
        }

        ret = pthread_attr_setinheritsched(&customAttr, PTHREAD_EXPLICIT_SCHED);
        if (0 != ret) {
            printf("thread pthread_attr_setinheritsched customAttr ret = %d\n", ret);
            return -1;
        }

        struct timespec cusTomPeriod;
        cusTomPeriod.tv_sec = 0;
        cusTomPeriod.tv_nsec = 5000 * 1000; // 5000us = 5ms
        //custom_thread: 普通线程，用于打印状态信息
        ret = pthread_create(&customThread, &customAttr, thread_func, (void *)&cusTomPeriod);
        if (0 != ret) {
            printf("pthread_create custom_thread error ret = %d\n", ret);
            return -1;
        }

        printf("\n<===============================================>\n");
        printf("customThread create success!");
        printf("\n<===============================================>\n");

        return 0;
    }

}

void TcTorsoDevice::run(void) {
    create_custom_thread(custom_thread);

    create_motor_thread();
}

int TcTorsoDevice::teach(int dir, float step, float vel) {

    return 0;
}

int TcTorsoDevice::move_joint(float pos, float vel, float acc, bool blocking, int opts) {

    return 0;
}

int TcTorsoDevice::get_joint_state(void) {
    int ret = 0;
    if (ret = check_master_slave_state()) {
        get_status_flags = true;
        std::cout << "Spe:" << motor0_Speed << std::endl;
        std::cout << "Pos:" << motor0_Position << std::endl;
        std::cout << "Toq:" << motor0_Torque << std::endl;

        State.set_name("ct_motor0_state");
        State.set_pos(motor0_Position);
        State.set_vel(motor0_Speed);
        State.set_effort(motor0_Torque);
        State.set_acc(0.0);
    }
    return ret;
};

