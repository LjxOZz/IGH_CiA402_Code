/* 
CPP封装层
*/
#include <sys/mman.h>
#include <unistd.h>

#include "nmxrt/publisher.hpp"

#include "ct_torso_device.hpp"

/*
extern "C" {

    void custom_task(void) {
        if (sPrintCount >= 5000) {

            if (check_master_slave_state()) {
                // printf ns time
                printf("period     %d ... %d us\n",
                    (int)(period_min_ns / 1000.0), (int)(period_max_ns / 1000.0));
                printf("exec       %d ... %d us\n",
                    (int)(exec_min_ns / 1000.0), (int)(exec_max_ns / 1000.0));
                printf("timeOutCount = %ld, continusTimeOut = %d, threadTimeOut = %d\n", 
                        timeOutCount, contTimeCount, threadConTimeOut);
                
                printf("\n");

                printf("master0 slave0 = %d, statuscode = %d\n", 0, masters[0].slave_values[0].StatusWord);

                printf("\n");
                printf("CiA402_test StatusWord  = %Xh\n", masters[0].slave_values[0].StatusWord);
                printf("CiA402_test ControlWord = %Xh\n", masters[0].slave_values[0].ControlWord);
                printf("CiA402_test ErrorStatus = %Xh\n", masters[0].slave_values[0].ErrorStatus);
                

            }
            sPrintCount = 0;
        }
    }

    void *custom_thread(void *arg)
    {
        printf("\n====custom-thread====\n");
        struct timespec *setPeriod = (struct timespec *)arg;

        uint32_t sleep_us = setPeriod->tv_sec * 1000000 + setPeriod->tv_nsec / 1000;

        while (1) {
            custom_task();
            usleep(sleep_us);
        }
    }

    int create_custom_thread(void) {

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
        ret = pthread_create(&customThread, &customAttr, &custom_thread, (void *)&cusTomPeriod);
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
*/