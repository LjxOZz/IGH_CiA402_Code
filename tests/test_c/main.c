#define _GNU_SOURCE

#include <sched.h>
#include <unistd.h>
#include <sys/mman.h>
#include <errno.h>
#include <pthread.h>
#include <sys/prctl.h>
#include <time.h>
#include <stdint.h>

#include "main.h"

/*
EnterCAT 启动命令
sudo modprobe ec_master
sudo /etc/init.d/ethercat start
sudo modprobe ec_generic
*/


#define dc_user

#define D_TARGET_CPU 1


/****************************************************************************/
//时间
#define FREQUENCY 1000
#define NSEC_PER_SEC (1000000000L)
#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
                       (B).tv_nsec - (A).tv_nsec)
#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)
/*****************************************************************************/



/* **************************************************** 全局变量 **************************************************** */
#pragma pack(push, 1)   // 开启 1 字节对齐



static unsigned int counter_01s = 0;
static unsigned int counter_10s = 0;

#pragma pack(pop)   

/*****************************************************************************/
//线程, 实时
struct timespec period, cusTomPeriod;

short threadQuitFlag = 0;

uint32_t period_ns = 0;     //
uint32_t exec_ns = 0;       //rt task 的执行时间

uint32_t period_min_ns  = 1000000000, period_max_ns = 0;
uint32_t exec_min_ns    = 1000000000, exec_max_ns   = 0;

static short sPeriodCount = 0;
static int sPrintCount = 0;

struct timespec dctime;
uint64_t apptime = 0;
unsigned int sync_ref_counter = 0;

struct timespec startTime, endTime, lastStartTime;
struct timespec taskStartTime, taskEndTime;
struct timespec nextTime;

static int timeOutCount = 0;
static int contTimeCount = 0;
static int threadConTimeOut = 0;
/*****************************************************************************/
// 线程/任务函数
void *rt_thread(void *arg);
void *custom_thread(void *arg);

void cyclic_task();
void custom_task();

/*****************************************************************************/


/*****************************************************************************/




/*
 Pp 模式 测试任务
*/
void CiA402_Init(void) {
    // struct timespec    wakeup_time, time;
    static uint8_t  mode = 0;
    
    static int32_t  actual_position_value = 0;
    static int32_t  actual_speed_value = 0;
    static uint16_t status_word = 0;
    static uint32_t profile_velocity = 0;
    static int32_t  target_position_value = 0;

    /* receive process data */
    if (counter_10s) {counter_10s--;} /* 每10秒检查一次主站状态 */
    else {
        counter_10s = 1000 * 10;
        check_master_state(masters[0].pmaster);             // 检查主站状态
    }
    ecrt_master_receive(masters[0].pmaster);        // 从总线接收数据
    ecrt_domain_process(masters[0].pdomains[0]);    // 处理接收的数据
    check_domain_state(masters[0].pdomains[0]);     // 检查域状态

    
    
    /* 初始化状态机 */
    if (counter_01s) {counter_01s--;}
    else {
        counter_01s = 1000;

        if (mode == 0) {
            // 1.复位
            write_pdo_u16(masters[0].slave_offsets[0].ControlWord, 0x80);

            mode = 1;
        }else if (mode == 1) {
            // 2.使能电压, 快速停止, 设置Pp模式 
            write_pdo_u8(masters[0].slave_offsets[0].OperationMode, (uint8_t)PpMode);
            write_pdo_u16(masters[0].slave_offsets[0].ControlWord, 0x06);

            status_word = masters[0].slave_values[0].StatusWord;
            if (status_word & 0x21) {
                printf("cyclic_task: mode1 statcode==%d\n", status_word);
                mode = 2;
            }
        }else if (mode == 2) {
            // 3.使能操作
            write_pdo_u16(masters[0].slave_offsets[0].ControlWord, 0x07);

            mode = 3;
        }else if (mode == 3) {
            // 4.
            actual_position_value = read_pdo_s32(masters[0].slave_offsets[0].ActualPos);
            actual_speed_value = read_pdo_s32(masters[0].slave_offsets[0].ActualSpe);
            printf("cyclic_task: mode3 pos==%d, spe==%d\n", actual_position_value, actual_speed_value);

            write_pdo_u16(masters[0].slave_offsets[0].ControlWord, 0x2F);

            if (write_pdo_u32(masters[0].slave_offsets[0].TargetPosition, 0)) { // ??
                fprintf(stderr, "Failed TargetPosition write_sdo_u32");
            }
            if (write_sdo_u32(psdo_profile_velocity, 100000)) {                 // ??
                fprintf(stderr, "Failed profile_velocity write_sdo_u32");
            }

            mode = 4;
        }else if (mode == 4) {
            printf("Mode4\n");
            if (is_all_slave_op()) {
                mode = 4;
                actual_position_value   = read_pdo_s32(masters[0].slave_offsets[0].ActualPos);
                actual_speed_value      = read_pdo_s32(masters[0].slave_offsets[0].ActualSpe);
                target_position_value   = read_pdo_s32(masters[0].slave_offsets[0].TargetPosition);

                profile_velocity = read_sdo_u32(psdo_profile_velocity);

                write_pdo_u16(masters[0].slave_offsets[0].ControlWord, 0x3F);
                printf("cyclic_task: mode4 tar_pos=%d now_pos=%d, tar_spe=%d now_spe==%d\n", 
                    target_position_value, actual_position_value, profile_velocity, actual_speed_value);

                // printf("-------------is_all_slave_op-------------\n");
            }
        }

        printf("CiA402_test Mode        = %d\n", mode);
        printf("CiA402_test StatusWord  = %Xh\n", masters[0].slave_values[0].StatusWord);
        printf("CiA402_test ControlWord = %Xh\n", masters[0].slave_values[0].ControlWord);
        printf("CiA402_test ErrorStatus = %Xh\n", masters[0].slave_values[0].ErrorStatus);
    }
    
    // write process data
    masters[0].slave_values[0].StatusWord = read_pdo_u16(masters[0].slave_offsets[0].StatusWord);
    masters[0].slave_values[0].ControlWord = read_pdo_u16(masters[0].slave_offsets[0].ControlWord);
    masters[0].slave_values[0].ErrorStatus = read_pdo_u16(masters[0].slave_offsets[0].ErrorStatus);

    /* */
#ifdef dc_user
    clock_gettime(CLOCK_MONOTONIC, &dctime);
    apptime = dctime.tv_sec * 1000000000 + dctime.tv_nsec;
    ecrt_master_application_time(masters[0].pmaster, apptime);

    if (sync_ref_counter) {
        sync_ref_counter--;
    }else {
        sync_ref_counter = 1;
        ecrt_master_sync_reference_clock(masters[0].pmaster);
    }
    ecrt_master_sync_slave_clocks(masters[0].pmaster);
#endif
    /* send process data */
    ecrt_domain_queue(masters[0].pdomains[0]);
    ecrt_master_send(masters[0].pmaster);
}


// 打印任务
void custom_task()
{
    if (sPrintCount >= 20000)
    {
        if (is_all_slave_op()) {
            // printf ns time
            printf("period     %d ... %d us\n",
                   (int)(period_min_ns / 1000.0), (int)(period_max_ns / 1000.0));
            printf("exec       %d ... %d us\n",
                   (int)(exec_min_ns / 1000.0), (int)(exec_max_ns / 1000.0));


            printf("master0 slave0==%d, statuscode = %d\n", 0, masters[0].slave_values[0].StatusWord);

            printf("\n");

            printf("timeOutCount===%d, continusTimeOut===%d, threadTimeOut==%d\n", 
                    timeOutCount, contTimeCount, threadConTimeOut);

            printf("\n");
        }

        check_master_slave_state();

        sPrintCount = 0;
    }
}

void *custom_thread(void *arg)
{
    printf("====custom-thread====\n");
    struct timespec *setPeriod = (struct timespec *)arg;

    uint32_t sleep_us = setPeriod->tv_sec * 1000000 + setPeriod->tv_nsec / 1000;

    while (1)
    {
        custom_task();
        usleep(sleep_us);
    }
}

/*

循环开始 → 时间测量 → 执行cyclic_task() → 统计性能 → 
├─ 如果超时：记录错误
└─ 如果正常：精确睡眠到下个周期 → 循环结束
*/
void *rt_thread(void *arg)
{

    /* 将当前线程限制为仅在指定处理器上运行 */
    cpu_set_t cpuSet;
    CPU_ZERO(&cpuSet);
    CPU_SET(D_TARGET_CPU, &cpuSet);
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuSet) != 0) {
        printf("failed to pthread_setaffinity_np\n");
        return NULL;
    }
    
    prctl(PR_SET_NAME, "rt thread");                        // 设置线程名字
    struct timespec *setPeriod = (struct timespec *)arg;    // 获取周期参数

    printf("====rt-thread====\n");

    while (1)
    {
        sPrintCount++;

        // if (is_slave_op()) 
        {

            clock_gettime(CLOCK_MONOTONIC, &startTime);
            if (sPeriodCount == 0) {
                lastStartTime = startTime;
            } else {
                period_ns = DIFF_NS(lastStartTime, startTime);  // 计算与上次的时间差
                lastStartTime = startTime;
            }

            clock_gettime(CLOCK_MONOTONIC, &taskStartTime);
            // do something...
            // cyclic_task();
            CiA402_Init();

            clock_gettime(CLOCK_MONOTONIC, &taskEndTime);
            endTime = taskEndTime;

            exec_ns = DIFF_NS(taskStartTime, taskEndTime);  //测量任务执行时间

            //如果处理时间超过设定的周期(1ms)  记录超时错误
            long int processTime = (endTime.tv_sec - startTime.tv_sec) * 1000000000 +
                                   (endTime.tv_nsec - startTime.tv_nsec);
            if (processTime >= setPeriod->tv_nsec) {    
                // printf("processTime====%ld,tv_nsec====%ld\n", processTime, setPeriod->tv_nsec);
                contTimeCount++;
                threadQuitFlag = 1;
                timeOutCount++;
                if (contTimeCount >= 2) {
                    threadConTimeOut++;
                }
            } else {    // 精确睡眠
                contTimeCount = 0;
                nextTime = startTime;
                // nextTime.tv_nsec += (setPeriod->tv_nsec - processTime);
                nextTime.tv_nsec += setPeriod->tv_nsec;
                if (nextTime.tv_nsec >= 1000000000) {
                    nextTime.tv_sec++;
                    nextTime.tv_nsec -= 1000000000;
                }
                clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &nextTime, NULL);
            }

            if (sPeriodCount == 1){
                if (period_ns < period_min_ns) {
                    period_min_ns = period_ns;
                }
                if (period_ns > period_max_ns) {
                    period_max_ns = period_ns;
                }
            }

            if (exec_ns < exec_min_ns) {
                exec_min_ns = exec_ns;
            } else if (exec_ns > exec_max_ns) {
                exec_max_ns = exec_ns;
            }

            // if (threadQuitFlag)
            // {
            //     printf("quit\n");
            //     break;
            // }

            if (sPeriodCount == 0) {
                sPeriodCount = 1;
            }

        }
    }
}

#define TEST
#ifdef TEST

int main(int argc, char **argv){
    // int ret = 0;


    ecrt_init();
    
    /* 线程相关 */
    pthread_t rtThread;
    pthread_t customThread;
    pthread_attr_t attr;
    pthread_attr_t customAttr;

    struct sched_param param = {};
    struct sched_param customPara = {};

    int ret = 0;
    /* 初始化线程 */
    ret = pthread_attr_init(&attr);
    if (0 != ret) {
        printf("pthread_attr_init error ret = %d\n", ret);
        return -1;
    }
    ret = pthread_attr_init(&customAttr);
    if (0 != ret) {
        printf("pthread_attr_init customAttr error ret = %d\n", ret);
        return -1;
    }
    /* 设置调度策略 */
    ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    if (0 != ret) {
        printf("pthread pthread_attr_setshedpolicy failed ret = %d\n", ret);
        return -1;
    }
    ret = pthread_attr_setschedpolicy(&customAttr, SCHED_OTHER);
    if (0 != ret) {
        printf("pthread pthread_attr_setshedpolicy customAttr failed ret = %d\n", ret);
        return -1;
    }

    /* 设置优先级 */
    param.sched_priority = 95;  //高
    ret = pthread_attr_setschedparam(&attr, &param);
    if (0 != ret) {
        printf("pthread pthread_attr_setschedparam failed ret = %d\n", ret);
        return -1;
    }
    customPara.sched_priority = 0;
    ret = pthread_attr_setschedparam(&customAttr, &customPara);
    if (0 != ret) {
        printf("pthread pthread_attr_setschedparam customPara failed ret = %d\n", ret);
        return -1;
    }

    /* 设置继承属性 */
    ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (0 != ret) {
        printf("thread pthread_attr_setinheritsched ret = %d\n", ret);
        return -1;
    }
    ret = pthread_attr_setinheritsched(&customAttr, PTHREAD_EXPLICIT_SCHED);
    if (0 != ret) {
        printf("thread pthread_attr_setinheritsched customAttr ret = %d\n", ret);
        return -1;
    }

    /* 设置周期时间 创建线程 */
    period.tv_sec = 0;
    period.tv_nsec = D_SM_TIME; // D_SM_TIME = 1000 * 1000 = 1ms
    //rt_thread: 实时线程，运行EtherCAT循环任务
    ret = pthread_create(&rtThread, &attr, &rt_thread, (void *)&period);
    if (0 != ret) {
        printf("pthread_create error ret = %d\n", ret);
        return -1;
    }

    cusTomPeriod.tv_sec = 0;
    cusTomPeriod.tv_nsec = 5000 * 1000; // 5000us = 5ms
    //custom_thread: 普通线程，用于打印状态信息
    ret = pthread_create(&customThread, &customAttr, &custom_thread, (void *)&cusTomPeriod);
    if (0 != ret) {
        printf("pthread_create custom_thread error ret = %d\n", ret);
        return -1;
    }

    //等待实时线程结束
    pthread_join(rtThread, NULL);

    munlockall();           // 解锁内存页

    printf("thread over\n");
    return 0;
}

#else
int main(int argc, char **argv) {


    printf("thread over\n");
    return 0;
}

#endif