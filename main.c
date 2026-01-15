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

/* Master 0, Slave 0, "TC200E TR CoE Drive"
 * Vendor ID:       0x00075500
 * Product code:    0x00000001
 * Revision number: 0x00000005
 * 使用sudo ethercat cstruct命令生成
 */
ec_pdo_entry_info_t slave_0_pdo_entries[] = {
    {0x6071, 0x00, 16},
    {0x6040, 0x00, 16},
    {0x607a, 0x00, 32},
    {0x60ff, 0x00, 32},
    {0x6060, 0x00, 8},
    {0x603f, 0x00, 16},
    {0x6041, 0x00, 16},
    {0x6064, 0x00, 32},
    {0x606c, 0x00, 32},
    {0x6077, 0x00, 16},
};

ec_pdo_info_t slave_0_pdos[] = {
    {0x1601, 5, slave_0_pdo_entries + 0}, /* 2st Receive  PDO Mapping */
    {0x1a01, 5, slave_0_pdo_entries + 5}, /* 2st Transmit PDO Mapping */
};

ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},
    {0xff}
};
/**
 * @brief 预定义的从站配置
 */
S_SlaveConfig slave_configs[] = {
    // 第一个从站设备: 伺服驱动器
    {
        .vendor_id = 0x00075500,
        .product_code = 0x00000001,
        .name = "TC Servo Drive",
        .sync_count = (sizeof(slave_0_syncs) / sizeof(slave_0_syncs[0])),
        .pdoEntryCount = (sizeof(slave_0_pdo_entries) / sizeof(slave_0_pdo_entries[0])),
        .pdos = slave_0_pdos,
        .syncs = slave_0_syncs,
        .pEntry = slave_0_pdo_entries,
    },
    // 第一个从站设备: xxxx

    // 添加更多从站配置...
};
/**
 * @brief Manage all EnterCAT hosts of this host
 */
S_EthercatMaster masters[D_MASTER_AMOUNT];


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
把这个函数和 void check_master_slave_state(void) 封装到一起
*/
static ec_slave_config_state_t ssc_ana_in_state = {};
int is_all_slave_op()
{
    int opCount = 0;

    ecrt_slave_config_state(masters[0].pslave_configs[0], &ssc_ana_in_state);
    if (ssc_ana_in_state.operational) {
        opCount++;
    }

    if ( opCount == 1 ) {return 1;}
    else {return 0;}
}

/*
 Pp 模式 测试任务
*/
void CiA402_Init(void) {
    // struct timespec    wakeup_time, time;
    static uint32_t     actual_position_value = 0;
    static uint32_t     actual_Speed_value = 0;
    static uint8_t      mode = 0;

    uint16_t            status_word;

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
            EC_WRITE_U16(masters[0].pdomain_pds[0] + masters[0].slave_offsets[0].ControlWord, 0x80);

            mode = 1;
        }else if (mode == 1) {
            // 2.使能电压, 快速停止, 设置Pp模式
            EC_WRITE_U8(masters[0].pdomain_pds[0] + masters[0].slave_offsets[0].OperationMode, (uint8_t)PpMode);
            EC_WRITE_U16(masters[0].pdomain_pds[0] + masters[0].slave_offsets[0].ControlWord, 0x06);

            status_word = masters[0].slave_values[0].StatusWord;
            if (status_word & 0x21) {
                printf("cyclic_task: mode1 statcode==%d\n", status_word);
                mode = 2;
            }
        }else if (mode == 2) {
            // 3.使能操作
            EC_WRITE_U16(masters[0].pdomain_pds[0] + masters[0].slave_offsets[0].ControlWord, 0x07);
            mode = 3;
        }else if (mode == 3) {
            // 4.
            actual_position_value = EC_READ_S32(masters[0].pdomain_pds[0] + masters[0].slave_offsets[0].ActualPos);
            actual_Speed_value = EC_READ_S32(masters[0].pdomain_pds[0] + masters[0].slave_offsets[0].ActualSpe);
            printf("cyclic_task: mode3 pos==%d, spe==%d\n", actual_position_value, actual_Speed_value);

            EC_WRITE_U16(masters[0].pdomain_pds[0] + masters[0].slave_offsets[0].ControlWord, 0x2F);
            mode = 4;
        }else if (mode == 4) {
            printf("Mode4\n");
            if (is_all_slave_op()) {
                
                mode = 4;
                actual_position_value = EC_READ_S32(masters[0].pdomain_pds[0] + masters[0].slave_offsets[0].ActualPos);
                actual_Speed_value = EC_READ_S32(masters[0].pdomain_pds[0] + masters[0].slave_offsets[0].ActualSpe);

                EC_WRITE_U32(masters[0].pdomain_pds[0] + masters[0].slave_offsets[0].TargetPosition, 0);
                
                EC_WRITE_U32(ecrt_sdo_request_data(psdo_profile_velocity), 1000000);
                ecrt_sdo_request_write(psdo_profile_velocity);

                // EC_WRITE_U32(ecrt_sdo_request_data(psdo_profile_acce), 100);
                // ecrt_sdo_request_write(psdo_profile_velocity);
                // EC_WRITE_U32(ecrt_sdo_request_data(psdo_profile_dece), 100);
                // ecrt_sdo_request_write(psdo_profile_velocity);

                EC_WRITE_U16(masters[0].pdomain_pds[0] + masters[0].slave_offsets[0].ControlWord, 0x3F);
                
                printf("cyclic_task: mode4 pos==%d, spe==%d\n", actual_position_value, actual_Speed_value);
                printf("-------------is_all_slave_op-------------\n");
            }
        }

        printf("CiA402_test Mode        = %d\n", mode);
        printf("CiA402_test StatusWord  = %Xh\n", masters[0].slave_values[0].StatusWord);
        printf("CiA402_test ControlWord = %Xh\n", masters[0].slave_values[0].ControlWord);
        printf("CiA402_test ErrorStatus = %Xh\n", masters[0].slave_values[0].ErrorStatus);
    }
    
    // write process data
    

    masters[0].slave_values[0].StatusWord = EC_READ_U16(masters[0].pdomain_pds[0] + masters[0].slave_offsets[0].StatusWord);
    masters[0].slave_values[0].ControlWord = EC_READ_U16(masters[0].pdomain_pds[0] + masters[0].slave_offsets[0].ControlWord);
    masters[0].slave_values[0].ErrorStatus = EC_READ_U16(masters[0].pdomain_pds[0] + masters[0].slave_offsets[0].ErrorStatus);

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