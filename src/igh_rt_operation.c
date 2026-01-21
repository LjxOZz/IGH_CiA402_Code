
#include <sched.h>
#include <pthread.h> 
#include <sys/mman.h>
#include <sys/prctl.h>
#include <stdbool.h>
#include <unistd.h>

#include "igh_rt_operation.h"
#include "igh_coe_motor.h"

#define dc_user     // 开启dc时钟

#define NSEC_PER_SEC (1000000000L)
#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
                       (B).tv_nsec - (A).tv_nsec)   // 计算时间差(ns)
#define RT_THREAD_CPU_CORE      1                   // 绑定的CPU核心（固定值，便于修改）
#define MAX_CONTINUOUS_TIMEOUT  2                   // 最大连续超时次数


/* ===================== rt_thread 全局变量(实时任务统计/控制) ===================== */
// 多线程访问，需加锁
volatile uint64_t period_ns = 0;                // 单次周期耗时(ns)
volatile uint64_t period_min_ns = UINT64_MAX;   // 周期耗时最小值(ns)
volatile uint64_t period_max_ns = 0;            // 周期耗时最大值(ns)

volatile uint64_t exec_ns = 0;                  // 任务执行耗时(ns)
volatile uint64_t exec_min_ns = UINT64_MAX;     // 执行耗时最小值(ns)
volatile uint64_t exec_max_ns = 0;              // 执行耗时最大值(ns)

volatile uint64_t timeOutCount = 0;             // 超时总次数
volatile uint32_t contTimeCount = 0;            // 连续超时计数
volatile uint32_t threadConTimeOut = 0;         // 连续超时阈值触发计数
volatile bool threadQuitFlag = false;           // 线程退出标志
static short sPeriodCount = 0;

static int sPrintCount = 0;
/* ===================== CiA402_Init 全局变量(实时任务统计/控制) ===================== */
static unsigned int counter_01s = 0;
static unsigned int counter_10s = 0;
struct timespec dctime;
uint64_t apptime = 0;
unsigned int sync_ref_counter = 0;


extern S_EthercatMaster masters[D_MASTER_AMOUNT];
extern S_SlaveConfig slave_configs[];

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

void *rt_thread(void *arg)
{
    prctl(PR_SET_NAME, "rt thread");                        // 设置线程名字
    /* 将当前线程限制为仅在指定处理器上运行 */
    cpu_set_t cpuSet;
    CPU_ZERO(&cpuSet);
    CPU_SET(RT_THREAD_CPU_CORE, &cpuSet);
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuSet) != 0) {
        perror("failed to pthread_setaffinity_np\n");
    }

    struct timespec *setPeriod = (struct timespec *)arg;    // 获取周期参数
    printf("\n====rt-thread====\n");

    // ===================== 2. 局部变量初始化（减少全局变量访问） =====================
    struct timespec local_last_start = {0, 0};
    struct timespec local_start;
    struct timespec local_task_start;
    struct timespec local_task_end;
    struct timespec local_next;
    uint64_t local_period_ns;
    uint64_t local_exec_ns;
    uint64_t local_process_time;
    bool local_quit_flag = false;
    uint32_t local_cont_timeout = 0;

    while (!local_quit_flag) {
        sPrintCount ++;
        
        clock_gettime(CLOCK_MONOTONIC, &local_start);
        
        if (sPeriodCount == 0) {
            local_last_start = local_start;
            sPeriodCount = 1;
        } else {
            local_period_ns = DIFF_NS(local_last_start, local_start);
            period_ns = local_period_ns;
            // 更新周期最值
            if (local_period_ns < period_min_ns) period_min_ns = local_period_ns;
            if (local_period_ns > period_max_ns) period_max_ns = local_period_ns;
        }

        // -------------------- 核心任务执行 --------------------
        clock_gettime(CLOCK_MONOTONIC, &local_task_start);
        CiA402_Init();
        // cyclic_task();
        clock_gettime(CLOCK_MONOTONIC, &local_task_end);

        // 计算任务执行耗时
        local_exec_ns = DIFF_NS(local_task_start, local_task_end);
        exec_ns = local_exec_ns; // 更新全局统计
        // 更新执行耗时最值
        if (local_exec_ns < exec_min_ns) exec_min_ns = local_exec_ns;
        if (local_exec_ns > exec_max_ns) exec_max_ns = local_exec_ns;
        // -------------------- 超时检查 & 精确睡眠 --------------------
        // 计算本次循环总耗时（从开始到任务结束）
        local_process_time = DIFF_NS(local_start, local_task_end);
        
        if (local_process_time >= setPeriod->tv_nsec) {
            // 超时处理：记录统计，触发退出逻辑
            timeOutCount++;
            local_cont_timeout++;
            contTimeCount = local_cont_timeout; // 更新全局
            
            // 连续超时超过阈值，标记退出
            if (local_cont_timeout >= MAX_CONTINUOUS_TIMEOUT) {
                threadConTimeOut++;
                local_quit_flag = true; // 局部标记，减少全局变量竞争
                threadQuitFlag = true;
            }
        } else {
            // 未超时：重置连续超时计数，计算精确睡眠时间
            local_cont_timeout = 0;
            contTimeCount = 0;

            // 计算下一次唤醒时间（绝对时间，避免相对睡眠的误差累积）
            local_next = local_start;
            local_next.tv_nsec += setPeriod->tv_nsec;
            // 处理纳秒溢出（必做：避免tv_nsec >= 1e9导致错误）
            if (local_next.tv_nsec >= 1000000000) {
                local_next.tv_sec += local_next.tv_nsec / 1000000000;
                local_next.tv_nsec %= 1000000000;
            }

            // 精确睡眠（TIMER_ABSTIME：基于绝对时间，精度更高）
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &local_next, NULL);
        }

        // printf("period  %ld us\n", local_period_ns);
        // printf("exec    %ld us\n", local_exec_ns);
        // printf("process %ld us\n", local_process_time);

        // 更新上一次开始时间
        local_last_start = local_start;
    }
    return 0;
}


void custom_task(void) {
    if (sPrintCount >= 20000) {
        if (is_all_slave_op()) {
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
        }

        check_master_slave_state();
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


int rt_init(void) {
    /* 锁定内存 */
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        printf("Warning: Failed to lock memory\n");
        return -2;
    }

    int ret = 0;
    pthread_t rtThread;
    pthread_attr_t attr;
    struct sched_param param = {};
    pthread_t customThread;
    pthread_attr_t customAttr;
    struct sched_param customPara = {};

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
    struct timespec period;
    period.tv_sec = 0;
    period.tv_nsec = D_SM_TIME; // D_SM_TIME = 1000 * 1000 = 1ms
    //rt_thread: 实时线程，运行EtherCAT循环任务
    ret = pthread_create(&rtThread, &attr, &rt_thread, (void *)&period);
    if (0 != ret) {
        printf("pthread_create error ret = %d\n", ret);
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
    printf("rtThread create success! Waiting for rtThread to end");
    printf("\n<===============================================>\n");
    
    pthread_join(rtThread, NULL);

    munlockall();           // 解锁内存页

    return 0;
}

