#define _GNU_SOURCE
#include <sched.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/mman.h>
#include <errno.h>
#include <pthread.h>
#include <sys/prctl.h>
#include <time.h>
#include <stdint.h>

#include "ecrt.h"

enum SERON_OPERATION_MODE
{
    NoMode = 0,
    PpMode = 1,
    PvMode = 3,
    TqMode = 4,
    HmMode = 6,
    IpMode = 7,
    CspMode = 8,
    CsvMode = 9,
    CstMode = 10,
};

typedef struct atomic_t
{
    volatile int counter;
} atomic_t;

#define dc_user

#define D_TARGET_CPU 1
#define D_PRIORITY 95


#define D_MAX_DOMAINS_PER_MASTER 1

#define D_MAX_SLAVES_PER_MASTER 32

#define D_SM_TIME 125 * 1000

#define D_SHITE_TIME (D_SM_TIME / 2)

#define TC 0x00075500, 0x00000002 // TC

static unsigned int counter = 0;
static unsigned int blink = 0;

/****************************************************************************/

#define FREQUENCY 1000

#define NSEC_PER_SEC (1000000000L)

#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
                       (B).tv_nsec - (A).tv_nsec)

#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

/*****************************************************************************/

/* Master 0, Slave 0, "TC200E CoE Drive"
 * Vendor ID:       0x00075500
 * Product code:    0x00000000
 * Revision number: 0x00000002
 */

#pragma pack(push, 1)

// offsets for PDO entries
typedef struct S_Ecat_Offset
{
    // write
    unsigned int TargetCurrent;  /*0x6071 int16*/
    unsigned int ControlWord;    /* ¿ØÖÆ×Ö    0x6040 uint16*/
    unsigned int TargetPosition; /* Ä¿±êÎ»ÖÃ  0x607A int32*/
    unsigned int TargetSpeed;    /* Ä¿±êËÙ¶È  0x60FF int32*/
    unsigned int OperationMode;  /* ¿ØÖÆÄ£Êœ  0x6060 uint8*/

    // read
    unsigned int Temperature;  /*0x300F uint16*/
    unsigned int TorqueSensor; /*0x303A current torque int32*/
    unsigned int ErrorStatus;  /*0x603F uint16*/
    unsigned int StatusWord;   /* ×ŽÌ¬×Ö    0x6041 uint16*/
    unsigned int ActPos;       /* ÊµŒÊÎ»ÖÃ  0x6064 int32*/
    unsigned int ActSpeed;     /* 0x606C int32*/
    unsigned int Current;      /*0x6077 int16*/
    unsigned int Voltage;      /*0x6079 uint32*/

} S_Ecat_Offset;

// offsets for PDO entries
typedef struct S_Ecat_Value
{
    // write
    int16_t TargetCurrent;  /* 0x6071 int16*/
    uint16_t ControlWord;   /* ¿ØÖÆ×Ö    0x6040 uint16*/
    int32_t TargetPosition; /* Ä¿±êÎ»ÖÃ  0x607A int32*/
    int32_t TargetSpeed;    /* Ä¿±êËÙ¶È  0x60FF int32*/
    uint8_t OperotionMode;  /* ¿ØÖÆÄ£Êœ  0x6060 uint8*/

    // read
    uint16_t Temperature; /*0x300F uint16*/
    int32_t TorqueSensor; /*0x303A current torque int32*/
    uint16_t ErrorStatus; /*0x603F uint16*/
    uint16_t StatusWord;  /* ×ŽÌ¬×Ö    0x6041 uint16*/
    int32_t ActPos;       /* ÊµŒÊÎ»ÖÃ  0x6064 int32*/
    int32_t ActSpeed;     /* 0x606C int32*/
    int16_t Current;      /*0x6077 int16*/
    uint32_t Voltage;     /*0x6079 uint32*/

} S_Ecat_Value;


//modified here
#define D_MASTER0_SLAVE_COUNT 7


S_Ecat_Offset s_Master0_EcatOffset[D_MASTER0_SLAVE_COUNT];
S_Ecat_Value s_Master0_EcatValue[D_MASTER0_SLAVE_COUNT];



// PDO注册条目结构
typedef struct S_PdoEntryConfig
{
    int master_index;
    int slave_position;
    uint32_t vendor_id;
    uint32_t product_code;
    uint16_t index;
    uint8_t subindex;
    uint8_t bit_length;
    unsigned int *offset_ptr;
} S_PdoEntryConfig;

//============================================================================//

ec_pdo_entry_info_t servo_pdo_entries[] = {
    //==========write================
    {0x6071, 0x00, 16}, /* uint16 */
    {0x6040, 0x00, 16}, /* uint16 */
    {0x607A, 0x00, 32}, /* int32 */
    {0x60FF, 0x00, 32}, /* int32 */
    {0x6060, 0x00, 8},  /* uint8 */

    //========read======================
    {0x300F, 0x00, 16}, /* uint16 */
    {0x303A, 0x00, 32}, /* int32 */
    {0x603F, 0x00, 16}, /* uint16 */
    {0x6041, 0x00, 16}, /* uint16 */
    {0x6064, 0x00, 32}, /* int32 */
    {0x606C, 0x00, 32}, /* int32 */
    {0x6077, 0x00, 16}, /* int16 */
    {0x6079, 0x00, 32}, /* uint32 */
};

ec_pdo_info_t servo_pdos[] = {
    // wirte
    {0x1601, 5, servo_pdo_entries + 0}, /* Channel 1 write*/
    // read
    {0x1a01, 8, servo_pdo_entries + 5}, /* Channel 2 read*/
};

ec_sync_info_t servo_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, servo_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, servo_pdos + 1, EC_WD_DISABLE},
    {0xff}};

/*************************modified by steven****************************************************** */

/*****************************Master slave config******************************************* */
// 定义从站配置
typedef struct S_Master_Slave_Conf
{
    int master_index;
    int slave_position;
    uint32_t vendor_id;
    uint32_t product_code;
    const char *description;
} S_Master_Slave_Conf;

//======================master0====================

// 实际的从站配置 - 根据您的硬件修改这里
//modified here
static S_Master_Slave_Conf master0_slave_conf[] = {
    // Master 0 - 伺服驱动器
    {0, 0, TC, "Master0-Slave0: TC Servo Drive 1"},
    {0, 1, TC, "Master0-Slave1: TC Servo Drive 2"},
    {0, 2, TC, "Master0-Slave1: TC Servo Drive 3"},
    {0, 3, TC, "Master0-Slave1: TC Servo Drive 4"},
    {0, 4, TC, "Master0-Slave1: TC Servo Drive 5"},
    {0, 5, TC, "Master0-Slave1: TC Servo Drive 6"},
    {0, 6, TC, "Master0-Slave1: TC Servo Drive 7"},
};


#define D_MASTER0_SLAVE_AMOUNT (sizeof(master0_slave_conf) / sizeof(master0_slave_conf[0]))


#define D_MASTER_AMOUNT 1

/*****************************Master slave config end*************************************************************** */

// 从站通用配置
typedef struct S_SlaveConfig
{
    uint32_t vendor_id;
    uint32_t product_code;
    const char *name;
    int sync_count;
    int pdoEntryCount;
    ec_pdo_info_t *pdos;
    ec_sync_info_t *syncs;
    ec_pdo_entry_info_t *pEntry;
} S_SlaveConfig;

// 预定义的从站配置
S_SlaveConfig slave_configs[] = {
    // 伺服驱动器
    {
        0x00075500,
        0x00000002,
        "TC Servo Drive",
        (sizeof(servo_syncs) / sizeof(servo_syncs[0])),
        (sizeof(servo_pdo_entries) / sizeof(servo_pdo_entries[0])),
        servo_pdos,
        servo_syncs,
        servo_pdo_entries,
    },
    // Beckhoff IO模块

    // 添加更多从站配置...
};

#define D_SLAVE_CONFIG_COUNT (sizeof(slave_configs) / sizeof(slave_configs[0]))

// 主站结构体
typedef struct S_EthercatMaster
{
    ec_master_t *pmaster;
    ec_domain_t *pdomains[D_MAX_DOMAINS_PER_MASTER];
    uint8_t *pdomain_pds[D_MAX_DOMAINS_PER_MASTER];

    int slave_count;
    ec_slave_config_t *pslave_configs[D_MAX_SLAVES_PER_MASTER];
    S_Ecat_Offset slave_offsets[D_MAX_SLAVES_PER_MASTER];
    S_Ecat_Value slave_values[D_MAX_SLAVES_PER_MASTER];

    ec_master_state_t master_state;
    ec_domain_state_t domain_states[D_MAX_DOMAINS_PER_MASTER];
} S_EthercatMaster;

static S_EthercatMaster masters[D_MASTER_AMOUNT];

#pragma pack(pop)

/*****************************************************************************/

// 全局PDO映射表
static S_PdoEntryConfig *pdo_entries = NULL;
static int pdo_entry_count = 0;
static int pdo_entry_capacity = 0;

// process data
static uint8_t *domain0_pd = NULL;
/****************************************************************************/

// EtherCAT master0
static ec_master_t *master0 = NULL;
static ec_master_state_t master0_state = {};

static ec_domain_t *domain0 = NULL;
static ec_domain_state_t domain0_state = {};

static ec_slave_config_t *sc_ana_in0 = NULL;
static ec_slave_config_state_t sc_ana_in_state0 = {};


static ec_domain_state_t sdomain_state = {};
static ec_master_state_t smaster_state = {};
static ec_slave_config_state_t ssc_ana_in_state = {};

/****************************************************************************/

struct timespec period, cusTomPeriod;
static atomic_t SystemClock;

short threadQuitFlag = 0;

uint32_t period_ns = 0;
uint32_t exec_ns = 0;

uint32_t period_min_ns = 1000000000, period_max_ns = 0;
uint32_t exec_min_ns = 1000000000, exec_max_ns = 0;

static short sPeriodCount = 0;

static int sPrintCount = 0;

struct timespec dctime;
uint64_t apptime = 0;
unsigned int sync_ref_counter = 0;

struct timespec startTime, endTime, lastStartTime;
struct timespec taskStartTime, taskEndTime;
struct timespec nextTime;

/*******************************************************************/
uint16_t ContollerWord = 0;
uint16_t statcode = 0;
int test = 0;
int master0_actpos[D_MASTER0_SLAVE_AMOUNT] = {0};
int master0_tempos[D_MASTER0_SLAVE_AMOUNT] = {0};


int mode = 0;
int actv = 0;
int state_561 = 0;
int state_563 = 0;
int32_t setSpeed = 100;

static int timeOutCount = 0;
static int contTimeCount = 0;
static int threadConTimeOut = 0;

//=========================================================================

void check_domain0_state(void);
void check_master0_state(void);
void check_master0_slave_state(void);
int is_master0_slave_op();


void *rt_thread(void *arg);
void *custom_thread(void *arg);

void cyclic_task();
void custom_task();

void init_ethercat_value();

// 初始化主站
int init_master(int master_index);

// 获取从站配置
const S_SlaveConfig *get_slave_config(uint32_t vendor_id, uint32_t product_code);

// 配置从站
int configure_slave(int master_index, int slave_position,
                    uint32_t vendor_id, uint32_t product_code);

// 注册PDO条目
int register_domain_pdo_entries(int master_index, int domain_index);

// 激活主站
int activate_master(int master_index);

// 获取主站指针
S_EthercatMaster *get_master(int master_index);

// 初始化从站值
void init_slave_values();

// 添加PDO条目到映射表
int add_pdo_entry(int master_index, int slave_position,
                  uint32_t vendor_id, uint32_t product_code,
                  uint16_t index, uint8_t subindex, uint8_t bit_length,
                  unsigned int *offset_ptr);

// 为主站和从站生成PDO注册数组
ec_pdo_entry_reg_t *generate_domain_regs(int master_index, int domain_index, int *reg_count);

// 清理PDO映射表
void cleanup_pdo_mapping();

// 配置伺服驱动器的PDO映射
int configure_servo_pdo_mapping(int master_index, int slave_position, uint32_t vendor_id, uint32_t product_code);

// 自动配置从站的PDO映射
int auto_configure_slave_pdo_mapping(int master_index, int slave_position,
                                     uint32_t vendor_id, uint32_t product_code);

int configure_dc_time(int master_index, int slave_position);

void check_domain_state(ec_domain_t *pdomain);

void check_master_state(ec_master_t *pmaster);

int is_all_slave_op();

int check_master_slave_state();

// 初始化主站
int init_master(int master_index)
{
    if (master_index >= D_MASTER_AMOUNT)
        return -1;

    masters[master_index].pmaster = ecrt_request_master(master_index);
    if (!masters[master_index].pmaster)
    {
        fprintf(stderr, "Failed to request master %d\n", master_index);
        return -1;
    }

    // 创建域
    for (int i = 0; i < D_MAX_DOMAINS_PER_MASTER; i++)
    {
        masters[master_index].pdomains[i] = ecrt_master_create_domain(masters[master_index].pmaster);
        if (!masters[master_index].pdomains[i])
        {
            fprintf(stderr, "Failed to create domain %d for master %d\n", i, master_index);
            return -1;
        }
    }

    return 0;
}

// 获取从站配置
const S_SlaveConfig *get_slave_config(uint32_t vendor_id, uint32_t product_code)
{
    for (int i = 0; i < D_SLAVE_CONFIG_COUNT; i++)
    {
        if (slave_configs[i].vendor_id == vendor_id &&
            slave_configs[i].product_code == product_code)
        {
            return &slave_configs[i];
        }
    }
    return NULL; // 未知从站类型
}

// config slave
int configure_slave(int master_index, int slave_position,
                    uint32_t vendor_id, uint32_t product_code)
{
    if (master_index >= D_MASTER_AMOUNT || slave_position >= D_MAX_SLAVES_PER_MASTER)
    {
        return -1;
    }

    S_EthercatMaster *pmaster = &masters[master_index];

    // 获取从站配置
    const S_SlaveConfig *pslave_config = get_slave_config(vendor_id, product_code);
    if (!pslave_config)
    {
        fprintf(stderr, "Unknown slave type: vendor_id=0x%08X, product_code=0x%08X\n",
                vendor_id, product_code);
        return -1;
    }

    // 创建从站配置
    pmaster->pslave_configs[slave_position] =
        ecrt_master_slave_config(pmaster->pmaster, 0, slave_position, vendor_id, product_code);

    if (!pmaster->pslave_configs[slave_position])
    {
        fprintf(stderr, "Failed to get slave config for slave %d\n", slave_position);
        return -1;
    }

    // 配置PDO
    if (ecrt_slave_config_pdos(pmaster->pslave_configs[slave_position],
                               pslave_config->sync_count, pslave_config->syncs))
    {
        fprintf(stderr, "Failed to configure PDOs for slave %d\n", slave_position);
        return -1;
    }

    // // 配置分布式时钟
    // ecrt_slave_config_dc(pmaster->slave_configs[slave_position], 0x300,
    //                     D_SM_TIME, D_SHITE_TIME, 0, 0);

    pmaster->slave_count++;
    printf("Configured slave %d: %s\n", slave_position, pslave_config->name);

    return 0;
}

// 注册PDO条目
int register_domain_pdo_entries(int master_index, int domain_index)
{
    if (master_index >= D_MASTER_AMOUNT || domain_index >= D_MAX_DOMAINS_PER_MASTER)
    {
        return -1;
    }

    int reg_count = 0;
    ec_pdo_entry_reg_t *domain_regs = generate_domain_regs(master_index, domain_index, &reg_count);

    if (!domain_regs)
    {
        printf("No PDO entries to register for master %d domain %d\n", master_index, domain_index);
        return 0;
    }

    if (ecrt_domain_reg_pdo_entry_list(masters[master_index].pdomains[domain_index],
                                       domain_regs))
    {
        fprintf(stderr, "PDO entry registration failed for master %d domain %d\n",
                master_index, domain_index);
        free(domain_regs);
        return -1;
    }

    printf("Registered %d PDO entries for master %d domain %d\n",
           reg_count, master_index, domain_index);

    free(domain_regs);
    return 0;
}

// 激活主站
int activate_master(int master_index)
{
    if (master_index >= D_MASTER_AMOUNT)
        return -1;

    if (ecrt_master_activate(masters[master_index].pmaster))
    {
        fprintf(stderr, "Failed to activate master %d\n", master_index);
        return -1;
    }

    // 获取域数据指针
    for (int i = 0; i < D_MAX_DOMAINS_PER_MASTER; i++)
    {
        masters[master_index].pdomain_pds[i] =
            ecrt_domain_data(masters[master_index].pdomains[i]);
        if (!masters[master_index].pdomain_pds[i])
        {
            fprintf(stderr, "Failed to get domain data for master %d domain %d\n",
                    master_index, i);
            return -1;
        }
    }

    printf("Master %d activated successfully\n", master_index);
    return 0;
}

// 获取主站指针
S_EthercatMaster *get_master(int master_index)
{
    if (master_index >= D_MASTER_AMOUNT)
        return NULL;
    return &masters[master_index];
}

// 初始化从站值
void init_slave_values()
{
    for (int master_idx = 0; master_idx < D_MASTER_AMOUNT; master_idx++)
    {
        S_EthercatMaster *pmaster = get_master(master_idx);
        for (int slave_idx = 0; slave_idx < pmaster->slave_count; slave_idx++)
        {

            // 初始化offset
            pmaster->slave_offsets[slave_idx].TargetCurrent = 0;
            pmaster->slave_offsets[slave_idx].ControlWord = 0;
            pmaster->slave_offsets[slave_idx].TargetPosition = 0;
            pmaster->slave_offsets[slave_idx].TargetSpeed = 100;
            pmaster->slave_offsets[slave_idx].OperationMode = CspMode;

            pmaster->slave_offsets[slave_idx].Temperature = 0;
            pmaster->slave_offsets[slave_idx].TorqueSensor = 0;
            pmaster->slave_offsets[slave_idx].ErrorStatus = 0;
            pmaster->slave_offsets[slave_idx].StatusWord = 0;
            pmaster->slave_offsets[slave_idx].ActPos = 0;
            pmaster->slave_offsets[slave_idx].ActSpeed = 0;
            pmaster->slave_offsets[slave_idx].Current = 0;
            pmaster->slave_offsets[slave_idx].Voltage = 0;

            // 初始化输出值
            pmaster->slave_values[slave_idx].TargetCurrent = 0;
            pmaster->slave_values[slave_idx].ControlWord = 0;
            pmaster->slave_values[slave_idx].TargetPosition = 0;
            pmaster->slave_values[slave_idx].TargetSpeed = 0;
            pmaster->slave_values[slave_idx].OperotionMode = 0;

            pmaster->slave_values[slave_idx].Temperature = 0;
            pmaster->slave_values[slave_idx].TorqueSensor = 0;
            pmaster->slave_values[slave_idx].ErrorStatus = 0;
            pmaster->slave_values[slave_idx].StatusWord = 0;
            pmaster->slave_values[slave_idx].ActPos = 0;
            pmaster->slave_values[slave_idx].ActSpeed = 0;
            pmaster->slave_values[slave_idx].Current = 0;
            pmaster->slave_values[slave_idx].Voltage = 0;

            // 初始化驱动器状态
            // drive_states[master_idx][slave_idx].state = 0;
            // drive_states[master_idx][slave_idx].control_word = 0;
            // drive_states[master_idx][slave_idx].target_position = 0;
            // drive_states[master_idx][slave_idx].actual_position = 0;
        }
    }
}

// 添加PDO条目到映射表
int add_pdo_entry(int master_index, int slave_position,
                  uint32_t vendor_id, uint32_t product_code,
                  uint16_t index, uint8_t subindex, uint8_t bit_length,
                  unsigned int *offset_ptr)
{

    if (pdo_entry_count >= pdo_entry_capacity)
    {
        // 动态扩展数组
        int new_capacity = pdo_entry_capacity == 0 ? 100 : pdo_entry_capacity * 2;
        S_PdoEntryConfig *new_entries = realloc(pdo_entries, new_capacity * sizeof(S_PdoEntryConfig));
        if (!new_entries)
            return -1;

        pdo_entries = new_entries;
        pdo_entry_capacity = new_capacity;
    }

    S_PdoEntryConfig *entry = &pdo_entries[pdo_entry_count++];
    entry->master_index = master_index;
    entry->slave_position = slave_position;
    entry->vendor_id = vendor_id;
    entry->product_code = product_code;
    entry->index = index;
    entry->subindex = subindex;
    entry->bit_length = bit_length;
    entry->offset_ptr = offset_ptr;

    return 0;
}

// 为主站和从站生成PDO注册数组
ec_pdo_entry_reg_t *generate_domain_regs(int master_index, int domain_index, int *reg_count)
{
    int count = 0;

    // 首先计算数量
    for (int i = 0; i < pdo_entry_count; i++)
    {
        if (pdo_entries[i].master_index == master_index)
        {
            count++;
        }
    }

    if (count == 0)
    {
        *reg_count = 0;
        return NULL;
    }

    // 分配内存
    ec_pdo_entry_reg_t *regs = calloc(count + 1, sizeof(ec_pdo_entry_reg_t));
    if (!regs)
        return NULL;

    // 填充数据
    int reg_index = 0;
    for (int i = 0; i < pdo_entry_count; i++)
    {
        if (pdo_entries[i].master_index == master_index)
        {
            regs[reg_index].alias = 0;
            regs[reg_index].position = pdo_entries[i].slave_position;
            regs[reg_index].vendor_id = pdo_entries[i].vendor_id;
            regs[reg_index].product_code = pdo_entries[i].product_code;
            regs[reg_index].index = pdo_entries[i].index;
            regs[reg_index].subindex = pdo_entries[i].subindex;
            regs[reg_index].offset = pdo_entries[i].offset_ptr;
            regs[reg_index].bit_position = NULL;
            reg_index++;
        }
    }

    // 结束标记
    regs[reg_index].index = 0;

    *reg_count = reg_index;
    return regs;
}

// 清理PDO映射表
void cleanup_pdo_mapping()
{
    if (pdo_entries)
    {
        free(pdo_entries);
        pdo_entries = NULL;
        pdo_entry_count = 0;
        pdo_entry_capacity = 0;
    }
}

// 配置伺服驱动器的PDO映射
int configure_servo_pdo_mapping(int master_index, int slave_position, uint32_t vendor_id, uint32_t product_code)
{
    S_EthercatMaster *pmaster = get_master(master_index);
    if (!pmaster || slave_position >= pmaster->slave_count)
        return -1;

    S_Ecat_Offset *poffsets = &pmaster->slave_offsets[slave_position];

    // 获取从站配置
    const S_SlaveConfig *pslave_config = get_slave_config(vendor_id, product_code);
    if (!pslave_config)
    {
        fprintf(stderr, "Unknown slave type: vendor_id=0x%08X, product_code=0x%08X\n",
                vendor_id, product_code);
        return -1;
    }

    // 创建成员指针数组
    unsigned int *member_ptrs[] = {
        &poffsets->TargetCurrent,
        &poffsets->ControlWord,
        &poffsets->TargetPosition,
        &poffsets->TargetSpeed,
        &poffsets->OperationMode,

        &poffsets->Temperature,
        &poffsets->TorqueSensor,
        &poffsets->ErrorStatus,
        &poffsets->StatusWord,
        &poffsets->ActPos,
        &poffsets->ActSpeed,
        &poffsets->Current,
        &poffsets->Voltage

    };

    int entryCount = pslave_config->pdoEntryCount;

    for (int i = 0; i < entryCount; i++)
    {
        uint16_t pdoIndex = pslave_config->pEntry[i].index;
        uint8_t pdoSubIndex = pslave_config->pEntry[i].subindex;
        uint8_t bitLen = pslave_config->pEntry[i].bit_length;
        add_pdo_entry(master_index, slave_position, vendor_id, product_code, pdoIndex, pdoSubIndex, bitLen, member_ptrs[i]);
    }

    return 0;
}

// 自动配置从站的PDO映射
int auto_configure_slave_pdo_mapping(int master_index, int slave_position,
                                     uint32_t vendor_id, uint32_t product_code)
{

    // 根据从站类型自动配置PDO映射
    if (0x00075500 == vendor_id && 0x00000002 == product_code)
    {
        // TC伺服驱动器
        return configure_servo_pdo_mapping(master_index, slave_position, vendor_id, product_code);
    }
    // else if (vendor_id == 0x00000002)
    // {
    //     // Beckhoff IO模块
    //     return configure_io_pdo_mapping(master_index, slave_position, product_code);
    // }
    // else
    // {
    //     // 默认配置为通用IO模块
    //     return configure_io_pdo_mapping(master_index, slave_position, product_code);
    // }
}

int configure_dc_time(int master_index, int slave_position)
{
    if (master_index >= D_MASTER_AMOUNT || slave_position >= D_MAX_SLAVES_PER_MASTER)
    {
        return -1;
    }

    S_EthercatMaster *pmaster = &masters[master_index];

    // 配置分布式时钟
    ecrt_slave_config_dc(pmaster->pslave_configs[slave_position], 0x300,
                         D_SM_TIME, D_SHITE_TIME, 0, 0);

    return 0;
}

void check_domain_state(ec_domain_t *pdomain)
{
    ec_domain_state_t ds;

    ecrt_domain_state(pdomain, &ds);

    if (ds.working_counter != sdomain_state.working_counter)
        // printf("Domain1: WC %u.\n", ds.working_counter);
        if (ds.wc_state != sdomain_state.wc_state)
            // printf("Domain1: State %u.\n", ds.wc_state);

            sdomain_state = ds;
}

void check_master_state(ec_master_t *pmaster)
{
    ec_master_state_t ms;

    ecrt_master_state(pmaster, &ms);

    if (ms.slaves_responding != smaster_state.slaves_responding)
        // printf("%u slave(s).\n", ms.slaves_responding);
        if (ms.al_states != smaster_state.al_states)
            // printf("AL states: 0x%02X.\n", ms.al_states);
            if (ms.link_up != smaster_state.link_up)
                // printf("Link is %s.\n", ms.link_up ? "up" : "down");

                smaster_state = ms;
}

void init_ethercat_value()
{
    for (int i = 0; i < D_MASTER0_SLAVE_AMOUNT; i++)
    {
        s_Master0_EcatOffset[i].ControlWord = 0;
        s_Master0_EcatOffset[i].OperationMode = 0;
        s_Master0_EcatOffset[i].TargetPosition = 0;
        s_Master0_EcatOffset[i].TargetSpeed = 0;
    }

}

int is_master0_slave_op()
{
    ecrt_slave_config_state(sc_ana_in0, &sc_ana_in_state0);
    if (sc_ana_in_state0.operational)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void check_master0_slave_state(void)
{
    ec_slave_config_state_t slave_state;
    ecrt_slave_config_state(sc_ana_in0, &slave_state);

    printf("master0*******Slave state: AL state=0x%02X, online=%d, operational=%d\n",
           slave_state.al_state, slave_state.online, slave_state.operational);
}

void check_domain0_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain0, &ds);

    if (ds.working_counter != domain0_state.working_counter)
        // printf("Domain1: WC %u.\n", ds.working_counter);
        if (ds.wc_state != domain0_state.wc_state)
            // printf("Domain1: State %u.\n", ds.wc_state);

            domain0_state = ds;
}

void check_master0_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master0, &ms);

    if (ms.slaves_responding != master0_state.slaves_responding)
        // printf("%u slave(s).\n", ms.slaves_responding);
        if (ms.al_states != master0_state.al_states)
            // printf("AL states: 0x%02X.\n", ms.al_states);
            if (ms.link_up != master0_state.link_up)
                // printf("Link is %s.\n", ms.link_up ? "up" : "down");

                master0_state = ms;
}

int is_all_slave_op()
{
    int opCount = 0;
    S_EthercatMaster *pmaster0 = get_master(0);
    for (int i = 0; i < D_MASTER0_SLAVE_AMOUNT; i++)
    {
        ecrt_slave_config_state(pmaster0->pslave_configs[i], &ssc_ana_in_state);
        if (ssc_ana_in_state.operational)
        {
            opCount++;
        }
    }

    if ((D_MASTER0_SLAVE_AMOUNT ) == opCount)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int check_master_slave_state()
{

    S_EthercatMaster *pmaster0 = get_master(0);
    for (int i = 0; i < D_MASTER0_SLAVE_AMOUNT; i++)
    {
        ec_slave_config_state_t slave_state;
        ecrt_slave_config_state(pmaster0->pslave_configs[i], &slave_state);

        printf("master0***i==%d****Slave state: AL state=0x%02X, online=%d, operational=%d\n",
               i, slave_state.al_state, slave_state.online, slave_state.operational);
    }

}


int temp = 0;
int temp1 = 0;

void cyclic_task()
{

    if (counter)
    {
        counter--;
    }
    else
    {
        counter = FREQUENCY * 10;

        for (int master_idx = 0; master_idx < D_MASTER_AMOUNT; master_idx++)
        {
            S_EthercatMaster *pmaster = get_master(master_idx);
            if (!pmaster)
                continue;

            check_master_state(pmaster->pmaster);
        }
    }

    for (int master_idx = 0; master_idx < D_MASTER_AMOUNT; master_idx++)
    {
        S_EthercatMaster *pmaster = get_master(master_idx);
        if (!pmaster)
            continue;

        // receive process data
        ecrt_master_receive(pmaster->pmaster);
        for (int domain_idx = 0; domain_idx < D_MAX_DOMAINS_PER_MASTER; domain_idx++)
        {
            if (pmaster->pdomains[domain_idx])
            {
                ecrt_domain_process(pmaster->pdomains[domain_idx]);

                check_domain_state(pmaster->pdomains[domain_idx]);
            }
        }
    }

    if (blink)
    {
        blink--;
    }
    else
    {
        blink = 3000;

        if (test == 0)
        {
            ContollerWord |= 0x0080;
            test = 1;
        }
        else if (test == 1)
        {
            ContollerWord = 0x06;

            int opCount = 0;
            for (int master_idx = 0; master_idx < D_MASTER_AMOUNT; master_idx++)
            {
                S_EthercatMaster *pmaster = get_master(master_idx);
                if (!pmaster)
                    continue;
                for (int i = 0; i < pmaster->slave_count; i++)
                {
                    statcode = pmaster->slave_values[i].StatusWord;
                    if (statcode & 0x21)
                    {
                        printf("master==master_idx==%d,slaveId==%d, statcode*******===%d\n", master_idx, i, statcode);
                        opCount++;
                        //                    test = 2;
                    }
                }
            }
            if ((D_MASTER0_SLAVE_AMOUNT) == opCount)
            {
                test = 2;
            }
        }
        else if (test == 2)
        {
            ContollerWord = 0x07;
            test = 3;
            for (int i = 0; i < D_MASTER0_SLAVE_AMOUNT; i++)
            {
                master0_actpos[i] = master0_tempos[i];
            }

            // printf("actpos = tempos = %d \n",tempos);
        }
        else if (test == 3)
        {
            ContollerWord = 0x0f;
            test = 4;
        }
        else if (test == 4)
        {
            test = 5;
        }
        else if (test == 5)
        {
            test = 6;
        }

        // printf("statuscode = %d \n",statcode);
        // printf("tempos = %d \n",tempos);
        // printf("actpos = %d \n",actpos);
    }

    if (6 == test)
    {
        printf("test====6\n");
        if (is_all_slave_op())
        {
            mode = 1;
            test = 7;

            printf("is_all_slave_op\n");
        }

        //    	actv = 5000;
        // printf("actv = %d \n",actv);
    }

    if (1 == mode)
    {

        temp++;
        if (temp < 10000000)
        {
            for (int i = 0; i < D_MASTER0_SLAVE_AMOUNT; i++)
            {
                master0_actpos[i] = master0_actpos[i] + 5000;
            }

        }

        if (temp >= 10000000)
        {
            temp1++;
            for (int i = 0; i < D_MASTER0_SLAVE_AMOUNT; i++)
            {
                master0_actpos[i] = master0_actpos[i] - 5000;
            }

        }

        if (temp1 >= 10000000)
        {
            temp1 = 0;
            temp = 0;
        }

        // if(actpos > 10000000){
        //     actpos = actpos - 300;
        // }else if(actpos <= 10000000 && actpos >= -10000000){
        //     actpos = actpos + 300;
        // }else if(actpos < -10000000){
        //     actpos = actpos + 300;
        // }
    }

    // if (0x06 == ContollerWord)
    // {
    //     printf("=================\n");
    //     printf("ContollerWord===%d\n", ContollerWord);
    //     printf("=================\n");
    // }

    // master0
    S_EthercatMaster *pmaster0 = get_master(0);

    for (int i = 0; i < D_MASTER0_SLAVE_AMOUNT; i++)
    {
        // write process data
        EC_WRITE_U16(pmaster0->pdomain_pds[0] + pmaster0->slave_offsets[i].ControlWord, ContollerWord);
        EC_WRITE_U8(pmaster0->pdomain_pds[0] + pmaster0->slave_offsets[i].OperationMode, (uint8_t)CspMode);
        EC_WRITE_S32(pmaster0->pdomain_pds[0] + pmaster0->slave_offsets[i].TargetPosition, (int32_t)master0_actpos[i]);
        //            EC_WRITE_S32(domain0_pd + s_Master0_EcatOffset[i].TargetSpeed, setSpeed);

        pmaster0->slave_values[i].StatusWord = EC_READ_U16(pmaster0->pdomain_pds[0] + pmaster0->slave_offsets[i].StatusWord);
        pmaster0->slave_values[i].ActPos = EC_READ_S32(pmaster0->pdomain_pds[0] + pmaster0->slave_offsets[i].ActPos);

        master0_tempos[i] = pmaster0->slave_values[i].ActPos;

        pmaster0->slave_values[i].Temperature = EC_READ_U16(pmaster0->pdomain_pds[0] + pmaster0->slave_offsets[i].Temperature);
        pmaster0->slave_values[i].TorqueSensor = EC_READ_S32(pmaster0->pdomain_pds[0] + pmaster0->slave_offsets[i].TorqueSensor);
        pmaster0->slave_values[i].ErrorStatus = EC_READ_U16(pmaster0->pdomain_pds[0] + pmaster0->slave_offsets[i].ErrorStatus);
        pmaster0->slave_values[i].ActSpeed = EC_READ_S32(pmaster0->pdomain_pds[0] + pmaster0->slave_offsets[i].ActSpeed);
        pmaster0->slave_values[i].Current = EC_READ_S16(pmaster0->pdomain_pds[0] + pmaster0->slave_offsets[i].Current);
        pmaster0->slave_values[i].Voltage = EC_READ_U32(pmaster0->pdomain_pds[0] + pmaster0->slave_offsets[i].Voltage);
    }


///////////////////////////////////////////////////////////////////dc process
#ifdef dc_user
    clock_gettime(CLOCK_MONOTONIC, &dctime);
    apptime = dctime.tv_sec * 1000000000 + dctime.tv_nsec;
    ecrt_master_application_time(pmaster0->pmaster, apptime);


    if (sync_ref_counter)
    {
        sync_ref_counter--;
    }
    else
    {
        sync_ref_counter = 1;
        ecrt_master_sync_reference_clock(pmaster0->pmaster);

    }
    ecrt_master_sync_slave_clocks(pmaster0->pmaster);

#endif

    //////////////////////////////////////////////////////////////////
    // if (is_slave_op())
    {
        // send process data
        ecrt_domain_queue(pmaster0->pdomains[0]);
    }

    ecrt_master_send(pmaster0->pmaster);


    //     if(561==statcode){
    //     	state_561 =1;
    //     }
    //     if(563==statcode){
    //     	state_563 =1;
    //     }
    //    printf("statuscode = %d 561:%d 563 %d\n", statcode, state_561, state_563);
}

void custom_task()
{
    if (sPrintCount >= 20000)
    {
        S_EthercatMaster *pmaster0 = get_master(0);
        S_EthercatMaster *pmaster1 = get_master(1);
        if (is_all_slave_op())
        {
            // printf ns time
            printf("period     %d ... %d us\n",
                   (int)(period_min_ns / 1000.0), (int)(period_max_ns / 1000.0));
            printf("exec       %d ... %d us\n",
                   (int)(exec_min_ns / 1000.0), (int)(exec_max_ns / 1000.0));

            for (int i = 0; i < D_MASTER0_SLAVE_AMOUNT; i++)
            {
                printf("master0 slave0 =i==%d==statuscode = %d\n", i, pmaster0->slave_values[i].StatusWord);
            }
            printf("\n");


            printf("timeOutCount===%d,continusTimeOut===%d,threadTimeOut==%d\n", timeOutCount, contTimeCount, threadConTimeOut);

            for (int i = 0; i < D_MASTER0_SLAVE_AMOUNT; i++)
            {
                printf("master0******actpos[i]====%d", master0_actpos[i]);
            }
            printf("\n");

        }

        check_master_slave_state();

        sPrintCount = 0;
    }
}

void *custom_thread(void *arg)
{
    printf("custom-thread===\n");
    struct timespec *setPeriod = (struct timespec *)arg;

    uint32_t sleep_us = setPeriod->tv_sec * 1000000 + setPeriod->tv_nsec / 1000;

    while (1)
    {
        custom_task();
        usleep(sleep_us);
    }
}

void *rt_thread(void *arg)
{

    cpu_set_t cpuSet;
    CPU_ZERO(&cpuSet);
    CPU_SET(D_TARGET_CPU, &cpuSet);

    if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuSet) != 0)
    {
        printf("failed to pthread_setaffinity_np\n");
        return NULL;
    }

    prctl(PR_SET_NAME, "rt thread");
    struct timespec *setPeriod = (struct timespec *)arg;

    printf("rt-thread===\n");

    while (1)
    {
        sPrintCount++;

        // if (is_slave_op())
        {

            clock_gettime(CLOCK_MONOTONIC, &startTime);
            if (0 == sPeriodCount)
            {
                lastStartTime = startTime;
            }
            else
            {
                period_ns = DIFF_NS(lastStartTime, startTime);
                lastStartTime = startTime;
            }

            clock_gettime(CLOCK_MONOTONIC, &taskStartTime);
            // do something...
            cyclic_task();

            clock_gettime(CLOCK_MONOTONIC, &taskEndTime);
            endTime = taskEndTime;

            exec_ns = DIFF_NS(taskStartTime, taskEndTime);

            long int processTime = (endTime.tv_sec - startTime.tv_sec) * 1000000000 +
                                   (endTime.tv_nsec - startTime.tv_nsec);

            if (processTime >= setPeriod->tv_nsec)
            {
                printf("processTime====%ld,tv_nsec====%ld\n", processTime, setPeriod->tv_nsec);
                contTimeCount++;
                threadQuitFlag = 1;
                timeOutCount++;
                if (contTimeCount >= 2)
                {
                    threadConTimeOut++;
                }
            }
            else
            {
                contTimeCount = 0;
                nextTime = startTime;
                //            nextTime.tv_nsec += (setPeriod->tv_nsec - processTime);
                nextTime.tv_nsec += setPeriod->tv_nsec;
                if (nextTime.tv_nsec >= 1000000000)
                {
                    nextTime.tv_sec++;
                    nextTime.tv_nsec -= 1000000000;
                }

                clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &nextTime, NULL);
            }

            if (1 == sPeriodCount)
            {
                if (period_ns < period_min_ns)
                {
                    period_min_ns = period_ns;
                }

                if (period_ns > period_max_ns)
                {
                    period_max_ns = period_ns;
                }
            }

            if (exec_ns < exec_min_ns)
            {
                exec_min_ns = exec_ns;
            }

            if (exec_ns > exec_max_ns)
            {
                exec_max_ns = exec_ns;
            }

            // if (threadQuitFlag)
            // {
            //     printf("quit\n");
            //     break;
            // }

            if (0 == sPeriodCount)
            {
                sPeriodCount = 1;
            }
        }
    }
}

int main(int argc, char **argv)
{
    // init slave values
    init_slave_values();

    // 初始化所有主站
    for (int i = 0; i < D_MASTER_AMOUNT; i++)
    {
        if (init_master(i) != 0)
        {
            fprintf(stderr, "Failed to initialize master %d\n", i);
            return -1;
        }
    }

    // master0 config
    for (int i = 0; i < D_MASTER0_SLAVE_AMOUNT; i++)
    {
        configure_slave(0, i, TC);
    }


    // master0 pdo map
    for (int i = 0; i < D_MASTER0_SLAVE_AMOUNT; i++)
    {
        auto_configure_slave_pdo_mapping(0, i, TC);
    }

    // 注册PDO条目到域
    for (int master_idx = 0; master_idx < D_MASTER_AMOUNT; master_idx++)
    {
        for (int domain_idx = 0; domain_idx < D_MAX_DOMAINS_PER_MASTER; domain_idx++)
        {
            if (register_domain_pdo_entries(master_idx, domain_idx) != 0)
            {
                fprintf(stderr, "Failed to register PDO entries for master %d domain %d\n",
                        master_idx, domain_idx);
                return -1;
            }
        }
    }

    // master0 dc
    for (int i = 0; i < D_MASTER0_SLAVE_AMOUNT; i++)
    {
        configure_dc_time(0, i);
    }


    // 激活所有主站
    for (int i = 0; i < D_MASTER_AMOUNT; i++)
    {
        if (activate_master(i) != 0)
        {
            fprintf(stderr, "Failed to activate master %d\n", i);
            return -1;
        }
    }

    // S_EthercatMaster *pmaster = &masters[0];

    // for (int i = 0; i < D_MASTER0_SLAVE_AMOUNT; i++)
    // {
    //     printf("==========================\n\n\n\n\n");

    //     printf("PDO offset msg\n");
    //     printf("PDO offset Temperature:%u\n", pmaster->slave_offsets[i].Temperature);
    //     printf("PDO offset TorqueSensor:%u\n", pmaster->slave_offsets[i].TorqueSensor);
    //     printf("PDO offset ErrorStatus:%u\n", pmaster->slave_offsets[i].ErrorStatus);
    //     printf("PDO offset StatusWord:%u\n", pmaster->slave_offsets[i].StatusWord);
    //     printf("PDO offset ActPos:%u\n", pmaster->slave_offsets[i].ActPos);
    //     printf("PDO offset ActSpeed:%u\n", pmaster->slave_offsets[i].ActSpeed);
    //     printf("PDO offset Current:%u\n", pmaster->slave_offsets[i].Current);

    //     printf("PDO offset Voltage:%u\n", pmaster->slave_offsets[i].Voltage);

    //     printf("PDO offset TargetCurrent:%u\n", pmaster->slave_offsets[i].TargetCurrent);
    //     printf("PDO offset ControlWord:%u\n", pmaster->slave_offsets[i].ControlWord);
    //     printf("PDO offset TargetPosition:%u\n", pmaster->slave_offsets[i].TargetPosition);
    //     printf("PDO offset TargetSpeed:%u\n", pmaster->slave_offsets[i].TargetSpeed);
    //     printf("PDO offset OperotionMode:%u\n", pmaster->slave_offsets[i].OperationMode);

    //     printf("==========================\n\n\n\n\n");
    // }

    //============================thread=============================
    /* Lock memory */

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    {
        printf("Warning: Failed to lock memory\n");
        exit(-2);
    }

    // cpu_set_t cpuSet;
    // CPU_ZERO(&cpuSet);
    // CPU_SET(D_TARGET_CPU,&cpuSet);

    // pid_t pid = getpid();

    // if(sched_setaffinity(pid,sizeof(cpu_set_t),&cpuSet)){
    //     perror("sched_setaffinity failed");
    //     return -1;
    // }

    // cpu_set_t act_set;
    // if(sched_getaffinity(pid,sizeof(cpu_set_t),&act_set) == -1){
    //     perror("sched_getaffinity failed");
    //     return -1;
    // }

    // printf("max cpu size===%d\n",CPU_SETSIZE);
    // printf("Actual affinity mask is: ");
    // for(int i = 0; i < CPU_SETSIZE;i++){
    //     if(CPU_ISSET(i,&act_set)){
    //         printf("CPU%d ",i);
    //     }
    // }

    // printf("\n");

    /* Set priority */
    pthread_t rtThread;
    pthread_t customThread;
    pthread_attr_t attr;
    pthread_attr_t customAttr;

    struct sched_param param = {};
    struct sched_param customPara = {};

    int ret = pthread_attr_init(&attr);
    if (0 != ret)
    {
        printf("pthread_attr_init error ret = %d\n", ret);
        return -1;
    }

    ret = pthread_attr_init(&customAttr);
    if (0 != ret)
    {
        printf("pthread_attr_init customAttr error ret = %d\n", ret);
        return -1;
    }

    ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    if (0 != ret)
    {
        printf("pthread pthread_attr_setshedpolicy failed ret = %d\n", ret);
        return -1;
    }

    ret = pthread_attr_setschedpolicy(&customAttr, SCHED_OTHER);
    if (0 != ret)
    {
        printf("pthread pthread_attr_setshedpolicy customAttr failed ret = %d\n", ret);
        return -1;
    }

    param.sched_priority = D_PRIORITY;
    ret = pthread_attr_setschedparam(&attr, &param);
    if (0 != ret)
    {
        printf("pthread pthread_attr_setschedparam failed ret = %d\n", ret);
        return -1;
    }

    customPara.sched_priority = 0;
    ret = pthread_attr_setschedparam(&customAttr, &customPara);
    if (0 != ret)
    {
        printf("pthread pthread_attr_setschedparam customPara failed ret = %d\n", ret);
        return -1;
    }

    ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (0 != ret)
    {
        printf("thread pthread_attr_setinheritsched ret = %d\n", ret);
        return -1;
    }

    ret = pthread_attr_setinheritsched(&customAttr, PTHREAD_EXPLICIT_SCHED);
    if (0 != ret)
    {
        printf("thread pthread_attr_setinheritsched customAttr ret = %d\n", ret);
        return -1;
    }

    // printf("Using priority %i.\n", param.sched_priority);
    // if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    //     perror("sched_setscheduler failed");
    //     return -1;
    // }

    period.tv_sec = 0;
    period.tv_nsec = D_SM_TIME; // 125us

    ret = pthread_create(&rtThread, &attr, &rt_thread, (void *)&period);
    if (0 != ret)
    {
        printf("pthread_create error ret = %d\n", ret);
        return -1;
    }

    cusTomPeriod.tv_sec = 0;
    cusTomPeriod.tv_nsec = 5000 * 1000; // 5000us

    ret = pthread_create(&customThread, &customAttr, &custom_thread, (void *)&cusTomPeriod);
    if (0 != ret)
    {
        printf("pthread_create custom_thread error ret = %d\n", ret);
        return -1;
    }

    pthread_join(rtThread, NULL);

    munlockall();

    cleanup_pdo_mapping();

    printf("thread over\n");

    return 0;
}
