#include "main.h"

static ec_master_state_t smaster_state = {};
static ec_domain_state_t sdomain_state = {};
static ec_slave_config_state_t sslave_state = {};

/* TC200E CoE drive parameters */
#define TcDriveSlavePos     0, 0        /* Master 0, Slave 0 */
#define TcDriveVendorID     0x00075500
#define TcDriveProductCode  0x00000001

/**
 * @brief PDO输入条目
 */
static const ec_pdo_entry_reg_t domain_input_regs[] = {
    {TcDriveSlavePos, TcDriveVendorID, TcDriveProductCode, 0x6071, 0x00, &masters[0].slave_offsets[0].TargetCurrent, 0},
    {TcDriveSlavePos, TcDriveVendorID, TcDriveProductCode, 0x6040, 0x00, &masters[0].slave_offsets[0].ControlWord, 0},
    {TcDriveSlavePos, TcDriveVendorID, TcDriveProductCode, 0x607A, 0x00, &masters[0].slave_offsets[0].TargetPosition, 0},
    {TcDriveSlavePos, TcDriveVendorID, TcDriveProductCode, 0x60FF, 0x00, &masters[0].slave_offsets[0].TargetSpeed, 0},
    {TcDriveSlavePos, TcDriveVendorID, TcDriveProductCode, 0x6060, 0x00, &masters[0].slave_offsets[0].OperationMode, 0},
    // {TcDriveSlavePos, TcDriveVendorID, TcDriveProductCode, 0x6081, 0x00, &masters[0].slave_offsets[0].ProfileSpeed, 0},
    {TcDriveSlavePos, TcDriveVendorID, TcDriveProductCode, 0x603F, 0x00, &masters[0].slave_offsets[0].ErrorStatus, 0},
    {TcDriveSlavePos, TcDriveVendorID, TcDriveProductCode, 0x6041, 0x00, &masters[0].slave_offsets[0].StatusWord, 0},
    {TcDriveSlavePos, TcDriveVendorID, TcDriveProductCode, 0x6064, 0x00, &masters[0].slave_offsets[0].ActualPos, 0},
    {TcDriveSlavePos, TcDriveVendorID, TcDriveProductCode, 0x606C, 0x00, &masters[0].slave_offsets[0].ActualSpe, 0},
    {TcDriveSlavePos, TcDriveVendorID, TcDriveProductCode, 0x6077, 0x00, &masters[0].slave_offsets[0].ActualTor, 0},
    {}
};

/**
 * @brief SDO请求结构体
 */
ec_sdo_request_t *psdo_profile_velocity;    /*设定目标速度(单位：puu/s)*/
ec_sdo_request_t *psdo_profile_acce;        /*规划加速度斜率(puu/s²)*/
ec_sdo_request_t *psdo_profile_dece;        /*规划减速度斜率(puu/s²)*/



/**
 * @brief   初始化从站值
 * @param   void
 * @return  void
 */
void init_slave_values(void) {
    for (int master_idx = 0; master_idx < D_MASTER_AMOUNT; master_idx++) {
        printf("Obtain the assignment of the main station structure");
    }
}
/**
 * @brief   获取主站状态, 并且保存到 全局的smaster_state变量中
 * @param pmaster 主站
 * @return  void
 */
void check_master_state(ec_master_t *pmaster) {
    ec_master_state_t ms;
    ecrt_master_state(pmaster, &ms);

    if (ms.slaves_responding != smaster_state.slaves_responding) {
        printf("%u slave(s).\n", ms.slaves_responding);
    }
    if (ms.al_states != smaster_state.al_states) {
        printf("AL states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != smaster_state.link_up) {
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }
    smaster_state = ms;
}
/**
 * @brief   获取通信域状态, 并且保存到 全局的sdomain_state变量中
 * @param pdomain 通信域
 * @return  void
 */
void check_domain_state(ec_domain_t *pdomain) {
    ec_domain_state_t ds;
    ecrt_domain_state(pdomain, &ds);

    if (ds.working_counter != sdomain_state.working_counter) {
        // printf("check_domain_state: WC %u.\n", ds.working_counter);
    }
    if (ds.wc_state != sdomain_state.wc_state) {
        // printf("check_domain_state: State %u.\n", ds.wc_state);
    }
    sdomain_state = ds;
}
/**
 * @brief   获取从站状态, 并且保存到 全局的sslave_state变量中
 * @param pslave_config 从站
 * @return  void
 */
void check_master_slave_state(void) {
    ec_slave_config_state_t ss;
    
    ecrt_slave_config_state(masters[0].pslave_configs[0], &ss);
    printf("master0==%d Slave state: AL state=0x%02X, online=%d, operational=%d\n",
            0, ss.al_state, ss.online, ss.operational);
    sslave_state = ss;
}
/**
 * @brief   初始化配置一个从站
 * @param void 
 * @return  成功:0 失败:-1
 */
int ecrt_init(void) {
    

    // 1.请求 EtherCAT 主站
    masters[0].pmaster = ecrt_request_master(0);
    if (!masters[0].pmaster) {
        fprintf(stderr, "Failed to request master 0\n");
        return -1;
    }
    // 2.创建通信域
    masters[0].pdomains[0] = ecrt_master_create_domain(masters[0].pmaster);
    if (!masters[0].pdomains[0]) {
        fprintf(stderr, "Failed to create domain 0 for master %d\n", 0);
        return -1;
    }
    // 3.获取从站配置
    masters[0].pslave_configs[0] = ecrt_master_slave_config(masters[0].pmaster, TcDriveSlavePos, TcDriveVendorID, TcDriveProductCode);
    if (!masters[0].pslave_configs[0]) {
        fprintf(stderr, "Failed to get slave config for slave 0\n");
        return -1;
    }
    // 4.配置从站的PDO
    if (ecrt_slave_config_pdos(masters[0].pslave_configs[0], slave_configs[0].sync_count, slave_configs[0].syncs)) {
        fprintf(stderr, "Failed to configure PDOs for slave 0\n");
        return -1;
    }

    // 5.配置从站的SDO
    psdo_profile_velocity = ecrt_slave_config_create_sdo_request(masters[0].pslave_configs[0], 0x6081, 0, 32);
    if(psdo_profile_velocity == NULL) {
        fprintf(stderr, "Failed to configure profile_velocity SDO for slave 0\n");
        return -1;
    }
    psdo_profile_acce = ecrt_slave_config_create_sdo_request(masters[0].pslave_configs[0], 0x6083, 0, 32);
    if(psdo_profile_acce == NULL) {
        fprintf(stderr, "Failed to configure profile_acce SDO for slave 0\n");
        return -1;
    }
    psdo_profile_dece = ecrt_slave_config_create_sdo_request(masters[0].pslave_configs[0], 0x6084, 0, 32);
    if(psdo_profile_dece == NULL) {
        fprintf(stderr, "Failed to configure profile_dece SDO for slave 0\n");
        return -1;
    }

    // 6.
    if (ecrt_domain_reg_pdo_entry_list(masters[0].pdomains[0], domain_input_regs)) {
        fprintf(stderr, "PDO entry registration failed for master 0 domain 0\n");
        return -1;
    }
    // 7.配置分布式时钟
    if (ecrt_slave_config_dc(masters[0].pslave_configs[0], 0x300, D_SM_TIME, D_SHITE_TIME, 0, 0)) {
        fprintf(stderr, "Failed to configure DC\n");
        return -1;
    }
    // 8. 激活主站
    if (ecrt_master_activate(masters[0].pmaster)) {
        fprintf(stderr, "Failed to activate master 0\n");
        return -1;
    }

    // 9.获取域数据指针
    if (!(masters[0].pdomain_pds[0] = ecrt_domain_data(masters[0].pdomains[0]))) {
        fprintf(stderr, "Failed to get domain data for master 0 domain 0\n");
        return -1;
    }

    printf("\n<===============================================>\n");
    printf("Master x Finish Activate!\n");
    printf("\n<===============================================>\n");
    
    return 0;
}

