#ifndef MY_IGH_COE_H
#define MY_IGH_COE_H

#include <stdio.h>
#include <stdint.h>
#include "ecrt.h" 

/**
 * @brief 主机上的EnterCAT接口数
 */
#define D_MASTER_AMOUNT 1
/**
 * @brief 最大通信域数量
 */
#define D_MAX_DOMAINS_PER_MASTER    1
/**
 * @brief 主站的最大从站数量
 */
#define D_MAX_SLAVES_PER_MASTER     32
/**
 * @brief 
 */
#define D_SM_TIME 1000 * 1000
/**
 * @brief 
 */
#define D_SHITE_TIME (D_SM_TIME / 2)


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

// offsets for PDO entries
typedef struct S_Ecat_Offset
{
    unsigned int TargetCurrent; /*0x6071 int16 */
    unsigned int ControlWord;   /*0x6040 uint16*/
    unsigned int TargetPosition;/*0x607A int32 */
    unsigned int TargetSpeed;   /*0x60FF int32 */
    unsigned int OperationMode; /*0x6060 uint8 */
    // unsigned int ProfileSpeed;  /*0x6081 uint32*/
    // read
    unsigned int ErrorStatus;   /*0x603F uint16*/
    unsigned int StatusWord;    /*0x6041 uint16*/
    unsigned int ActualPos;     /*0x6064 int32 */
    unsigned int ActualSpe;     /*0x606C int32 */
    unsigned int ActualTor;     /*0x6077 int16 */
} S_Ecat_Offset;

// offsets for PDO entries
typedef struct S_Ecat_Value
{
    // write
    int16_t TargetCurrent;  /* 0x6071 int16*/
    uint16_t ControlWord;   /*0x6040 uint16*/
    int32_t TargetPosition; /*0x607A int32*/
    int32_t TargetSpeed;    /*0x60FF int32*/
    uint8_t OperotionMode;  /*0x6060 uint8*/
    uint32_t ProfileSpeed;  /*0x6081 uint32*/
    // read
    uint16_t ErrorStatus; /*0x603F uint16*/
    uint16_t StatusWord;  /* 0x6041 uint16*/
    int32_t ActPos;       /*0x6064 int32*/
    int32_t ActSpeed;     /* 0x606C int32*/
    int16_t Current;      /*0x6077 int16*/

} S_Ecat_Value;


/**
 * @brief 表示Ethercat主站的结构体
 */
typedef struct S_EthercatMaster
{
    ec_master_t *pmaster; /**< 主站 */
    ec_domain_t *pdomains[D_MAX_DOMAINS_PER_MASTER]; /**< 通信域 */
    
    uint8_t *pdomain_pds[D_MAX_DOMAINS_PER_MASTER]; /**<  */

    int slave_count; /**< 从站数量 */
    ec_slave_config_t *pslave_configs[D_MAX_SLAVES_PER_MASTER]; /**< 从站配置 */
    S_Ecat_Offset slave_offsets[D_MAX_SLAVES_PER_MASTER];
    S_Ecat_Value slave_values[D_MAX_SLAVES_PER_MASTER];

    ec_master_state_t master_state;
    ec_domain_state_t domain_states[D_MAX_DOMAINS_PER_MASTER];

} S_EthercatMaster;

/**
 * @brief 表示Ethercat从站配置
 */
typedef struct S_SlaveConfig
{
    uint32_t vendor_id; /**< 供应商id */
    uint32_t product_code; /**< 产品代码 */
    const char *name; /**< 名字 */
    int sync_count; /**< 同步计数 */
    int pdoEntryCount;
    ec_pdo_info_t *pdos;
    ec_sync_info_t *syncs;
    ec_pdo_entry_info_t *pEntry;
} S_SlaveConfig;

/**
 * @brief PDO注册条目结构
 */
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


/*  */

int ecrt_init(void);

void check_master_state(ec_master_t *pmaster);
void check_domain_state(ec_domain_t *pdomain);
void check_master_slave_state(void);

extern ec_sdo_request_t *psdo_profile_velocity;
extern ec_sdo_request_t *psdo_profile_acce;
extern ec_sdo_request_t *psdo_profile_dece;


#endif
