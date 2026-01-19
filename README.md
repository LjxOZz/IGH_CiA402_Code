# 简介

这是一个Demo:
- 使用EnterCAT控制电机 Pp模式运动
- 目前只控制了一个电机, 但留了多主站,多从站的接口(没有写对应的处理API)

# 目前的API
**igh_coe_motor.h** 文件的API简介:

## 1. 初始化相关
- int ecrt_init(void): 实现初始化(主站0,从站0)设备
  - PDO配置
  - SDO配置
  - 同步时钟配置

## 2. PDO
**XX**: u8, u16, u32, s8, s16, s32
- read_pdo_uXX: pdo读
- write_pdo_uXX: pdo写

## 3. SDO
**XX**: u8, u16, u32, s8, s16, s32
- read_sdo_uXX: sdo读
- write_sdo_uXX: sdo写


## 运行
```
sudo modprobe ec_master
sudo /etc/init.d/ethercat start
sudo modprobe ec_generic

mkdir build && cd build
cmake ..
sudo ./rt_posix_demo
```





