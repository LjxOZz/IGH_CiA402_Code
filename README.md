# 简介

这是一个demo

## 目前实现效果

1. 使用EnterCAT控制电机 Pp模式运动
2. 目前只控制了一个电机, 但留了多主站,多从站的接口(没有写对应的处理API)
3. 添加Cpp封装框架
4. 添加nmxrt库

## 计划修改

- [x] 1\. 重新封装IGH库相关API(单电机的PDO, SDO)  ---1.16
- [x] 2\. 编写 CMake 和 Cpp 封装IGH库框架         ---1.19
- [ ] 3\. 相关API添加多电机操作(**`masters`**)
  - [ ] 3.1 改进检查状态API
  - [ ] 3.2 判断使用什么CIA402的控制模式
  - [ ] 3.3 添加Cpp层的PDO SDO发送接受方法
- [ ] 4\. 重写同步时钟(**DC**)相关函数
- [ ] 5\. 接入`nmxrt`库
  - [ ] 5.1 重写`CMakeList.txt`框架
  - [ ] 5.2 实现`publisher`电机状态 (速度,位置,力矩,等等)


# API

## Cpp封装层

实现 **TcTorsoDevice** 类

## 电机控制API
**igh_coe_motor.h**

### 1. 初始化
- `int ecrt_init(void)`: 实现初始化(主站0,从站0)设备
  - PDO配置
  - SDO配置
  - 同步时钟配置

### 2. PDO/SDO
**`XX`**: u8, u16, u32, s8, s16, s32
- `read_pdo_uXX`: pdo读
- `write_pdo_uXX`: pdo写
- `read_sdo_uXX`: sdo读
- `write_sdo_uXX`: sdo写

## 线程API
**igh_rt_operation.h**

### 1. 初始化
- `int rt_init(void)`


# 环境搭建

## 1.安装 rt-linux and  ethercat 
```
sudo dpkg -i tztek-jetson-service-ethercat-v2.0.deb

sudo modprobe ec_master
sudo /etc/init.d/ethercat start
sudo modprobe ec_generic
```

## 2.安装nmxrt库
略

## 3.编译代码
```
mkdir build && cd build
cmake ..
sudo ./tests/test_c/c_igh_test
sudo ./tests/test_c/cpp_igh_test
```





