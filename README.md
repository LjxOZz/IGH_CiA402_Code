# 简介

这是一个demo

## 目前

1. 使用EnterCAT控制电机 Pp模式运动
2. 目前只控制了一个电机, 但留了多主站,多从站的接口(没有写对应的处理API)
3. 添加Cpp封装框架
4. 添加nmxrt库

## 计划修改

- [x] 1\. 重新封装IGH库相关API(单电机的PDO, SDO) ---1.16
- [x] 2\. 编写 CMake 和 Cpp 封装IGH库框架 ---1.19
- [ ] 3\. 相关API添加多电机操作(**`masters`**)
  - [x] 3.1 改进检查状态API ---1.23
  - [x] 3.2 判断使用什么CIA402的控制模式 ---1.23(用CSP模式)
  - [x] 3.3 添加获取电机状态接口
  - [ ] 3.4 添加CSP模式控制接口
- [ ] 4\. 重写同步时钟(**DC**)相关函数
- [ ] 5\. 接入`nmxrt`库
  - [ ] 5.1 重写`CMakeList.txt`框架
  - [ ] 5.2 实现`publisher`电机状态 (速度,位置,力矩,等等)
- [ ] 6\. c库实现多从机
 
# 框架

## ./: c语言实现

1. 电机控制接口`igh_coe_motor.h`: 
   1. `ecrt_init`
   2. `read_pdo`, `write_pdo`
   3. `read_sdo`, `write_sdo`
2. 电机模式实时运行时的接口函数`igh_rt_operation.h`: 
   1. `motor_csp_run_cycle`
   2. `motor_pp_run_cycle`
3. 等等

## cpp代码

Cpp封装:

1. C线程
2. 实现 **`TcTorsoDevice`** 类
3. ~nmxrt~

## tests代码
- [x] 1\. c库接口测试

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





