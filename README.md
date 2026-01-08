ethercat默认路径为/usr/local/ethercat/  如果默认安装路径不是此路径则需要修改Makefile(修改指向用户自己安装路径 替换所有的/usr/local/ethercat/)

# EtherCAT 配置
ETHERCAT_INCLUDE := -I/usr/local/ethercat/include
ETHERCAT_LIB := -L/usr/local/ethercat/lib -lethercat

# 运行测试
run: $(TARGET)
	sudo LD_LIBRARY_PATH=/usr/local/ethercat/lib:$(LD_LIBRARY_PATH) ./$(TARGET)

# 显示构建信息
info:
	@echo "CFLAGS: $(CFLAGS)"
	@echo "LDFLAGS: $(LDFLAGS)"
	@echo "Xenomai version:"
	@$(XENO_CONFIG) --version
	@echo "EtherCAT library:"
	@ls -la /usr/local/ethercat/lib/libethercat.* 2>/dev/null || echo "EtherCAT library not found"




1、线程相关配置为宏定义

// bind cpu id  实时线程绑定cpu1
#define D_TARGET_CPU 1

// thread priority 实时线程优先级为95
#define D_PRIORITY 95

// sm cycle time    实时线程运行周期125us
#define D_SM_TIME  125 * 1000

2、从站个数配置需要修改2个地方，必须同步修改

1）从站个数宏定义
//modified here
#define D_MASTER0_SLAVE_COUNT 7

2）从站配置
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

假如要修改为5个从站，则进行以下修改：

#define D_MASTER0_SLAVE_COUNT 5

static S_Master_Slave_Conf master0_slave_conf[] = {
    // Master 0 - 伺服驱动器
    {0, 0, TC, "Master0-Slave0: TC Servo Drive 1"},
    {0, 1, TC, "Master0-Slave1: TC Servo Drive 2"},
    {0, 2, TC, "Master0-Slave1: TC Servo Drive 3"},
    {0, 3, TC, "Master0-Slave1: TC Servo Drive 4"},
    {0, 4, TC, "Master0-Slave1: TC Servo Drive 5"},
//    {0, 5, TC, "Master0-Slave1: TC Servo Drive 6"},
//    {0, 6, TC, "Master0-Slave1: TC Servo Drive 7"},
};

3、修改main.c以后需要make clean，然后make，在本地生成可执行文件rt_posix_demo

4、运行

sudo ./rt_posix_demo






