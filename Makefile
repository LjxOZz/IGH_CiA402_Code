# 编译器
CC := gcc

# 目标程序
TARGET := rt_posix_demo

# 源文件
SRCS := main.c

# 对象文件
OBJS := $(SRCS:.c=.o)


# EtherCAT 配置
ETHERCAT_INCLUDE := -I/include
ETHERCAT_LIB := -lethercat #-L/usr/local/ethercat/lib

# # EtherCAT 配置
# ETHERCAT_INCLUDE := -I/usr/local/etherlab/include
# ETHERCAT_LIB := -L/usr/local/etherlab/lib -lethercat

# 编译选项
CFLAGS := -O2 -g -Wall $(XENO_CFLAGS) $(ETHERCAT_INCLUDE) -I.

# 链接选项 - 注意库的顺序很重要！
LDFLAGS := $(XENO_LIBS) $(ETHERCAT_LIB) -lpthread -lrt

# 默认目标
all: $(TARGET)

# 链接目标程序
$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)

# 编译源文件
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

	# 具体的依赖关系
main.o: main.c

# 清理
clean:
	rm -f $(TARGET) $(OBJS)

# 安装（设置能力）
install: $(TARGET)
	sudo setcap cap_sys_nice+ep $(TARGET)

# 运行测试
run: $(TARGET)
	sudo LD_LIBRARY_PATH=/usr/local/ethercat/lib:$(LD_LIBRARY_PATH) ./$(TARGET)

# # 运行测试
# run: $(TARGET)
# 	sudo LD_LIBRARY_PATH=/usr/local/etherlab/lib:$(LD_LIBRARY_PATH) ./$(TARGET)




# 显示构建信息
info:
	@echo "CFLAGS: $(CFLAGS)"
	@echo "LDFLAGS: $(LDFLAGS)"
	@echo "Xenomai version:"
	@$(XENO_CONFIG) --version
	@echo "EtherCAT library:"
	@ls -la /usr/local/ethercat/lib/libethercat.* 2>/dev/null || echo "EtherCAT library not found"

# # 显示构建信息
# info:
# 	@echo "CFLAGS: $(CFLAGS)"
# 	@echo "LDFLAGS: $(LDFLAGS)"
# 	@echo "Xenomai version:"
# 	@$(XENO_CONFIG) --version
# 	@echo "EtherCAT library:"
# 	@ls -la /usr/local/etherlab/lib/libethercat.* 2>/dev/null || echo "EtherCAT library not found"

.PHONY: all clean install run info
