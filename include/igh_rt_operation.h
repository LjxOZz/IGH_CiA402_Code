#ifndef MY_IGH_RT_H
#define MY_IGH_RT_H

#include <stdint.h>

typedef enum {
    MOTOR_STATE_INIT = 0,
    MOTOR_STATE_READY,
    MOTOR_STATE_ERROR
} motor_state_t;

/* Cpp 封装 */
#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint64_t period_min_ns;
extern volatile uint64_t period_max_ns;
extern volatile uint64_t exec_min_ns;
extern volatile uint64_t exec_max_ns;
extern volatile uint64_t timeOutCount;
extern volatile uint32_t contTimeCount;
extern volatile uint32_t threadConTimeOut;

extern int sPrintCount;

extern pthread_t rtThread;
extern bool get_status_flags;
extern int32_t motor0_Speed;
extern int32_t motor0_Position;
extern int16_t motor0_Torque;

int create_motor_thread(void);
// int create_custom_thread(void);


#ifdef __cplusplus
}
#endif

#endif

