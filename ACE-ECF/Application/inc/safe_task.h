#ifndef __TASK_SAFE_H  //如果未定义
#define __TASK_SAFE_H  //那么定义
#include <stdlib.h>
/*************************freertos*************************/
#include "FreeRTOS.h"
#include "freertos.h"
#include "queue.h"
#include "semphr.h"

typedef void (*Callback)(void);
struct safe_task{
    char      *name;
    uint32_t  Disconnection_count;
    uint32_t  Disconnection_threshold;//失联阈值
    uint8_t   Disconnection_falg;
    void (*UserCallBack)(void);
    struct safe_task *next_task;
};
typedef struct safe_task safe_task_t;

void Safe_Init(uint32_t RUN_cycle_ms);
void Safe_Task(void const *argument);
uint8_t Safe_Task_online_ptr_(safe_task_t* temp_task_ptr);
uint8_t Safe_Task_online_name_(char * name);
uint8_t Safe_task_delect_ptr_(safe_task_t* delect_task_ptr);
uint8_t Safe_Task_delect_name_(char * delect_task_name);
#endif