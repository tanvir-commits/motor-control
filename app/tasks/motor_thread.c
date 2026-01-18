#include "motor_thread.h"
#include "tmc4671.h"
#include "tx_api.h"

/* ThreadX thread stack and control block */
#define MOTOR_THREAD_STACK_SIZE  1024
static uint8_t motor_thread_stack[MOTOR_THREAD_STACK_SIZE];
static TX_THREAD motor_thread;

/* Thread entry function */
void motor_thread_entry(ULONG input)
{
    /* Initialize TMC4671 */
    tmc4671_init();
    
    /* Configure for encoder (ENX 8 MAG 256IMP) */
    tmc4671_config_encoder_abn();
    
    /* Enable motor */
    tmc4671_enable();
    
    /* Main thread loop */
    while (1) {
        /* Example: Move motor by 20mm */
        tmc4671_move_by_mm(20.0f);
        
        /* Wait until movement is complete */
        while (tmc4671_is_moving()) {
            tx_thread_sleep(10);  /* Sleep 10 ticks (adjust based on your tick rate) */
        }
        
        /* Wait 2 seconds before next movement */
        tx_thread_sleep(200);  /* 200 ticks = 2 seconds (if 1 tick = 10ms) */
        
        /* Move back by 20mm */
        tmc4671_move_by_mm(-20.0f);
        
        /* Wait until movement is complete */
        while (tmc4671_is_moving()) {
            tx_thread_sleep(10);
        }
        
        /* Wait 2 seconds before repeating */
        tx_thread_sleep(200);
    }
}

/* Thread creation function - call from main or app_init */
void motor_thread_create(void)
{
    tx_thread_create(
        &motor_thread,
        "Motor Thread",
        motor_thread_entry,
        0,
        motor_thread_stack,
        MOTOR_THREAD_STACK_SIZE,
        5,  /* Priority (adjust as needed) */
        5,  /* Preempt threshold */
        TX_NO_TIME_SLICE,
        TX_AUTO_START
    );
}

