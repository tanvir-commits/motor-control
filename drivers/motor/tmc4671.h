#pragma once
#include <stdint.h>
#include <stdbool.h>

/* TMC4671 Register Addresses */
#define TMC4671_REG_ENABLE                   0x00
#define TMC4671_REG_PWM_POLARITIES          0x17
#define TMC4671_REG_PWM_MAXCNT              0x18
#define TMC4671_REG_MOTOR_TYPE_N_POLE_PAIRS 0x1B
#define TMC4671_REG_UQ_UD_EXT               0x24
#define TMC4671_REG_PID_TORQUE_FLUX_P       0x30
#define TMC4671_REG_PID_TORQUE_FLUX_I       0x31
#define TMC4671_REG_PID_VELOCITY_P          0x32
#define TMC4671_REG_PID_VELOCITY_I          0x33
#define TMC4671_REG_PID_POSITION_P          0x34
#define TMC4671_REG_PID_POSITION_I          0x35
#define TMC4671_REG_VELOCITY_SELECTION      0x50
#define TMC4671_REG_POSITION_SELECTION      0x51
#define TMC4671_REG_PHI_E_SELECTION         0x52
#define TMC4671_REG_MODE_RAMP_MODE_MOTION   0x60
#define TMC4671_REG_TARGET_POSITION         0x61
#define TMC4671_REG_TARGET_VELOCITY         0x62
#define TMC4671_REG_TARGET_TORQUE           0x63
#define TMC4671_REG_ACTUAL_POSITION         0x6A
#define TMC4671_REG_ACTUAL_VELOCITY         0x6B
#define TMC4671_REG_ACTUAL_TORQUE           0x6C

/* Communication Interface Selection */
typedef enum {
    TMC4671_IF_SPI = 0,    /* SPI interface (default) */
    TMC4671_IF_UART = 1    /* UART/LPUART interface */
} tmc4671_interface_t;

/* SPI Read/Write Bit Masks */
#define TMC4671_WRITE_BIT                   0x80
#define TMC4671_READ_BIT                    0x00

/* UART Protocol Constants */
#define TMC4671_UART_SYNC_BYTE              0x05
#define TMC4671_UART_NODE_ADDRESS           0x01    /* Default node address (master address) */
#define TMC4671_UART_BAUD_RATE              115200   /* Default baud rate (can be up to 1 Mbps) */
#define TMC4671_UART_WRITE_BIT              0x80    /* OR with address for write operations */
#define TMC4671_UART_CRC_POLYNOMIAL         0x07    /* CRC-8 polynomial: x^8 + x^2 + x^1 + 1 */

/* Register Bitfield Structures */
typedef union {
    uint32_t all;
    struct {
        uint32_t motor_type        : 8;   /* 0-7: Motor type (1=DC) */
        uint32_t reserved1         : 8;   /* 8-15: Reserved */
        uint32_t n_pole_pairs      : 16;  /* 16-31: Pole pairs (1 for DC) */
    } bits;
} tmc4671_motor_type_n_pole_pairs_t;

typedef union {
    uint32_t all;
    struct {
        uint32_t source            : 8;    /* 0-7: Feedback source (0=open loop, 3=ABN encoder) */
        uint32_t reserved          : 24;   /* 8-31: Reserved */
    } bits;
} tmc4671_feedback_selection_t;

typedef union {
    uint32_t all;
    struct {
        uint32_t motion_mode       : 8;    /* 0-7: Motion mode (0=position, 1=velocity, 2=torque) */
        uint32_t reserved          : 24;   /* 8-31: Reserved */
    } bits;
} tmc4671_mode_ramp_mode_motion_t;

/* Register Cache Structure - Debugger-Friendly */
typedef struct {
    uint32_t enable;                                    /* 0x00: Enable */
    uint32_t reserved_01_16[16];                        /* Reserved */
    uint32_t pwm_polarities;                            /* 0x17: PWM polarities */
    uint16_t pwm_maxcnt;                                /* 0x18: PWM frequency */
    uint16_t reserved_18_1B[2];                        /* Reserved */
    tmc4671_motor_type_n_pole_pairs_t motor_type_n_pole_pairs; /* 0x1B: Motor type */
    uint32_t reserved_1C_23[8];                        /* Reserved */
    uint32_t uq_ud_ext;                                 /* 0x24: Open-loop voltage */
    uint32_t reserved_25_2F[11];                       /* Reserved */
    uint16_t pid_torque_p;                              /* 0x30: Torque P gain */
    uint16_t pid_torque_i;                              /* 0x31: Torque I gain */
    uint16_t pid_velocity_p;                             /* 0x32: Velocity P gain */
    uint16_t pid_velocity_i;                            /* 0x33: Velocity I gain */
    uint16_t pid_position_p;                            /* 0x34: Position P gain */
    uint16_t pid_position_i;                            /* 0x35: Position I gain */
    uint32_t reserved_36_4F[26];                       /* Reserved */
    tmc4671_feedback_selection_t velocity_selection;    /* 0x50: Velocity feedback */
    tmc4671_feedback_selection_t position_selection;   /* 0x51: Position feedback */
    tmc4671_feedback_selection_t phi_e_selection;      /* 0x52: Electrical angle feedback */
    uint32_t reserved_53_5F[13];                       /* Reserved */
    tmc4671_mode_ramp_mode_motion_t mode_ramp_mode_motion; /* 0x60: Motion mode */
    int32_t target_position;                            /* 0x61: Target position */
    int32_t target_velocity;                            /* 0x62: Target velocity */
    int32_t target_torque;                              /* 0x63: Target torque */
    uint32_t reserved_64_69[6];                        /* Reserved */
    int32_t actual_position;                            /* 0x6A: Actual position (read-only) */
    int32_t actual_velocity;                            /* 0x6B: Actual velocity (read-only) */
    int32_t actual_torque;                              /* 0x6C: Actual torque (read-only) */
} tmc4671_reg_cache_t;

/* Global register cache - add to debugger watch window: tmc4671_regs */
extern tmc4671_reg_cache_t tmc4671_regs;

/* Initialization */
void tmc4671_init(void);
void tmc4671_set_interface(tmc4671_interface_t interface);  /* Select SPI or UART */

/* Register Read/Write */
uint32_t tmc4671_read_reg(uint8_t address);
void tmc4671_write_reg(uint8_t address, uint32_t value);

/* Register Cache Synchronization */
void tmc4671_read_reg_cache(uint8_t address);  /* Read from HW to cache */
void tmc4671_write_reg_cache(uint8_t address);  /* Write from cache to HW */
void tmc4671_sync_all_regs(void);               /* Sync all registers */

/* Motor Control */
void tmc4671_enable(void);
void tmc4671_disable(void);
void tmc4671_set_target_position(int32_t position);
int32_t tmc4671_get_actual_position(void);
bool tmc4671_is_moving(void);

/* Encoder Configuration */
void tmc4671_config_encoder_abn(void);

/* Position Movement */
void tmc4671_move_by_mm(float mm);

