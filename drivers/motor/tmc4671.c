#include "tmc4671.h"
#include "pio.h"
#include "pio_uart.h"
#include <string.h>

/* External SPI chip select pin - configure in your HAL */
extern GPIO_TypeDef* TMC4671_CS_PORT;
extern uint16_t TMC4671_CS_PIN;

/* Global register cache */
tmc4671_reg_cache_t tmc4671_regs = {0};

/* Communication interface selection */
static tmc4671_interface_t tmc4671_interface = TMC4671_IF_SPI;  /* Default to SPI */

/* Configuration for Maxon DCX08M EB KL 2.4V + ENX 8 MAG 256IMP + GPX08 A 64:1 */
#define ENCODER_BASE_COUNTS_PER_REV  256   /* ENX 8 MAG base resolution */
#define ENCODER_QUAD_MULTIPLIER      4     /* 4× quadrature */
#define ENCODER_EFFECTIVE_COUNTS     (ENCODER_BASE_COUNTS_PER_REV * ENCODER_QUAD_MULTIPLIER)  /* 1024 ticks/motor rev */
#define GEAR_RATIO                  64     /* GPX08 A 64:1 gear ratio */
#define LEADSCREW_LEAD_MM           2.0   /* mm per output shaft revolution (adjust for your application) */

/* CRC-8 calculation for UART protocol */
/* Polynomial: x^8 + x^2 + x^1 + 1 (0x07) */
/* Calculated over all bytes in datagram except CRC byte itself */
static uint8_t tmc4671_uart_crc8(uint8_t* data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ TMC4671_UART_CRC_POLYNOMIAL;  /* 0x07 */
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/* SPI Transfer Function */
static uint32_t tmc4671_spi_transfer(uint8_t address, uint32_t value, bool is_write)
{
    uint8_t tx[5];
    uint8_t rx[5];
    
    /* Prepare SPI frame */
    tx[0] = address | (is_write ? TMC4671_WRITE_BIT : TMC4671_READ_BIT);
    tx[1] = (value >> 24) & 0xFF;
    tx[2] = (value >> 16) & 0xFF;
    tx[3] = (value >> 8) & 0xFF;
    tx[4] = value & 0xFF;
    
    /* Transfer via PAL/PIO layer */
    pio_spi_transfer_cs(TMC4671_CS_PIN, tx, rx, 5);
    
    /* Return received data (for reads) */
    return ((uint32_t)rx[1] << 24) |
           ((uint32_t)rx[2] << 16) |
           ((uint32_t)rx[3] << 8)  |
           ((uint32_t)rx[4]);
}

/* UART Transfer Function */
/* Protocol: 8N1 (8 data bits, no parity, 1 stop bit) */
/* Write: Sync(0x05) + Node(0x01) + Addr(OR 0x80) + Data(4) + CRC(1) = 8 bytes */
/* Read Request: Sync(0x05) + Node(0x01) + Addr + CRC(1) = 4 bytes */
/* Read Response: Sync(0x05) + Node(0x01) + Addr + Data(4) + CRC(1) = 8 bytes */
static uint32_t tmc4671_uart_transfer(uint8_t address, uint32_t value, bool is_write)
{
    uint8_t tx[8];
    uint8_t rx[8] = {0};
    
    if (is_write) {
        /* Write frame: 8 bytes */
        tx[0] = TMC4671_UART_SYNC_BYTE;                    /* Sync byte */
        tx[1] = TMC4671_UART_NODE_ADDRESS;                 /* Master address */
        tx[2] = address | TMC4671_UART_WRITE_BIT;          /* Register address OR 0x80 for write */
        tx[3] = (value >> 24) & 0xFF;                      /* Data MSB */
        tx[4] = (value >> 16) & 0xFF;
        tx[5] = (value >> 8) & 0xFF;
        tx[6] = value & 0xFF;                              /* Data LSB */
        tx[7] = tmc4671_uart_crc8(tx, 7);                  /* CRC of bytes 0-6 */
        
        /* Transfer via PAL/PIO layer */
        pio_uart_transfer(tx, 8, NULL, 0);
    } else {
        /* Read request: 4 bytes */
        tx[0] = TMC4671_UART_SYNC_BYTE;                    /* Sync byte */
        tx[1] = TMC4671_UART_NODE_ADDRESS;                 /* Master address */
        tx[2] = address;                                   /* Register address (no 0x80 for read) */
        tx[3] = tmc4671_uart_crc8(tx, 3);                  /* CRC of bytes 0-2 */
        
        /* Transfer request and receive response */
        pio_uart_transfer(tx, 4, rx, 8);
        
        /* Parse response: bytes 3-6 contain the 32-bit data */
        if (rx[0] == TMC4671_UART_SYNC_BYTE && rx[1] == TMC4671_UART_NODE_ADDRESS) {
            /* Verify CRC of response (bytes 0-6) */
            uint8_t calc_crc = tmc4671_uart_crc8(rx, 7);
            if (calc_crc == rx[7]) {
                /* Return data from response (bytes 3-6, Big Endian) */
                return ((uint32_t)rx[3] << 24) |
                       ((uint32_t)rx[4] << 16) |
                       ((uint32_t)rx[5] << 8)  |
                       ((uint32_t)rx[6]);
            }
        }
    }
    
    return 0;
}

/* Register Read/Write - uses selected interface */
uint32_t tmc4671_read_reg(uint8_t address)
{
    if (tmc4671_interface == TMC4671_IF_UART) {
        return tmc4671_uart_transfer(address, 0, false);
    } else {
        return tmc4671_spi_transfer(address, 0, false);
    }
}

void tmc4671_write_reg(uint8_t address, uint32_t value)
{
    if (tmc4671_interface == TMC4671_IF_UART) {
        tmc4671_uart_transfer(address, value, true);
    } else {
        tmc4671_spi_transfer(address, value, true);
    }
}

/* Interface Selection */
void tmc4671_set_interface(tmc4671_interface_t interface)
{
    tmc4671_interface = interface;
}

/* Register Cache Synchronization */
void tmc4671_read_reg_cache(uint8_t address)
{
    uint32_t value = tmc4671_read_reg(address);
    
    switch (address) {
        case TMC4671_REG_ENABLE:
            tmc4671_regs.enable = value;
            break;
        case TMC4671_REG_MOTOR_TYPE_N_POLE_PAIRS:
            tmc4671_regs.motor_type_n_pole_pairs.all = value;
            break;
        case TMC4671_REG_PWM_MAXCNT:
            tmc4671_regs.pwm_maxcnt = (uint16_t)value;
            break;
        case TMC4671_REG_PWM_POLARITIES:
            tmc4671_regs.pwm_polarities = value;
            break;
        case TMC4671_REG_UQ_UD_EXT:
            tmc4671_regs.uq_ud_ext = value;
            break;
        case TMC4671_REG_VELOCITY_SELECTION:
            tmc4671_regs.velocity_selection.all = value;
            break;
        case TMC4671_REG_POSITION_SELECTION:
            tmc4671_regs.position_selection.all = value;
            break;
        case TMC4671_REG_PHI_E_SELECTION:
            tmc4671_regs.phi_e_selection.all = value;
            break;
        case TMC4671_REG_MODE_RAMP_MODE_MOTION:
            tmc4671_regs.mode_ramp_mode_motion.all = value;
            break;
        case TMC4671_REG_TARGET_POSITION:
            tmc4671_regs.target_position = (int32_t)value;
            break;
        case TMC4671_REG_TARGET_VELOCITY:
            tmc4671_regs.target_velocity = (int32_t)value;
            break;
        case TMC4671_REG_TARGET_TORQUE:
            tmc4671_regs.target_torque = (int32_t)value;
            break;
        case TMC4671_REG_ACTUAL_POSITION:
            tmc4671_regs.actual_position = (int32_t)value;
            break;
        case TMC4671_REG_ACTUAL_VELOCITY:
            tmc4671_regs.actual_velocity = (int32_t)value;
            break;
        case TMC4671_REG_ACTUAL_TORQUE:
            tmc4671_regs.actual_torque = (int32_t)value;
            break;
        case TMC4671_REG_PID_TORQUE_FLUX_P:
            tmc4671_regs.pid_torque_p = (uint16_t)value;
            break;
        case TMC4671_REG_PID_TORQUE_FLUX_I:
            tmc4671_regs.pid_torque_i = (uint16_t)value;
            break;
        case TMC4671_REG_PID_VELOCITY_P:
            tmc4671_regs.pid_velocity_p = (uint16_t)value;
            break;
        case TMC4671_REG_PID_VELOCITY_I:
            tmc4671_regs.pid_velocity_i = (uint16_t)value;
            break;
        case TMC4671_REG_PID_POSITION_P:
            tmc4671_regs.pid_position_p = (uint16_t)value;
            break;
        case TMC4671_REG_PID_POSITION_I:
            tmc4671_regs.pid_position_i = (uint16_t)value;
            break;
        default:
            break;
    }
}

void tmc4671_write_reg_cache(uint8_t address)
{
    uint32_t value = 0;
    
    switch (address) {
        case TMC4671_REG_ENABLE:
            value = tmc4671_regs.enable;
            break;
        case TMC4671_REG_MOTOR_TYPE_N_POLE_PAIRS:
            value = tmc4671_regs.motor_type_n_pole_pairs.all;
            break;
        case TMC4671_REG_PWM_MAXCNT:
            value = (uint32_t)tmc4671_regs.pwm_maxcnt;
            break;
        case TMC4671_REG_PWM_POLARITIES:
            value = tmc4671_regs.pwm_polarities;
            break;
        case TMC4671_REG_UQ_UD_EXT:
            value = tmc4671_regs.uq_ud_ext;
            break;
        case TMC4671_REG_VELOCITY_SELECTION:
            value = tmc4671_regs.velocity_selection.all;
            break;
        case TMC4671_REG_POSITION_SELECTION:
            value = tmc4671_regs.position_selection.all;
            break;
        case TMC4671_REG_PHI_E_SELECTION:
            value = tmc4671_regs.phi_e_selection.all;
            break;
        case TMC4671_REG_MODE_RAMP_MODE_MOTION:
            value = tmc4671_regs.mode_ramp_mode_motion.all;
            break;
        case TMC4671_REG_TARGET_POSITION:
            value = (uint32_t)tmc4671_regs.target_position;
            break;
        case TMC4671_REG_TARGET_VELOCITY:
            value = (uint32_t)tmc4671_regs.target_velocity;
            break;
        case TMC4671_REG_TARGET_TORQUE:
            value = (uint32_t)tmc4671_regs.target_torque;
            break;
        case TMC4671_REG_PID_TORQUE_FLUX_P:
            value = (uint32_t)tmc4671_regs.pid_torque_p;
            break;
        case TMC4671_REG_PID_TORQUE_FLUX_I:
            value = (uint32_t)tmc4671_regs.pid_torque_i;
            break;
        case TMC4671_REG_PID_VELOCITY_P:
            value = (uint32_t)tmc4671_regs.pid_velocity_p;
            break;
        case TMC4671_REG_PID_VELOCITY_I:
            value = (uint32_t)tmc4671_regs.pid_velocity_i;
            break;
        case TMC4671_REG_PID_POSITION_P:
            value = (uint32_t)tmc4671_regs.pid_position_p;
            break;
        case TMC4671_REG_PID_POSITION_I:
            value = (uint32_t)tmc4671_regs.pid_position_i;
            break;
        default:
            return;  /* Unknown register */
    }
    
    tmc4671_write_reg(address, value);
}

void tmc4671_sync_all_regs(void)
{
    /* Read all important registers from hardware to cache */
    tmc4671_read_reg_cache(TMC4671_REG_ENABLE);
    tmc4671_read_reg_cache(TMC4671_REG_MOTOR_TYPE_N_POLE_PAIRS);
    tmc4671_read_reg_cache(TMC4671_REG_PWM_MAXCNT);
    tmc4671_read_reg_cache(TMC4671_REG_PWM_POLARITIES);
    tmc4671_read_reg_cache(TMC4671_REG_UQ_UD_EXT);
    tmc4671_read_reg_cache(TMC4671_REG_VELOCITY_SELECTION);
    tmc4671_read_reg_cache(TMC4671_REG_POSITION_SELECTION);
    tmc4671_read_reg_cache(TMC4671_REG_PHI_E_SELECTION);
    tmc4671_read_reg_cache(TMC4671_REG_MODE_RAMP_MODE_MOTION);
    tmc4671_read_reg_cache(TMC4671_REG_TARGET_POSITION);
    tmc4671_read_reg_cache(TMC4671_REG_ACTUAL_POSITION);
    tmc4671_read_reg_cache(TMC4671_REG_ACTUAL_VELOCITY);
    tmc4671_read_reg_cache(TMC4671_REG_ACTUAL_TORQUE);
    tmc4671_read_reg_cache(TMC4671_REG_PID_TORQUE_FLUX_P);
    tmc4671_read_reg_cache(TMC4671_REG_PID_TORQUE_FLUX_I);
    tmc4671_read_reg_cache(TMC4671_REG_PID_VELOCITY_P);
    tmc4671_read_reg_cache(TMC4671_REG_PID_VELOCITY_I);
    tmc4671_read_reg_cache(TMC4671_REG_PID_POSITION_P);
    tmc4671_read_reg_cache(TMC4671_REG_PID_POSITION_I);
}

/* Initialization */
void tmc4671_init(void)
{
    /* Initialize TMC4671 for Maxon DCX08M EB KL 2.4V motor */
    
    /* Set motor type: DC motor, 1 pole pair */
    tmc4671_regs.motor_type_n_pole_pairs.bits.motor_type = 1;        /* Decimal: 1 = DC motor */
    tmc4671_regs.motor_type_n_pole_pairs.bits.n_pole_pairs = 1;      /* Decimal: 1 pole pair */
    tmc4671_write_reg_cache(TMC4671_REG_MOTOR_TYPE_N_POLE_PAIRS);
    
    /* Configure PWM for H-bridge - 25kHz */
    tmc4671_regs.pwm_maxcnt = 3999;  /* Decimal: 3999 → 100MHz/(3999+1) = 25kHz */
    tmc4671_regs.pwm_polarities = 0;  /* Decimal: 0 = normal polarity */
    tmc4671_write_reg_cache(TMC4671_REG_PWM_MAXCNT);
    tmc4671_write_reg_cache(TMC4671_REG_PWM_POLARITIES);
    
    /* Set feedback sources (open loop for basic operation) */
    tmc4671_regs.velocity_selection.bits.source = 0;  /* Decimal: 0 = open loop */
    tmc4671_regs.position_selection.bits.source = 0;  /* Decimal: 0 = open loop */
    tmc4671_regs.phi_e_selection.bits.source = 0;     /* Decimal: 0 = open loop */
    tmc4671_write_reg_cache(TMC4671_REG_VELOCITY_SELECTION);
    tmc4671_write_reg_cache(TMC4671_REG_POSITION_SELECTION);
    tmc4671_write_reg_cache(TMC4671_REG_PHI_E_SELECTION);
    
    /* Set basic PID gains (conservative starting values) */
    tmc4671_regs.pid_torque_p = 128;      /* Decimal: 128 */
    tmc4671_regs.pid_torque_i = 32;       /* Decimal: 32 */
    tmc4671_regs.pid_velocity_p = 128;    /* Decimal: 128 */
    tmc4671_regs.pid_velocity_i = 32;     /* Decimal: 32 */
    tmc4671_regs.pid_position_p = 256;     /* Decimal: 256 */
    tmc4671_regs.pid_position_i = 64;     /* Decimal: 64 */
    tmc4671_write_reg_cache(TMC4671_REG_PID_TORQUE_FLUX_P);
    tmc4671_write_reg_cache(TMC4671_REG_PID_TORQUE_FLUX_I);
    tmc4671_write_reg_cache(TMC4671_REG_PID_VELOCITY_P);
    tmc4671_write_reg_cache(TMC4671_REG_PID_VELOCITY_I);
    tmc4671_write_reg_cache(TMC4671_REG_PID_POSITION_P);
    tmc4671_write_reg_cache(TMC4671_REG_PID_POSITION_I);
    
    /* Set default mode: position control */
    tmc4671_regs.mode_ramp_mode_motion.bits.motion_mode = 0;  /* Decimal: 0 = position mode */
    tmc4671_write_reg_cache(TMC4671_REG_MODE_RAMP_MODE_MOTION);
    
    /* Sync all registers from hardware to cache */
    tmc4671_sync_all_regs();
}

/* Motor Control */
void tmc4671_enable(void)
{
    tmc4671_regs.enable = 1;  /* Decimal: 1 = enabled */
    tmc4671_write_reg_cache(TMC4671_REG_ENABLE);
}

void tmc4671_disable(void)
{
    tmc4671_regs.enable = 0;  /* Decimal: 0 = disabled */
    tmc4671_write_reg_cache(TMC4671_REG_ENABLE);
}

void tmc4671_set_target_position(int32_t position)
{
    tmc4671_regs.target_position = position;
    tmc4671_write_reg_cache(TMC4671_REG_TARGET_POSITION);
}

int32_t tmc4671_get_actual_position(void)
{
    tmc4671_read_reg_cache(TMC4671_REG_ACTUAL_POSITION);
    return tmc4671_regs.actual_position;
}

bool tmc4671_is_moving(void)
{
    int32_t actual = tmc4671_get_actual_position();
    int32_t target = tmc4671_regs.target_position;
    
    /* Motor is moving if position not reached (with small tolerance) */
    int32_t error = actual - target;
    if (error < 0) error = -error;
    return (error > 10);  /* 10 ticks tolerance */
}

/* Encoder Configuration */
void tmc4671_config_encoder_abn(void)
{
    /* Configure for closed-loop position control with ENX 8 MAG 256IMP encoder */
    tmc4671_regs.velocity_selection.bits.source = 3;  /* Decimal: 3 = ABN encoder */
    tmc4671_regs.position_selection.bits.source = 3;  /* Decimal: 3 = ABN encoder */
    tmc4671_regs.phi_e_selection.bits.source = 3;     /* Decimal: 3 = ABN encoder */
    tmc4671_write_reg_cache(TMC4671_REG_VELOCITY_SELECTION);
    tmc4671_write_reg_cache(TMC4671_REG_POSITION_SELECTION);
    tmc4671_write_reg_cache(TMC4671_REG_PHI_E_SELECTION);
    
    /* Set motion mode to position control */
    tmc4671_regs.mode_ramp_mode_motion.bits.motion_mode = 0;  /* Decimal: 0 = position mode */
    tmc4671_write_reg_cache(TMC4671_REG_MODE_RAMP_MODE_MOTION);
}

/* Position Movement - Convert mm to encoder ticks */
static int32_t mm_to_ticks(float mm)
{
    /* mm → output shaft revolutions → motor shaft revolutions → encoder ticks */
    float output_revolutions = mm / LEADSCREW_LEAD_MM;
    float motor_revolutions = output_revolutions * GEAR_RATIO;
    return (int32_t)(motor_revolutions * ENCODER_EFFECTIVE_COUNTS);
}

void tmc4671_move_by_mm(float mm)
{
    /* Get current position */
    int32_t current_pos = tmc4671_get_actual_position();
    
    /* Convert mm to ticks */
    int32_t ticks = mm_to_ticks(mm);
    
    /* Set target position */
    tmc4671_set_target_position(current_pos + ticks);
}

