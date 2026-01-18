/*
 * Motor Test - Verify Register Read/Write
 * 
 * This file contains test functions to verify TMC4671 SPI communication
 * and register read/write functionality.
 */

#include "tmc4671.h"
#include "tx_api.h"

/* Test: Read and write a register to verify SPI communication */
void test_register_read_write(void)
{
    uint32_t read_value;
    uint32_t write_value = 0x12345678;
    
    /* Test 1: Write to ENABLE register and read back */
    tmc4671_write_reg(TMC4671_REG_ENABLE, 1);
    read_value = tmc4671_read_reg(TMC4671_REG_ENABLE);
    /* Verify: read_value should be 1 */
    
    /* Test 2: Write to PWM_MAXCNT register and read back */
    tmc4671_write_reg(TMC4671_REG_PWM_MAXCNT, 3999);
    read_value = tmc4671_read_reg(TMC4671_REG_PWM_MAXCNT);
    /* Verify: read_value should be 3999 */
    
    /* Test 3: Write to MOTOR_TYPE_N_POLE_PAIRS and read back */
    tmc4671_write_reg(TMC4671_REG_MOTOR_TYPE_N_POLE_PAIRS, 0x00010001);
    read_value = tmc4671_read_reg(TMC4671_REG_MOTOR_TYPE_N_POLE_PAIRS);
    /* Verify: read_value should be 0x00010001 (motor_type=1, pole_pairs=1) */
}

/* Test: Read all important registers */
void test_read_all_registers(void)
{
    /* Read all registers and update cache */
    tmc4671_sync_all_regs();
    
    /* Now check tmc4671_regs in debugger watch window:
     * - tmc4671_regs.enable
     * - tmc4671_regs.motor_type_n_pole_pairs
     * - tmc4671_regs.pwm_maxcnt
     * - tmc4671_regs.actual_position
     * - etc.
     */
}

/* Test: Write all registers using cache */
void test_write_all_registers(void)
{
    /* Initialize cache with values */
    tmc4671_regs.enable = 1;
    tmc4671_regs.motor_type_n_pole_pairs.bits.motor_type = 1;
    tmc4671_regs.motor_type_n_pole_pairs.bits.n_pole_pairs = 1;
    tmc4671_regs.pwm_maxcnt = 3999;
    tmc4671_regs.pwm_polarities = 0;
    tmc4671_regs.velocity_selection.bits.source = 3;  /* ABN encoder */
    tmc4671_regs.position_selection.bits.source = 3;  /* ABN encoder */
    tmc4671_regs.phi_e_selection.bits.source = 3;     /* ABN encoder */
    tmc4671_regs.mode_ramp_mode_motion.bits.motion_mode = 0;  /* Position mode */
    tmc4671_regs.pid_torque_p = 128;
    tmc4671_regs.pid_torque_i = 32;
    tmc4671_regs.pid_velocity_p = 128;
    tmc4671_regs.pid_velocity_i = 32;
    tmc4671_regs.pid_position_p = 256;
    tmc4671_regs.pid_position_i = 64;
    
    /* Write all registers from cache to hardware */
    tmc4671_write_reg_cache(TMC4671_REG_ENABLE);
    tmc4671_write_reg_cache(TMC4671_REG_MOTOR_TYPE_N_POLE_PAIRS);
    tmc4671_write_reg_cache(TMC4671_REG_PWM_MAXCNT);
    tmc4671_write_reg_cache(TMC4671_REG_PWM_POLARITIES);
    tmc4671_write_reg_cache(TMC4671_REG_VELOCITY_SELECTION);
    tmc4671_write_reg_cache(TMC4671_REG_POSITION_SELECTION);
    tmc4671_write_reg_cache(TMC4671_REG_PHI_E_SELECTION);
    tmc4671_write_reg_cache(TMC4671_REG_MODE_RAMP_MODE_MOTION);
    tmc4671_write_reg_cache(TMC4671_REG_PID_TORQUE_FLUX_P);
    tmc4671_write_reg_cache(TMC4671_REG_PID_TORQUE_FLUX_I);
    tmc4671_write_reg_cache(TMC4671_REG_PID_VELOCITY_P);
    tmc4671_write_reg_cache(TMC4671_REG_PID_VELOCITY_I);
    tmc4671_write_reg_cache(TMC4671_REG_PID_POSITION_P);
    tmc4671_write_reg_cache(TMC4671_REG_PID_POSITION_I);
}

/* Test: Move motor by 20mm */
void test_move_20mm(void)
{
    /* Initialize and configure */
    tmc4671_init();
    tmc4671_config_encoder_abn();
    tmc4671_enable();
    
    /* Move by 20mm */
    tmc4671_move_by_mm(20.0f);
    
    /* Wait until movement completes */
    while (tmc4671_is_moving()) {
        tx_thread_sleep(10);
    }
}

