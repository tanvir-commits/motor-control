# TMC4671 Motor Control - Minimal Code Files

This directory contains the minimal code needed to:
1. Read and write TMC4671 registers (verify SPI communication)
2. Move the motor by 20mm using encoder feedback

## Files Created

### 1. `drivers/motor/tmc4671.h`
- Register address definitions
- Bitfield structures for registers
- Register cache structure (debugger-friendly)
- Function declarations

### 2. `drivers/motor/tmc4671.c`
- SPI communication functions
- Register read/write functions
- Register cache synchronization
- Motor initialization
- Encoder configuration
- Position movement functions (mm to ticks conversion)

### 3. `app/tasks/motor_thread.c`
- ThreadX thread for motor control
- Example: Move motor by 20mm, wait, move back

### 4. `app/tasks/motor_test.c`
- Test functions to verify register read/write
- Test function to move motor by 20mm

## Quick Start

### Step 1: Select Communication Interface

```c
#include "tmc4671.h"

void setup_motor(void)
{
    /* Option 1: Use SPI (default) */
    tmc4671_set_interface(TMC4671_IF_SPI);
    
    /* Option 2: Use UART/LPUART */
    /* tmc4671_set_interface(TMC4671_IF_UART); */
    
    /* Initialize motor */
    tmc4671_init();
}
```

### Step 2: Verify Register Read/Write

```c
#include "motor_test.h"

void test_communication(void)
{
    /* Test basic read/write */
    test_register_read_write();
    
    /* Read all registers into cache */
    test_read_all_registers();
    
    /* Check tmc4671_regs in debugger watch window */
}
```

### Step 3: Move Motor by 20mm

```c
#include "motor_thread.h"

void main(void)
{
    /* Initialize ThreadX, SPI/UART, etc. */
    
    /* Create motor thread */
    motor_thread_create();
    
    /* Or call directly: */
    test_move_20mm();
}
```

## Configuration Required

### 1. Communication Interface Selection

**For SPI (default):**
```c
/* In your HAL/GPIO configuration file */
GPIO_TypeDef* TMC4671_CS_PORT = GPIOA;  /* Adjust to your pin */
uint16_t TMC4671_CS_PIN = GPIO_PIN_4;   /* Adjust to your pin */

/* Select SPI interface */
tmc4671_set_interface(TMC4671_IF_SPI);
```

**For UART/LPUART (alternative):**
```c
/* Configure LPUART in HAL:
 * - Baud rate: 115200 (default, can be changed)
 * - Data bits: 8
 * - Stop bits: 1
 * - Parity: None
 * - Flow control: None
 * 
 * Connection:
 * - MCU TX → TMC4671 RXD
 * - MCU RX ← TMC4671 TXD
 * - Common ground
 */

/* Select UART interface */
tmc4671_set_interface(TMC4671_IF_UART);
```

**UART Protocol Details:**
- Sync byte: 0x05
- Node address: 0x01 (default)
- Frame: Sync(1) + Node(1) + Address(1) + Data(4) + CRC(1) = 8 bytes
- CRC-8 polynomial: 0x07

### 2. PAL/PIO SPI Function (for SPI interface)
The code calls `pio_spi_transfer_cs()` which should be implemented in your PAL/PIO layer:

```c
/* In pio/pio_spi1.c */
void pio_spi_transfer_cs(uint8_t cs_pin, uint8_t* tx, uint8_t* rx, uint16_t len)
{
    /* Assert chip select */
    HAL_GPIO_WritePin(TMC4671_CS_PORT, TMC4671_CS_PIN, GPIO_PIN_RESET);
    
    /* Transfer data via SPI */
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, len, 100);
    
    /* Deassert chip select */
    HAL_GPIO_WritePin(TMC4671_CS_PORT, TMC4671_CS_PIN, GPIO_PIN_SET);
}
```

### 2b. PAL/PIO UART Function (for UART/LPUART interface - optional)
If using UART/LPUART instead of SPI, implement `pio_uart_transfer()`:

```c
/* In pio/pio_uart.c */
void pio_uart_transfer(uint8_t* tx, uint16_t tx_len, uint8_t* rx, uint16_t rx_len)
{
    /* Transmit data */
    if (tx != NULL && tx_len > 0) {
        HAL_UART_Transmit(&hlpuart1, tx, tx_len, 100);
    }
    
    /* Receive data */
    if (rx != NULL && rx_len > 0) {
        HAL_UART_Receive(&hlpuart1, rx, rx_len, 100);
    }
}
```

**Note:** The implementation in `pio/pio_uart.c` is already provided. Just configure `hlpuart1` in your HAL.

### 3. Leadscrew Configuration
If your application doesn't use a leadscrew, adjust `LEADSCREW_LEAD_MM` in `tmc4671.c`:

```c
/* For direct drive wheel: */
#define WHEEL_CIRCUMFERENCE_MM  50.0  /* mm per output shaft revolution */

/* Then modify mm_to_ticks() function accordingly */
```

## Register Count Summary

**For basic read/write test:**
- 1 register (ENABLE) - minimum to test SPI

**For open-loop operation:**
- 7 registers (no PID needed)

**For closed-loop position control (20mm movement):**
- 13 registers (7 basic + 6 PID)

## Debugger Usage

Add to watch window:
- `tmc4671_regs` - View all register values
- `tmc4671_regs.actual_position` - Monitor motor position
- `tmc4671_regs.target_position` - See target position
- `tmc4671_regs.enable` - Check if motor is enabled

You can modify register values in the debugger and call `tmc4671_write_reg_cache()` to write them to hardware.

## Motor Combo Specifications

- **Motor**: Maxon DCX08M EB KL 2.4V
- **Encoder**: ENX 8 MAG 256IMP (256 counts/rev, 1024 effective with 4× quadrature)
- **Gearhead**: GPX08 A 64:1
- **Output Resolution**: 65,536 ticks per output shaft revolution

## Testing Workflow

1. **First**: Test SPI communication with `test_register_read_write()`
2. **Second**: Initialize motor with `tmc4671_init()`
3. **Third**: Configure encoder with `tmc4671_config_encoder_abn()`
4. **Fourth**: Enable motor with `tmc4671_enable()`
5. **Fifth**: Move by 20mm with `tmc4671_move_by_mm(20.0f)`

## Notes

- All register values are in decimal format
- Position is in encoder ticks (motor shaft, not output shaft)
- The `mm_to_ticks()` function accounts for gear ratio (64:1)
- Adjust `LEADSCREW_LEAD_MM` based on your mechanical setup

