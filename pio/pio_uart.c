#include "pio_uart.h"
#include "stm32u5xx_hal.h"

/* External LPUART handle - configure in your HAL */
extern UART_HandleTypeDef hlpuart1;

/* UART/LPUART Transfer Function for TMC4671 */
void pio_uart_transfer(uint8_t* tx, uint16_t tx_len, uint8_t* rx, uint16_t rx_len)
{
    /* Transmit data */
    if (tx != NULL && tx_len > 0) {
        HAL_UART_Transmit(&hlpuart1, tx, tx_len, 100);
    }
    
    /* Receive data (if rx buffer provided) */
    if (rx != NULL && rx_len > 0) {
        HAL_UART_Receive(&hlpuart1, rx, rx_len, 100);
    }
}

