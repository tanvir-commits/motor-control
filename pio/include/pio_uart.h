#pragma once
#include <stdint.h>

/* UART/LPUART Transfer Function for TMC4671 */
/* Transmits tx buffer and receives into rx buffer */
/* Both buffers must be at least len bytes */
void pio_uart_transfer(uint8_t* tx, uint16_t tx_len, uint8_t* rx, uint16_t rx_len);

