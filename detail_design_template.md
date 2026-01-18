# Detailed Design – Sensor Integration Examples

This document describes the minimal design for integrating sensors into the system during PoC. It includes examples for both I2C (force sensor) and SPI (magnetic sensor) interfaces.

---

## 1. Purpose

Provide simple drivers to:

- Read raw sensor data from multiple sensors (I2C and SPI interfaces)
- Convert data into usable values
- Publish results to the system cache

The drivers should be easy to understand and free of HAL/RTOS dependencies.

---

## 2. Responsibilities

### Driver is responsible for:
- Talking to the sensor through the PAL/PIO API  
- Decoding the sensor data frame  
- Converting counts to a basic value  

### Driver is NOT responsible for:
- Threading  
- HAL calls  
- Timing delays  

Those belong to other layers.

---

## 3. Interfaces

### Driver API

`drivers/force/honeywell_fmam_force.h`

```c
#pragma once
#include <stdint.h>

typedef struct {
    uint16_t raw_counts;
    float    value;
} fmam_force_sample_t;

void fmam_force_init(void);
fmam_force_sample_t fmam_force_read(void);
```

---

### Platform I/O API (used by the driver)

`pio/include/pio.h`

```c
#pragma once
#include <stdint.h>

/* I2C functions */
void pio_i2c_read(uint8_t addr_7b, uint8_t* rx, uint16_t rx_len);

/* SPI functions */
void pio_spi_transfer(uint8_t* tx, uint8_t* rx, uint16_t len);
```

---

## 4. Force Sensor Details

- Device: Honeywell FMAMSDXX025WC2C3  
- Interface: I2C  
- 7-bit address: 0x28  

The sensor returns a small data frame containing status bits and force counts.

---

## 4A. Magnetic Sensor Details

- Device: MPS MA782GGU-Z  
- Interface: SPI  
- Chip Select: GPIO pin (handled by PAL/PIO layer)

The sensor returns magnetic field data via SPI transfer.

---

## 5. Force Sensor Implementation

### Driver Code (PoC level)

`drivers/force/honeywell_fmam_force.c`

```c
#include "honeywell_fmam_force.h"
#include "pio.h"

#define FMAM_I2C_ADDR  (0x28u)

void fmam_force_init(void)
{
    /* Nothing required for PoC */
}

fmam_force_sample_t fmam_force_read(void)
{
    uint8_t rx[4] = {0};

    pio_i2c_read(FMAM_I2C_ADDR, rx, (uint16_t)sizeof(rx));

    uint16_t counts =
        (uint16_t)(((uint16_t)(rx[0] & 0x3Fu) << 8) | rx[1]);

    fmam_force_sample_t s;
    s.raw_counts = counts;
    s.value = (float)counts;

    return s;
}
```

---

## 5A. Magnetic Sensor Implementation

### Driver API

`drivers/magnetic/mps_ma782.h`

```c
#pragma once
#include <stdint.h>

typedef struct {
    uint16_t raw_data;
    float    value;
} ma782_sample_t;

void ma782_init(void);
ma782_sample_t ma782_read(void);
```

### Driver Code (PoC level)

`drivers/magnetic/mps_ma782.c`

```c
#include "mps_ma782.h"
#include "pio.h"

void ma782_init(void)
{
    /* Nothing required for PoC */
}

ma782_sample_t ma782_read(void)
{
    uint8_t tx[2] = {0x00, 0x00};  /* Read command */
    uint8_t rx[2] = {0};

    pio_spi_transfer(tx, rx, 2);

    uint16_t data = (uint16_t)((rx[0] << 8) | rx[1]);

    ma782_sample_t s;
    s.raw_data = data;
    s.value = (float)data;  /* Basic conversion for PoC */

    return s;
}
```

---

## 6. Sensor Cache Structure

The sensor cache is a shared data structure that holds the latest sensor readings. This allows other tasks to read sensor values without directly accessing hardware.

`app/tasks/sensor_cache.h`

```c
#pragma once
#include <stdint.h>

typedef struct {
    uint16_t raw;
    float    value;
} force_sensor_t;

typedef struct {
    uint16_t raw_data;
    float    value;
} magnetic_sensor_t;

typedef struct {
    force_sensor_t    force;
    magnetic_sensor_t magnetic;
    /* Add other sensors here as needed */
} sensor_cache_t;

extern sensor_cache_t sensors;
```

`app/tasks/sensor_cache.c`

```c
#include "sensor_cache.h"

sensor_cache_t sensors = {0};
```

---

## 7. Integration with SensorTask

```c
#include "sensor_cache.h"
#include "honeywell_fmam_force.h"
#include "mps_ma782.h"

void SensorTaskLoop(void)
{
    /* Read I2C force sensor */
    fmam_force_sample_t force_sample = fmam_force_read();
    sensors.force.raw = force_sample.raw_counts;
    sensors.force.value = force_sample.value;

    /* Read SPI magnetic sensor */
    ma782_sample_t mag_sample = ma782_read();
    sensors.magnetic.raw_data = mag_sample.raw_data;
    sensors.magnetic.value = mag_sample.value;
}
```

Other tasks simply read the cached values:

```c
#include "sensor_cache.h"

float current_force = sensors.force.value;
float current_magnetic = sensors.magnetic.value;
```

---

## 8. PAL/PIO Implementation (concept)

### I2C Implementation

`pio/pio_i2c.c`

```c
#include "pio.h"
#include "stm32u5xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

void pio_i2c_read(uint8_t addr_7b, uint8_t* rx, uint16_t rx_len)
{
    HAL_I2C_Master_Receive(&hi2c1,
                           (uint16_t)(addr_7b << 1),
                           rx,
                           rx_len,
                           10);
}
```

### SPI Implementation

`pio/pio_spi.c`

```c
#include "pio.h"
#include "stm32u5xx_hal.h"

extern SPI_HandleTypeDef hspi1;

void pio_spi_transfer(uint8_t* tx, uint8_t* rx, uint16_t len)
{
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, len, 10);
}
```

---

## 9. Future Improvements (After PoC)

Later stages may add:

- Correct scaling using sensor transfer functions  
- Proper frame status decoding  
- Unit tests  
- Error handling and retry logic

None of that is required to get a working prototype.
