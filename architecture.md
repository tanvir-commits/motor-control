# Embedded Software Architecture

Project: STM32U5 + ThreadX  
Target Device: Honeywell FMAMSDXX025WC2C3 Force Sensor

---

## Big Picture

The goal of this architecture is simple:

- Keep application logic clean and portable  
- Isolate all hardware details in one place  
- Make drivers easy to understand and replace  
- Avoid mixing HAL, RTOS, and device logic together  

At PoC stage we care about working code first, but we still want a structure that will survive real product development.

---

## Layer Diagram (Text)

```
+---------------------------------------------------+
| Application                                       |
|  - business logic                                 |
|  - control algorithms                             |
|  - reads cached sensor values                     |
+--------------------------+------------------------+
                           |
                           v
+---------------------------------------------------+
| ThreadX Tasks                                     |
|                                                   |
|  SensorTask                                       |
|   - periodically reads sensors                    |
|   - updates shared sensor cache                   |
|                                                   |
|  Other tasks (CLI / Comms / Control)              |
|   - only read cached values                       |
|   - do NOT talk to hardware directly              |
+--------------------------+------------------------+
                           |
                           v
+---------------------------------------------------+
| Device Drivers (Portable)                         |
|  - decode sensor data                             |
|  - convert to engineering units                   |
|  - no HAL calls                                   |
|  - no ThreadX calls                               |
+--------------------------+------------------------+
                           |
                           v
+---------------------------------------------------+
| PAL / PIO (Platform Abstraction Layer)            |
|  - simple hardware access API                     |
|  - wraps STM32 HAL                                |
|  - provides I2C / SPI / ADC / time functions      |
|  - only layer allowed to touch HAL                |
+--------------------------+------------------------+
                           |
                           v
+---------------------------------------------------+
| STM32 HAL + MCU Peripherals                       |
+--------------------------+------------------------+
                           |
                           v
+---------------------------------------------------+
| Physical Hardware (Sensors, Buses, Pins)          |
+---------------------------------------------------+
```

---

## Ground Rules

### 1. One owner of hardware
- The SensorTask is the only code that actually reads sensors.
- All other threads simply read the cached values.
- No random task is allowed to touch I2C directly.

### 2. Clean drivers
- Sensor drivers must be plain C code.
- They should not include STM32 HAL headers or ThreadX headers.
- Drivers only call the PAL/PIO API.

### 3. PAL/PIO is the boundary
- All HAL calls live in PAL/PIO files.
- Drivers and application code never see HAL functions.

### 4. Keep it simple for PoC
At this stage:

- No mutex logic  
- No retries  
- No complex error frameworks  

We just want a clean structure that works.

---

## Folder Layout (PoC)

```
app/
  tasks/
    sensor_task.c

drivers/
  force/
    honeywell_fmam_force.c
    honeywell_fmam_force.h

pio/
  include/
    pio.h
  pio_i2c.c
  pio_time.c

docs/
  architecture.md
  detail_design_template.md
```

---

## Naming Conventions

- Use pio_*.c for platform I/O files  
- Avoid hal_* (conflicts with STM32 HAL)  
- Avoid hw_* unless writing real low-level register code  
