# Mini Car Project

## Project Description

This project involves building and controlling a mini car using an ESP32 microcontroller. The car is equipped with two motors and uses a joystick for manual control. The system also monitors wheel speed via encoder sensors to calculate real-time speed and direction.
The project demonstrates the use of several ESP32 peripherals, including GPIO, ADC, LEDC (PWM), timers, and interrupts, to create a responsive and efficient system.

---

## Features

- **Joystick-based Control**:
  - X-axis and Y-axis inputs from an analog joystick to control speed and direction.
  - Processes joystick input to adjust motor speed dynamically.

- **Motor Control**:
  - Dual-motor configuration with PWM for smooth speed control.
  - Forward and reverse directions with adjustable speed limits.

- **Wheel Speed Monitoring**:
  - Real-time counting of encoder pulses using GPIO interrupts.
  - Independent counters for left and right wheels.

- **Peripheral Integration**:
  - Continuous ADC mode for joystick input.
  - Timers for periodic tasks and time-sensitive operations.
  - PWM via LEDC for precise motor speed control.

---

## Hardware and Peripherals

### 1. **ESP32 Development Board**
   - **Purpose**: Central processing unit.
   - **Key Features**: Dual-core processor, SPI, I2C, UART, GPIO, ADC, PWM, and timer capabilities.

### 2. **Joystick**
   - **Connection**: Two ADC channels (`ADC_CHANNEL_0` and `ADC_CHANNEL_3`).
   - **Purpose**: Provides analog input for controlling motor speed and direction.

### 3. **Motors**
   - **Type**: Two DC motors with H-bridge drivers.
   - **Control Pins**:
     - `GPIO18`, `GPIO19`: Motor A direction.
     - `GPIO32`, `GPIO33`: Motor B direction.
     - `GPIO25`, `GPIO26`: PWM speed control (LEDC).

### 4. **Wheel Encoders**
   - **Purpose**: Generate pulse signals for tracking wheel speed.
   - **Connection**: `GPIO18` (left wheel) and `GPIO19` (right wheel).
   - **Counting**: Uses GPIO interrupts to increment counters.

---


## Software Functionality

### Key Modules

1. **Joystick Input**:
   - Uses ADC in continuous mode to read joystick X and Y-axis values.
   - Maps raw joystick values to motor speed and direction.

2. **Motor Control**:
   - Dynamically adjusts motor speed and direction based on joystick input.
   - Configures GPIO pins for H-bridge control and LEDC for PWM.

3. **Wheel Speed Monitoring**:
   - Configures GPIO interrupts to track encoder pulses for both wheels.
   - Uses independent counters to track left and right wheel speeds.

4. **Timers**:
   - Uses a periodic timer for tasks like speed calculations.

---

## Setup and Execution

### Hardware Connections
- **Joystick**:
  - X-axis: `ADC_CHANNEL_0` (GPIO pin corresponding to ADC).
  - Y-axis: `ADC_CHANNEL_3` (GPIO pin corresponding to ADC).

- **Motors**:
  - Motor A:
    - Direction: `GPIO18` (IN1), `GPIO19` (IN2).
    - PWM Speed: `GPIO25`.
  - Motor B:
    - Direction: `GPIO32` (IN3), `GPIO33` (IN4).
    - PWM Speed: `GPIO26`.

- **Wheel Encoders**:
  - Left Wheel: Connected to `GPIO18`.
  - Right Wheel: Connected to `GPIO19`.

### Build and Flash
1. **Set up ESP-IDF environment**:
   Follow the [ESP-IDF Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html).

2. **Compile and Flash**:
   ```bash
   idf.py build flash monitor
   ```

3. **Run the Code**:
   - Observe joystick values and motor speeds in the terminal.
   - Adjust joystick to control car movement.

---

## Future Enhancements

1. **Closed-loop Control**:
   - Implement PID for precise speed control.  
2. **Wireless Control**:
   - Add Bluetooth or Wi-Fi for remote operation.
3. **Speed Display**:
   - Add an OLED or LCD to display real-time speed data.

4. **Obstacle Avoidance**:
   - Integrate distance sensors for autonomous navigation.
5. **Data Logging**:
   - Log encoder data and joystick inputs to an SD card.

---
