# Fall Detection System

## Overview
This project implements a **real-time fall detection system** using an embedded microcontroller and multiple onboard sensors. The system monitors **acceleration, rotational movement, and altitude changes** to detect potential fall events.

Sensor data from the **accelerometer, gyroscope, and barometric pressure sensor** are processed continuously. A **state machine algorithm** analyzes the data to determine whether a fall has occurred. When a fall is confirmed, the system triggers an **alarm buzzer** and displays a warning message on an **OLED screen**.

Sensor readings and detection states are also transmitted via **UART**, allowing the data to be visualized using external tools such as Python plotting scripts.

---

# Features

## Multi-Sensor Fall Detection
The system combines multiple sensor readings to detect falls more accurately.

Sensors used:

- **Accelerometer** – detects free fall and impact acceleration  
- **Gyroscope** – detects tumbling motion during the fall  
- **Barometric Pressure Sensor** – calculates altitude and detects height changes  

This multi-sensor approach helps reduce false alarms.

---

## State Machine Detection Algorithm

The fall detection algorithm operates using a **state machine**.

| State | Description |
|------|-------------|
| `IDLE` | Normal monitoring state |
| `FREE_FALL` | Low acceleration detected, indicating possible free fall |
| `IMPACT_WAIT` | Impact detected, waiting to confirm fall |
| `FALLEN_INACTIVE` | Fall confirmed and user is inactive |

Transitions between states are triggered by sensor thresholds such as acceleration magnitude, gyro activity, and altitude change.

---

## Emergency Alert System

If a fall is confirmed:

- A **buzzer alarm** is activated
- An **OLED display message** appears

Example display message:
Help me,
I fell down!

This alerts nearby people that assistance may be required.

---

## Real-Time Monitoring via UART

The system sends sensor readings and system status via UART.

Example output:
Accel: 10.39 | g_mag: 1.84 | Alt: 58.86 m | dAlt/dt: 0.22 m/s
State: IDLE | Tumble: NO | RapidAlt: NO | HeightDrop: 0.00 m

This output can be logged using **TeraTerm** and visualized using external scripts.

---

# Hardware Components

The project uses the **STM32L4S5I IoT Discovery Board** with the following sensors:

| Component | Function |
|----------|----------|
| Accelerometer | Measures acceleration along X, Y, Z axes |
| Gyroscope | Measures angular velocity |
| Pressure Sensor | Measures atmospheric pressure for altitude calculation |
| OLED Display | Displays emergency message |
| Buzzer | Audible alert when fall detected |
| LED | Status indication |

---

# Software Architecture

## Sensor Processing
Raw sensor readings are filtered using a **moving average filter** implemented in both:

- C implementation
- Assembly implementation

This improves signal stability and smooths noisy sensor data.

---

## Fall Detection Algorithm
The algorithm evaluates multiple conditions before confirming a fall:

- Acceleration magnitude
- Gyroscope magnitude
- Altitude rate of change
- Height dropped
- User inactivity

Combining these conditions improves detection accuracy.

---

## Communication
Sensor data and system status are transmitted through **UART** for monitoring and debugging.

---

# Installation and Setup

## 1. Hardware Setup

Connect the STM32 board and ensure the following peripherals are working:

- Accelerometer
- Gyroscope
- Pressure sensor
- OLED display
- Buzzer

---

## 2. Build the Project

Compile the project using **STM32CubeIDE**.

Steps:

1. Open the project in STM32CubeIDE  
2. Build the project  
3. Flash the firmware to the STM32 board  

---

## 3. Monitor UART Output

Use **TeraTerm** or another serial terminal.

Recommended settings:

| Parameter | Value |
|----------|------|
| Baud Rate | 115200 |
| Data Bits | 8 |
| Parity | None |
| Stop Bits | 1 |

This allows real-time monitoring of sensor data.

---

# Using the System

## Normal Operation
When powered on, the system continuously monitors sensor readings in the **IDLE state**.

The LED blinks slowly to indicate the system is active.

---

## Simulating a Fall

A fall event can be simulated by:

1. Dropping the device from a small height  
2. Allowing the system to detect:
   - Free fall
   - Impact
   - Tumbling motion
   - Height change  

If all conditions are satisfied, the system transitions to **FALLEN_INACTIVE**.

---

## Fall Detected

When a fall is confirmed:

- The **buzzer alarm activates**
- The **OLED screen displays an emergency message**
- The system remains in alert mode until reset.

---

# Example Detection Flow
IDLE
↓
FREE_FALL
↓
IMPACT_WAIT
↓
FALLEN_INACTIVE


---

# Known Limitations

- Small drops may not trigger a fall event.
- Sudden device movements may produce false triggers.
- Sensor noise may affect altitude estimation.

Threshold tuning may improve detection accuracy.

---

# Future Improvements

Possible enhancements include:

- Wireless alert notifications
- Mobile application integration
- Machine learning fall detection
- Battery-powered wearable implementation

---
