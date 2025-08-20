# MotionTracker

Real-time multi-sensory human upper limb motion recording system based on STM32 microcontroller.

---
## Overview

This project implements a real-time motion registration system using the **STM32 NUCLEO-L476RG** microcontroller. It interfaces with up to **five incremental optical encoders**, logs time-synchronized position data to an **SD card**, and optionally streams the data via **UART**.

Originally, the purpose of this project was to register the motion of a **human upper extremity**, based on a **5-degree-of-freedom limb model**, using a **passive exoskeleton** within which the encoders were mounted. However, it can be easily adapted to any **STM32-based kinematic system**.

You can read more about the passive exoskeleton upper extremity motion registration system [here](https://link.springer.com/chapter/10.1007/978-3-031-78266-4_14).  

The utilisation of a system is presented below:


![usecase](https://github.com/user-attachments/assets/883334ab-c5f7-4cdb-a715-13bd2e9231f1)


---

## Main features

- Real-time encoder data acquisition (5 channels)
- SD card logging using FatFs (via SPI)
- Optional UART output for live monitoring

To receive data from UART simply run the script below while connected with the device:

```python
import serial
import time

# Open port COM3 with 115200 baud rate - modify it based on your setup 
conn = serial.Serial("COM3", 115200, timeout=1)

time.sleep(1)  

while True:
    line = conn.readline().decode(errors="ignore").strip()
    if line:
        print(line)
```

---

## Hardware Requirements

- STM32 NUCLEO-L476RG board (or similar)
- Micro SD card module with reader module  (SPI interface) 
- 5x Incremental optical rotary encoders
- Push-button (for stopping data logging)
- UART-to-USB interface (optional, for PC communication)

---

## Project Structure

```bash
.
├── Core/
│   └── Src/
│       └── main.c  # Main application logic
├── Inc/
│   └── main.h  

