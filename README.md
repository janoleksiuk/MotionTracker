# MotionTrack-STM32

> Real-Time Multi-Encoder Motion Logging System for STM32

This project implements a real-time motion registration system using the **STM32 NUCLEO-L476RG** microcontroller. It interfaces with up to **five incremental optical encoders**, logs time-synchronized position data to an **SD card**, and optionally streams the data via **UART**.

Originally, the purpose of this project was to register the motion of a **human upper extremity**, based on a **5-degree-of-freedom limb model**, using a **passive exoskeleton** within which the encoders were mounted. However, it can be easily adapted to any **STM32-based kinematic system**.

You can read more about the passive exoskeleton upper extremity motion registration system in the following publication:  
🔗 [The Concept of the 5-DOF Passive Exoskeleton for Tracking the Motion of a Human Upper Extremity](https://link.springer.com/chapter/10.1007/978-3-031-78266-4_14)

The utilisation of a system is presented below:
![exo](https://github.com/user-attachments/assets/8eabff84-d843-4e11-be10-22da0b0d34c9)

---

## 📌 Main features

- ⏱️ Real-time encoder data acquisition (5 channels)
- 💾 SD card logging using FatFs (via SPI)
- 🔌 Optional UART output for live monitoring

---

## 🛠️ Hardware Requirements

- STM32 NUCLEO-L476RG board (or similar)
- Micro SD card module with reader module  (SPI interface) 
- 5x Incremental optical rotary encoders
- Push-button (for stopping data logging)
- UART-to-USB interface (optional, for PC communication)

---

## 📂 Project Structure

```bash
.
├── Core/
│   └── Src/
│       └── main.c         # Main application logic
│
├── Inc/
│   └── main.h             # Header declarations
├── README.md              # Project documentation
