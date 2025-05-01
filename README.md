# Remote Surgical Robot – Graduation Project

This project is a **Remote Surgical Robot** developed as part of a graduation project. It demonstrates the integration of precise motion control and embedded systems for potential applications in remote medical procedures.

## 🔧 Project Overview

The system enables remote control of a robotic mechanism using stepper motors, designed for precision movements typical in surgical environments. It focuses on reliable, real-time motor control through embedded programming.

## 🧠 Key Features

- Real-time control of stepper motors for surgical-grade movement
- Remote manipulation capabilities
- Scalable design for multi-axis robotic arms
- Low-latency response suitable for medical use cases

## 🛠️ Hardware Used

- **STM32H750VBT6** development board (ARM Cortex-M7)
- **NEMA 17** and **NEMA 23** stepper motors
- **TB6600** stepper motor drivers
- Potentiometer for manual input (for calibration and control testing)

## 💻 Software & Tools

- **STM32CubeIDE** for embedded development (HAL-based)
- **STM32 HAL Drivers** for peripheral management (ADC, TIM, GPIO)
- C programming for real-time embedded control

## 📁 Project Structure
Graduation-Project/ ├── Core/ │ ├── Inc/ # Header files (e.g., main.h, motor_control.h) │ └── Src/ # Source code files (e.g., main.c, motor_control.c) ├── Drivers/ # STM32 HAL drivers and CMSIS files ├── .ioc # STM32CubeMX configuration file ├── .project # STM32CubeIDE project file ├── .cproject # CDT project settings ├── README.md # Project documentation (this file)

## 🚀 Getting Started

1. Clone the repository and open the project in **STM32CubeIDE**.
2. Connect the stepper motors and drivers according to your pin configuration.
3. Build and flash the firmware to the **STM32H750VBT6** board.
4. Use a potentiometer or serial input to control motor movement.

## ⚠️ Disclaimer

This is an academic project and is **not intended for real medical use**.  
Further research, validation, and medical certifications are required before any practical or clinical application.

