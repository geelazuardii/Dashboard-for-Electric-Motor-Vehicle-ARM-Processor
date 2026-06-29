# Centralized-Dashboard-for-Electric-Motor-Vehicle-using-ARM-Processor
Centralized Dashboard System for Electric Motor Vehicle using STM32F407VG Discovery, TouchGFX, GPS, JK BMS, and Fardriver Controller integration

# Centralized Dashboard for Electric Motor Vehicle using ARM Cortex-M4

<p align="center">
<img src="docs/images/dashboard_cover.png" width="900">
</p>

<p align="center">

![STM32](https://img.shields.io/badge/MCU-STM32F407VG-green)
![TouchGFX](https://img.shields.io/badge/UI-TouchGFX-blue)
![ILI9488](https://img.shields.io/badge/Display-ILI9488-orange)
![GPS](https://img.shields.io/badge/GPS-NEOm6n-yellow)
![JKBMS](https://img.shields.io/badge/BMS-JK_BMS-red)
![License](https://img.shields.io/badge/License-MIT-brightgreen)

</p>

---

## Overview

This project presents a centralized digital dashboard system developed for an electric motor vehicle using the **STM32F407VG Discovery** board as the main processing unit. The dashboard integrates multiple vehicle subsystems including the Fardriver motor controller, JK Battery Management System (JK-BMS), GPS module, SD Card data logger, and a 3.5-inch TFT display driven by TouchGFX.

The objective of this project is to provide a single human-machine interface capable of displaying real-time vehicle information while maintaining reliable communication with various electronic modules.

---

## Dashboard Preview

<p align="center">
<img src="docs/images/dashboard_ui.png" width="800">
</p>

---

# Main Features

✔ Real-time vehicle speed display

✔ Battery State of Charge (SOC)

✔ Battery voltage and current monitoring

✔ Motor RPM monitoring

✔ Motor controller status

✔ GPS location and speed

✔ Turn signal indicators

✔ High beam indicator

✔ Reverse indicator

✔ SD Card data logging

✔ TouchGFX graphical user interface

✔ Interrupt-based SIF protocol parser

✔ UART communication with external modules

---

# System Architecture

<p align="center">
<img src="docs/architecture/system_architecture.png" width="900">
</p>

The dashboard receives vehicle information from several independent subsystems. Data are processed by the STM32F407 microcontroller before being synchronized and displayed through the graphical user interface developed using TouchGFX.

---

# Hardware Specification

| Device | Description |
|----------|----------------|
| MCU | STM32F407VG Discovery |
| Display | 3.5" TFT ILI9488 SPI |
| Motor Controller | Fardriver ND Series |
| Battery Management | JK BMS |
| GPS Module | NEO M6N |
| Storage | Micro SD Card |
| Development IDE | STM32CubeIDE |
| GUI Framework | TouchGFX |

---

# Software Architecture

```
main()

│

├── System Initialization

│

├── Peripheral Initialization

│     ├── GPIO

│     ├── SPI

│     ├── UART

│     ├── FATFS

│     └── TouchGFX

│

├── Main Loop

│     ├── Process SIF Packet

│     ├── Update Vehicle Data

│     ├── Read JK BMS

│     ├── Parse GPS

│     ├── Log SD Card

│     └── Update GUI

```

---

# Repository Structure

```
Core/
Drivers/
FATFS/
Middlewares/
TouchGFX/
docs/
README.md
LICENSE
```

---

# Communication Interfaces

| Interface | Purpose |
|-----------|-------------|
| SPI | TFT Display |
| SPI | Touch Controller |
| UART2 | GPS Module |
| UART3 | JK BMS |
| GPIO Interrupt | Fardriver SIF |
| SPI | SD Card |

---

# Vehicle Data

The dashboard displays the following information:

- Vehicle Speed
- Motor RPM
- Battery SOC
- Battery Voltage
- Battery Current
- Motor Temperature
- GPS Coordinates
- GPS Speed
- High Beam Status
- Left Turn Signal
- Right Turn Signal
- Hazard Indicator
- Reverse Mode
- Controller Fault

---

# Data Processing Flow

<p align="center">
<img src="docs/flowchart/data_processing.png" width="850">
</p>

---

# Testing

The following tests were conducted during system validation.

| Test | Result |
|--------|------------|
| GUI Initialization | Pass |
| GPS Communication | Pass |
| JK BMS Communication | Pass |
| SIF Data Parsing | Pass |
| SD Card Logging | Pass |
| RPM Verification | Pass |
| Processing Time Measurement | Pass |

---

# Gallery

### Dashboard

<img src="docs/images/dashboard1.jpg">

---

### Wiring Diagram

<img src="docs/hardware/wiring.png">

---

### Hardware Prototype

<img src="docs/images/prototype.jpg">

---

### TouchGFX Interface

<img src="docs/images/touchgfx.jpg">

---

### GPS Logger

<img src="docs/images/gps_logger.jpg">

---

# Future Development

- CAN Bus Integration
- Bluetooth Telemetry
- OTA Firmware Update
- Cloud Data Synchronization
- VESC Communication
- Battery Health Prediction

---

# Author

Muhammad Galan Lazuardi Saka

Department of Informatics and Commputer Engineering

Politeknik Elektronika Negeri Surabaya (PENS)

---

# License

This project is released under the MIT License.
## Features

- Vehicle Speed
- Battery SOC
- RPM Monitoring
- GPS
- SD Card Logging
- JK BMS Communication
- TouchGFX GUI
- SIF Fardriver Protocol

Author:
Muhammad Galan Lazuardi Saka
