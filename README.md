# ⚡ STM32 High-Precision Reaction Timer

My implementation of a digital reflex analyzer. Developed in bare-metal C using Interrupts and Hardware Timers on the Nucleo-32 platform.

![Status](https://img.shields.io/badge/Version-1.0-blue)
![Hardware](https://img.shields.io/badge/Hardware-STM32G031-blue)

## 🎯 Features
* **Microsecond Precision:** Uses Hardware Interrupts (EXTI) for latency-free input detection.
* **State Machine Architecture:** Robust FSM design (`IDLE`, `WAIT`, `MEASURE`, `RESULT`) to prevent invalid states.
* **Hardware PWM:** Implements a non-blocking "Breathing LED" effect using Timer 3 comparators
* **UART Telemetry:** Logs reaction data to Serial Console (115200 baud).

## 🛠️ Hardware Setup
| Component | Nucleo Pin | Notes |
| :--- | :--- | :--- |
| **Tactile Button** | `PA0` | Active High (Internal Pull-Down) |
| **Status LED** | `PC6` | PWM Driven (Timer 3 Channel 1) |
| **UART TX** | `PA2` | Serial Logging to PC |
| **UART RX** | `PA3` | Command Input (Optional) |

## 🚀 How to Run
1.  Clone this repository.
2.  Open **STM32CubeIDE**.
3.  `File > Open Projects from File System`.
4.  Flash to any Nucleo-32 board.
5.  Open Serial Monitor (115200/8/N/1).