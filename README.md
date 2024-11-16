# Ford Super Duty 2024 CAN Bus Interface

## Overview

This project provides a CAN Bus interface for the 2024 Ford Super Duty, designed to integrate WLED on the ESP with the vehicle's CAN bus system. By using this software and an STM32F44 series processor on a Nucleo-32 board, digital signals from the Body Control Module (BCM) and Steering Column Control Module (SCCM) can be accessed and utilized to control external LED lighting, such as side running lights, hazard lights, reverse lights, and more. This approach eliminates the need to splice into the vehicle's wiring harness to extract analog signals, preserving the integrity of the factory wiring.

While the project is designed for the 2024 Ford Super Duty, it may also be compatible with other Ford trucks or model years. However, compatibility with other models or years is currently untested.

Please note that this is an initial development version, tailored to a specific use case and hardware setup. Further development is planned to make the solution more generalized and user-friendly.

---

## Hardware Requirements

The following hardware is required to implement this solution:

- **STM32F44RE Nucleo Development Board**  
  (STM32F44 series processor)  
  [Buy on Amazon](https://www.amazon.ca/gp/product/B01I8XLEM8)

- **OLED Display for Debugging**  
  (SSD1306, 128x64 resolution)  
  [Buy on Amazon](https://www.amazon.ca/GeeekPi-SSD1306-Display-Arduino-Raspberry/dp/B0833PF7ML)

- **Waveshare SN65HVD230 CAN Transceiver Board**  
  [Buy on Amazon](https://www.amazon.ca/dp/B0B5DTN62K)

- **OBD-II 16-Pin Male Connector to Open Pigtail**  
  [Buy on Amazon](https://www.amazon.ca/gp/product/B0CXHLG31Q)

This hardware configuration provides the minimum setup required for development and testing. Future iterations of the project may involve additional hardware changes or optimizations.

---

## Why STM32?

The STM32 platform was chosen for its ARM architecture, which is highly regarded in industrial applications. Compared to other options like the ESP32, the STM32 provides superior performance, reliability, and a rich ecosystem of tools and libraries tailored to automotive-grade applications. While other microcontrollers have their strengths, the STM32 offers key advantages for CAN Bus interfacing in automotive environments. 

This choice is not intended to disparage other microcontroller platforms but reflects the specific requirements of this project.

---

## Future MCU Support

Currently, this project is designed exclusively for STM32 microcontrollers. Future plans include porting the solution to ESP32 and other platforms once the codebase is fully developed and a pipeline is established for maintaining and updating multiple MCU versions. For now, the project is limited to the STM32 architecture.

---

## Important Disclaimer

This project is provided as-is, with no guarantees of support or functionality. It is currently under active development and has not undergone thorough testing or security validation. It is intended for use in specific scenarios, such as the developer's environment, and may not be suitable for other applications.

If you choose to use this code or adopt ideas from this project, **you are fully responsible for ensuring the safety and functionality** of your implementation in your own environment, vehicle, or device. Proper testing and adherence to all relevant safety and regulatory standards are required before deployment.

---

## Closing Notes

This project represents an initial step in leveraging modern microcontrollers and CAN Bus communication for innovative automotive applications. As development progresses, further improvements and broader compatibility are anticipated. For now, this solution is tailored to meet a specific use case but serves as a foundation for potential future expansions.
