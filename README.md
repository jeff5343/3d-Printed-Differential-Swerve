# 3D Printed Differential Swerve
A compact differential swerve drive system.

## Project Overview
The goal of this project was to create a swerve bot from scratch. After iterating through several prototypes, I adapted and optimized [WildWilly’s differential swerve design](https://www.printables.com/model/300105-differential-swerve-drive), custom-drawing a scaled-down version.

* **PID Control:** `MiniPID` to handle motor velocity and heading.
* **Hardware Interfacing:** `pigpio` for PWM control on the Raspberry Pi.
* **Input Handling:** `libevdev` to process Bluetooth events from an Xbox controller.
* **Concurrency:** multithreading to process encoder ticks and controller inputs.

## Design & CAD
* **Module Dimensions:** 4" x 3.5" x 2.5"
* **Total Chassis:** 8.25" x 8.25"

<img width="600" alt="Diff-Swerve" src="https://github.com/user-attachments/assets/cfb582d8-75ce-4cb2-9ba4-20033a245fbc">
<br>
<img width="470" alt="Diff-Swerve-Module" src="https://github.com/user-attachments/assets/832c4338-be59-4003-98e9-b717c2810e5f">
