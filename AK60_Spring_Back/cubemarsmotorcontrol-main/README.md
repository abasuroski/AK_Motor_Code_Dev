# CubeMars Motor Position Control

This project uses an STM32 to control a CubeMars motor over CAN. The goal was to get smooth position control where the motor can be moved by hand and will return to a set position without jitter or instability.

The control is a simple PD loop with some filtering on velocity and reduced stiffness near the target to keep it from twitching. It’s meant to feel responsive but not rigid.

## Hardware
- STM32 Nucleo-F446RE  
- Waveshare CAN Bus Shield  
- CubeMars AK-series motor
  - AK70-10 KV100 Tested
 
## Software
- STM32CubeMX used to create and edit initial .ioc file and generate project
- STM32CubeIDE used to edit main.c and flash to board

## Notes
CAN communication is handled through the Waveshare shield, with PB8/PB9 used for RX/TX. Basic filtering is applied to accept all messages.

Gains (Kp, Kd) and filtering parameters can be adjusted depending on the setup and how stiff or compliant you want the motor to feel.
