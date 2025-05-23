# Below Ground Board

This Firmware allows a host computer to communicate with a custom "BelowGroundBoard" PCB designed for the WatDig design team at the University of Waterloo. The system controls and relays sensor data about the state of a Tunnel Boring Machine (TBM). A flow chart depicting the general design of the system can be found at the following link...
https://lucid.app/lucidchart/40cd09a3-0b17-4176-88fb-b93ab9d76a61/edit?viewport_loc=-2870%2C-2245%2C5084%2C2400%2C0_0&invitationId=inv_9890a6aa-6289-44ab-988a-534f96138113

### System Overview
This system consists of 9 sensors wired to directly to 9 twelve bit ADC channels on a STM32C071CBT6 microcontroller. 3 of the 9 sensors are connected to 3 linear actuators which meneuver the tail end of the tunnel boring machine. A singular drv8244-q1 h-bridge motor driver connected via SPI to the STM32 microcontroller is used to individually control the 3 linear actuators through the toggling of 3 triacs, allowing for bi-directional motor control with the help of a PID control algorithm. Additionally, two inertial measurement units (IMUs) are connected via I2C to the STM32 microcontroller. One of the IMUs is attached to the main body of the TBM, while the other IMU is attached to the tail end of the TBM. This allows the system to understand the position and trajectory of the TBM while it is underground. All this data is contained within a "register_database" in the STM32 microcontroller, which is essentially just a global array that the host computer can read and write to via the Modbus protocol. The only modbus functions supported in this system are reading multiple holding registers (function code 0x03) or writing multiple holding registers (function code 0x10). Issuing invalid Modbus commands such as writing to a read-only register or exceeding the acceptable value range of a register will return an exception code in accordance with the Modbus protocol. The following link outlines the registers which the user has access to in the system...
https://docs.google.com/spreadsheets/d/11n6w8ZuzljPktblNUjErZGDjPZ7gsNEzAxKXmISzQjk/edit?usp=sharing

![image](https://github.com/user-attachments/assets/973bcd67-53dd-4fc7-a3bc-2a119710f1c3)

![image](https://github.com/user-attachments/assets/1243ba3f-7a1d-4415-8fd8-ae47233f5fba)
