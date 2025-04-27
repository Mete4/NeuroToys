# NeuroToys: Non-Invasive Brain-Computer Interface for Real-Time Robot Control

## Hardware Design and Development

This document outlines the hardware design and development process for the NeuroToys project. The hardware aspect of this project integrates a variety of components, including a microcontroller, EEG, and a custom chassis to achieve real-time and remote control based on recieved EEG signals. Factors such as power management, wireless communication stability, signal integrity, and mechanical robustness are prioritized, ensuring that the platform could perform consistently during extended operation (more than 6 hours of typical use). By combining off-the-shelf EEG technology with widely available microcontrollers and motors, the NeuroToys project demonstrates the potential for accessible brain-computer interfacing in systems across various domains, including robotics, recreation, and assistive technology.

This report details the motivation behind each hardware selection, circuit design, mechanical integration, and practical considerations that shaped the final system.

## Component Selection
The hardware design is primarily based on the following components:

#### ESP32 Microcontroller
* The ESP32 is a powerful microcontroller with built-in Bluetooth capabilities, making it ideal for our use case. It is functionally interchangable with the Arduino framework, which allowed us to quickly get started with the development process. It is used to process commands sent from our BCI and actuate the motors to drive or turn the car.

#### Muse 2 EEG Headset
* The Muse 2 is a non-invasive EEG that provides real-time brainwave data. It is used to capture and transmit brain signals (measured in mV via adjacent electrodes contacting the forehead) to the host computer for further processing. The Muse 2 is a consumer-grade device, making it accessible for our project.

#### L298N Motor Driver
* The L298N motor driver is a dual H-bridge module that allows independent control of two DC motors. The driver receives control signals from the ESP32 and powers the motors accordingly, enabling forward movement, reverse movement, and turns.

#### 18650 Li-Ion Batteries
* The 18650 Li-Ion Batteries provides a reliable, rechargeable power source for the car. The 2s configuration offers a nominal voltage of 7.4V, which is well-suited for both the ESP32 and the L298N motor driver. The batteries’ high energy density ensures that the system can operate for extended periods without frequent recharging, and is protected by a 32V 3A in-line fuse to prevent overcurrent.

#### DC Geared Motors
* The DC geared motors are responsible for driving the wheels of the robot. These motors are used in many similar products, and are ideal for operating at the 7.4V.

## Circuit Design
The 2S 18650 batteries provide a stable 7.4V power supply, which is distributed to the L298N motor driver and regulated for the ESP32. The ESP32 processes commands from the host computer as input signals, sending a 5V enable signal to the L298N. The L298N, in turn, drives each of the four DC motors in their respective directions for forward, reverse, and turning movements. 

<img src=./images/circuit_schematic.png alt="Alt Text" width="600" height=auto>

## Assembly
A custom acryllic chassis has been designed to house the electrical components of the car. Motors, component housing, and components are fastened on the chassis using standard screws and nuts.

### CAD
<img src=./images/CAD1.png alt="Alt Text" width="300" height=auto>
<img src=./images/CAD3.png alt="Alt Text" width="300" height=auto>

<img src=./images/CAD2.png alt="Alt Text" width="300" height=auto>

### Prototype
<img src=./images/prototype1.jpg alt="Alt Text" width="300" height=auto>

<img src=./images/prototype_box1.jpg alt="Alt Text" width="300" height=auto>


###