# NeuroToys: Non-Invasive Brain-Computer Interface for Real-Time Robot Control

## Hardware Design and Development

This document outlines the hardware design and development process for the NeuroToys project. The hardware aspect of this project integrates a variety of components, including a microcontroller, EEG, and a custom chassis to achieve real-time and remote control based on recieved EEG signals. Factors such as power management, wireless communication stability, signal integrity, and mechanical robustness are prioritized, ensuring that the platform could perform consistently during extended operation (more than 6 hours of typical use). By combining off-the-shelf EEG technology with widely available microcontrollers and motors, the NeuroToys project demonstrates the potential for accessible brain-computer interfacing in systems across various domains, including robotics, recreation, and assistive technology.

The hardware deliverables of this project include:
* An RC car platform that can be controlled via Bluetooth
* An EEG headset that can transmit brain signals to a computer, used to control the car
* A custom chassis and enclosure designed to house the electrical components of the car
* A custom circuit design that integrates the microcontroller, motor driver, and power management components

This report details the motivation behind hardware component selection, circuit design, mechanical integration, and practical considerations that shaped the final system.

## Component Selection
The hardware design is primarily based on the following components:

#### ESP32 Microcontroller
The ESP32 is a powerful microcontroller with built-in Bluetooth capabilities, making it ideal for our use case. It is functionally interchangable with the Arduino framework, which allowed us to quickly get started with the development process. It is used to process commands sent from our BCI and actuate the motors to drive or turn the car.
The ESP32 also features a wide range of GPIO (general purpose input/output) pins, enabling simultaneous handling of wireless communication and motor control tasks without performance bottlenecks. Its support for PWM (Pulse Width Modulation) output was considered for fine-tuning motor speed according to focus levels, but was determined to be unnecessary for our application.
In addition, the ESP32's integrated ADC (Analog to Digital Converter) modules offer flexibility for potential future upgrades, such as incorporating additional analog sensors or feedback systems. Power consumption was another consideration — the ESP32 can operate in low-power modes if needed, which could help extend system runtime if motor usage becomes more intermittent in future versions of the project.

#### Muse 2 EEG Headset
The Muse 2 is a non-invasive EEG that provides real-time brainwave data. It is used to capture and transmit brain signals (measured in µV via adjacent electrodes contacting the forehead) to the host computer for further processing. The Muse 2 is a consumer-grade device, making it accessible for our project. The Muse 2 communicates via Bluetooth Low Energy (BLE), which offers low-latency, low-power transmission, which is ideal for real-time signal acquisition without significantly impacting battery life.

#### L298N Motor Driver
The L298N motor driver is a dual H-bridge module that allows independent control of two DC motors. The driver receives control signals from the ESP32 and powers the motors accordingly, enabling forward movement, reverse movement, and turns. Its capability to handle motor supply voltages up to 46V and continuous current of up to 2A per channel provides a large safety margin for our 7.4V DC motor configuration. Built-in heatsinks on the module helps maintain stable operation under sustained loads.

#### 18650 Li-Ion Batteries
The 18650 Li-Ion Batteries provides a reliable, rechargeable power source for the car. The 2s configuration offers a nominal voltage of 7.4V, which is well-suited for both the ESP32 and the L298N motor driver. The batteries’ high energy density ensures that the system can operate for extended periods without frequent recharging, and is protected by a 32V 3A in-line fuse to prevent overcurrent. In addition to fusing, we implemented a battery holder with integrated short-circuit protection to further safeguard the system. The battery pack can easily be swapped out, supporting quick turnaround during prolonged testing sessions.

#### DC Geared Motors
The DC geared motors are responsible for driving the wheels of the robot. These motors are used in many similar products, and are ideal for operating at the 7.4V. 
The built-in gear reduction provides high torque at relatively low RPMs, which is critical for smooth, controllable movement.The motors were selected to ensure sufficient torque to handle the additional weight of the chassis, batteries, and electronics while maintaining energy efficiency. Toy car rubber tires are inserted onto the DC motor shaft.

## Circuit Design
The 2S 18650 batteries provide a stable 7.4V power supply, which is distributed to the L298N motor driver and regulated for the ESP32. The ESP32 processes commands from the host computer as input signals, sending a 5V enable signal to the L298N. The L298N, in turn, drives each of the four DC motors in their respective directions for forward, reverse, and turning movements. A 32V 3A in-line fuse is used to protect the circuit from overcurrent, and a 5V voltage regulator is used to power the ESP32. The circuit is designed to be compact and efficient, ensuring that all components fit within the custom chassis.

<img src=./images/circuit_schematic.png alt="Alt Text" width="600" height=auto>

## Processing Pipeline
There are three core components to our processing pipeline: the Muse 2 EEG headpiece, a
computer which runs the Python interface, and the ESP32 located on the RC car. The headpiece transmits
raw brain voltage data (µV). The Python interface processes this signal by performing a Fourier
Transform on the AF7 electrode (on the left side of the forehead – chosen arbitrarily, as they both
pick up the same electrical activity of the prefrontal cortex) to isolate the beta frequency band
from the EEG data, which is associated with focus. The beta power is then calculated (expressed
in µV²), representing the user's focus level. A threshold is established to determine whether a
command should be sent to the ESP32, which then controls the forward movement of the toy car.

A similar signal processing technique is used for right and left control, where the AF7 and AF8
electrodes on both sides of the forehead go through a band pass filter separately to isolate the
gamma frequency band from the raw EEG data. Gamma is a lower frequency range (0.5 - 5 Hz),
allowing for isolation of corneoretinal motion (eye-blink) detection instead of fast neural activity.
Peak detection algorithms are then used to classify between right and left eye blinks (AF8 vs.
AF7 peaks), which in turn send the right and left turn commands to the car.

We have experimented with implementing a reverse mode, toggling forward
and backwards at threshold concentration levels via blinking both eyes simultaneously. However,
our tests have determined that this feature makes controlling the device substantially more
difficult than necessary, and we have opted to remove it from the final version of the project.

## Assembly
A custom acryllic chassis has been designed to house the electrical components of the car. Motors, component housing, and components are fastened on the chassis using standard screws and nuts. Acryllic was chosen for the enclosure as it is compact and lightweight, ensuring that the car can operate smoothly. We chose a poplar wood chassis as it is simple to work with and provides a sturdy base for the components. The chassis is designed to be modular, allowing for easy access to components for any maintenance that may be required.

### CAD
<img src=./images/CAD1.png alt="Alt Text" width="300" height=auto>
<img src=./images/CAD3.png alt="Alt Text" width="300" height=auto>

<img src=./images/CAD2.png alt="Alt Text" width="300" height=auto>

### Prototype
<img src=./images/prototype1.jpg alt="Alt Text" width="300" height=auto>

<img src=./images/prototype_box1.jpg alt="Alt Text" width="300" height=auto>

### Final Assembly
<img src=./images/assembly1.png alt="Alt Text" width="500" height=auto>
