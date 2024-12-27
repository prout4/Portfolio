## Capstone Project

In this project I developed an autonomous forklift system for the transport of neuronal samples between a laboratory space and testing chambers. My main roles in this project were the development of structural components and the full design of all electronic subsystems. The system uses a set of 4 stepper motors to drive 2 vertical lead screws and 2 custom-designed telescopic forks. An addition 2 servo motors are used to open and close a semi-airtight enclosure (to minimize contamination of the neuronal samples).


https://github.com/user-attachments/assets/4a9908a7-11cc-4c60-a13d-33bdc0796760


This page will focus on the electronics and controls used. For further details about the mechanical subsystems see [this presentation](ME470FinPresentation.pptx).

The forklift is a modular system that mounts to an off-the-shelf AMR. A [Raspberry Pi Pico 2](https://www.raspberrypi.com/products/raspberry-pi-pico-2/) is used to control the forklift mechanism and interfaces with the AMR through a custom 2-wire interface (allowing for easy integration with the AMR). The Pico interfaces with time-of-flight (ToF) sensors, limit switches, and motor encoders to determine the pose of the forklift and to drive the motors. Each motor is driven by a [TMC2209](https://www.analog.com/en/products/tmc2209.html) stepper driver using the step-dir interface. This minimizes the number of connections required while still offering a high degree of control over each motor. All components are connected to a custom-designed PCB (shown below).

![PCB](/Capstone%20Project%20(In%20Progress)/PCB.png)

Motor control is fairly straightforward. Each motor is defined as a struct, with the necessary pins, timers, and position variables needed to drive them. Desired axis positions are set in a state machine, starting motion in the appropriate direction. Once the desired position is reached, the state machine moves to the next state. Since stepper motors are prone to skipping steps, feedback is added through the use of time-of-flight sensors and encoders. Skipped steps are detected by checking if the motor positions are out of sync with each other. This is then corrected by speeding up the required motor until it matches the proper position. In the case of extremely large errors, all motor motion is stopped while the signle motor is adjusted to the proper position.
