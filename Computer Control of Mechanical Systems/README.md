## Computer Control of Mechanical Systems (ME461)

This course focused using microcontrollers ([TMS320F28379D](https://www.ti.com/product/TMS320F28379D)) to control dynamic mechanical systems. 

### Wall Following
This project used two distance sensors (placed at front and right of robot) to follow a wall.

The [eQEP](https://dev.ti.com/tirex/explore/node?node=A__AYuKeLONYZ4NJtiy3OplhQ__C28X-ACADEMY__1sbHxUB__LATEST) peripheral of the TI microcontroller was used to read motor encoders for feedback. CAN bus communication was also implemented to read data from the distance sensors.

Each wheel is speed controlled using a decoupled PI controller. The velocity of each wheel is determined using a simple linear approximation. This does produce a fairly noisy velocity approximation, but is good enough to control the wheels. Integration is approximated using the tustin rule.
The angle of the robot car is set by controlling the difference traveled between the left and right wheels with a 'turn' setpoint. This allows the car to maintain stable velocity while turning corners and adjusting the distance from the wall.

The wall following algorithm uses a proportional controller to maintain a set distance from the wall. When the robot loses the wall at the right, the proportional controller causes the robot to execute a right turn, returning to following the wall. If a wall is detected in front of the robot, the robot makes a left turn. Distance away from the wall is maintained while turning left to prevent the robot from crashing and to allow for following of more unusual turn geometries.

As a fun challenge, we attempted to have the robot make a left-turn only robot (looping turn when at a right corner). Corners are found by checking whether the measured distance from the right wall is extremely large. Once a right corner is detected, the robot enters a looping left turn of decreasing radius. When a wall  is detected at close range, the robot switches out of the looping turn to resume following the wall. This controller required fine tuning of the looping turn to prevent the robot from crashing into the front wall or fully losing the wall.

### Segbot
This project balances the robot car on two wheel.

Balancing requires the implemetation of SPI to read accelerometer data. A Kalman filter was also implemnted since the accelerometer data is very noisy.

More refined velocities were required compared to wall following, so the transfer function $\frac{100z-100}{z-0.6}$ (discretized version of $\frac{125s}{s+125}$) was used to determine motor velocity and rate of tilt. The tilt, rate of tilt, and motor velocities were combined into a full state feedback controller to balance the robot. This controller was able to balance the segbot, but did not control the robot's position or velocity, making it fairly easy force the robot into an unstable state. 

Adding velocity and steering controller with a PID controller was able to fix this issues and allow us to drive the segbot around (communicating through an ESP32 over wifi to a LabView VI). This controller is implemented similarly to the wall-following robot and sets desired wheel rotation differences and a refernce velocity.
