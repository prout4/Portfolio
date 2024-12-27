## Digital Control of Dynamic Systems (SE420)

### Furuta Pendulum Full State Feedback Control
The Furuta Pendulum is an underactuated 2 link pendulum system (unactuated elbow). The goal of this experiment was to develop a controller to swing the pendulum up and stabilize it using full state feedback.

![image](https://github.com/user-attachments/assets/28682fb7-8bf5-4b04-a549-4db7179732e3)

Pole placement was first used to stabilize the equations of motion, linearized about the point of full inversion. This yielded semi-stable results, with somewhat large oscillations to keep the pendulum stable. Adding friction compensation improved stability by reducing the torque required to make small corrections.

Swing-up control was also implemented using the energy of the system as the feedback variable. The energy of the system is highest when the pendulum is fully inverted, so a controller that attempts to always increase energy will attempt to swing the pendulum up to the highest point. Note that if the energy-based swing-up controller is allowed to run indefinitely, it will attempt to spin the pendulum at ever increasing speed since the kinetic energy at high-speed rotation is higher than the potential energy at full inversion. To prevent this, the swing-up controller is switched off when the pendulum reaches sufficient energy. The full-state feedback controller then attempts to balance the pendulum.


https://github.com/user-attachments/assets/497cc80f-8d7e-450f-ad3e-97ff22f502ac

