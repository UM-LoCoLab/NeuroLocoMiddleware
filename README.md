# NeuroLocoMiddleware
This is a middleware library for controlling wearable robots following the electrical design pattern of the Neurobionics and Locomotor control system laboratories at the University of Michigan. This pattern uses a raspberry pi microcontroller to measure and command an array of sensors and actuators using USB. The actuators include Dephy's actpack. Two other actuators are available through related libraries, but through a CAN bus. Sensors include the lord microstrain IMU and analog Futek force sensors as read through an ADS 1115.
The library also provides some convenient soft real-time loop and inter-controller data sharing functionality.
Development is still active as of 21 March 2022.
