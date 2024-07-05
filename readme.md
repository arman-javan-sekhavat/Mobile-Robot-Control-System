This repository includes the code for mobile robot motion control.

The code is divided into two parts. One part should be programmed onto an ESP32 development board, and the other one should be compiled and used on a computer running Windows..

The control panel enables the user to specify (draw) an arbitrary and hypothetical path on the ground, by using mouse cursor, and control the robot such that it follows the specified path, automatically.




Different Layers of the System


---   Monitoring Layer ---
A Windows application plays the role of  a control panel. This application can be used to monitor the camera view, robot's position and orientation.


---   Image Processing Layer   ---
The raw images will be received from a camera. These images will be processed to determine the position and orientation of the robot.


---   Control Layer   ---
The user is able to draw the desired path of the robot in the control panel. We'll use a manually tuned PID as the control law. The control law will be implemented in the control panel. The control commands will be calculated based on the robot's position and orientation and sent to the robot over a Wi-Fi network by using the socket API. Note that the PID parameters must be tuned for each different surface if the slipping friction is present between the robot and the surface.