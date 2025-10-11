# Mobile Robot Control System

This repository includes the code for mobile robot motion control.

The code is divided into two parts. One part should be programmed onto an ESP32 development board, and the other one should be compiled and used on a computer running Windows.

The control panel enables the user to specify (draw) an arbitrary and hypothetical path on the ground using a mouse cursor and control the robot so that it automatically follows the specified path.

### Different Layers of the System

1) Monitoring Layer: A Windows application plays the role of a control panel. This application can be used to monitor the camera view, robot's position, and orientation.

2) Image Processing Layer: Raw images will be received from a camera. These images will be processed to determine the position and orientation of the robot.

3) Control Layer: The user is able to draw the desired path of the robot in the control panel. A manually tuned PID will be used as the control law. The control law will be implemented in the control panel. The control commands will be calculated based on the robot's position and orientation and sent to the robot over a Wi-Fi network using the socket API. Note that the PID parameters must be tuned for each different surface if there is slipping friction between the robot and the surface.

## Physical Structure
<img width="800" height="600" alt="image" src="https://github.com/user-attachments/assets/6d792514-aa8e-497e-94f4-15177dfb4fbd" />

## Control Panel GUI
<img width="726" height="478" alt="image" src="https://github.com/user-attachments/assets/099573ca-25ed-456a-989a-12cb63efb884" />

## Performance Test
<img width="600" height="319" alt="image" src="https://github.com/user-attachments/assets/2d65d95b-c7c4-4d5e-a49c-4b89c44e50bf" />
