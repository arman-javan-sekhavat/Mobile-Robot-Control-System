# Mobile Robot Control System

This repository includes the code for mobile robot motion control.

The code is divided into two parts. One part should be programmed onto an ESP32 development board, and the other one should be compiled and used on a computer running Windows.

The control panel enables the user to specify (draw) an arbitrary and hypothetical path on the ground using a mouse cursor and control the robot so that it automatically follows the specified path.

## Different Layers of the System

1) Monitoring Layer: A Windows application plays the role of a control panel. This application can be used to monitor the camera view, robot's position, and orientation.

2) Image Processing Layer: Raw images will be received from a camera. These images will be processed to determine the position and orientation of the robot.

3) Control Layer: The user is able to draw the desired path of the robot in the control panel. A manually tuned PID will be used as the control law. The control law will be implemented in the control panel. The control commands will be calculated based on the robot's position and orientation and sent to the robot over a Wi-Fi network using the socket API. Note that the PID parameters must be tuned for each different surface if there is slipping friction between the robot and the surface.

## Physical Structure
<img width="800" height="600" alt="image" src="https://github.com/user-attachments/assets/6d792514-aa8e-497e-94f4-15177dfb4fbd" />

## Control Panel GUI
<img width="726" height="478" alt="image" src="https://github.com/user-attachments/assets/099573ca-25ed-456a-989a-12cb63efb884" />

## Performance Test
<img width="600" height="319" alt="image" src="https://github.com/user-attachments/assets/2d65d95b-c7c4-4d5e-a49c-4b89c44e50bf" />

## How to Use

1)	Install the DroidCam application both on a mobile phone and on a laptop.
2)	Activate the ‘Mobile Hotspot’ functionality of the mobile phone. Set the network name to “WLAN” and set the security option to “Open” (no password protection).
3)	Connect the laptop to the network named “WLAN”.
4)	Open the DroidCam application both in the mobile phone and in the laptop.
5)	On the laptop, choose the option “connect over WiFi (LAN)”.
6)	Select “Video“ and deselect “Audio” on the laptop.
7)	Enter the IP address and the port number displayed on the mobile phone into the corresponding fields of DroidCam installed on laptop.
8)	Press the “Start” button on the laptop.
9)	Make sure that you can receive the images from the mobile phone’s camera.
10)	On the laptop, run the command prompt, type “ipconfig” and press “Enter”.
11)	Copy the “IPv4 address” of the laptop to the “Mobile_Robot.ino” file as the value of the “HOST” macro. Don’t forget the double quotation marks (“) around the IP address.
<img width="570" height="128" alt="image" src="https://github.com/user-attachments/assets/ff6d042b-4f5d-4d2f-bbb2-36963de74ea0" />
<img width="508" height="152" alt="image" src="https://github.com/user-attachments/assets/3affcd68-c167-4d15-acc8-aa29cb84a69a" />

12)	Compile and upload the “Mobile_Robot.ino” sketch to the ESP32 board.
13)	Execute the “Control Panel”.
14)	Specify the camera index. Camera indices are non-negative integers, typically 0 or 1.
15)	Specify an exact rectangular area on the ground, by sticking pieces of tape on four corners of a rectangle.
16)	Measure the width and height of the rectangle and enter them in the Control Panel expressed in millimeters.
17)	Press “Show camera view”.
18)	Press “Mark the corners”.
19)	On the “Camera View” window, select the four corners of the rectangle, each by a single left click.
20)	Press “Hide camera view”
21)	Specify an appropriate scale.
22)	Click on “Show transformed view”.
23)	On the “Transformed View” window, double click on the robot’s current position. Then, draw an arbitrary path by moving the mouse and double click again to specify the end of the path.
24)	Enter the PID parameters and press “Apply”.
25)	Press “Start the server”.
26)	Wait until the server is started.
27)	Turn on the robot.
28)	Wait until the robot (TCP client) is connected to the laptop (TCP server).
29)	After a few seconds (e. g. 3 seconds), click on the “Move” button.
30)	The robot will automatically follow the drawn path, provided the PID parameters are tuned properly.

In order to repeat the test to tune the PID parameters, follow the steps below:

31)	Turn off the robot.
32)	Set the new PID parameters and press “Apply”.
33)	Again, on the “Transformed View” window, double click on the robot’s current position. Then, draw an arbitrary path by moving the mouse and double click again to specify the end of the path.
34)	Press “Reset”.
35)	Press “Start the server” and wait until the server is started.
36)	Turn on the robot and wait until the robot is connected to the Control Panel.
37)	Press “Move”.
38)	Repeat the steps 31 to 37.
