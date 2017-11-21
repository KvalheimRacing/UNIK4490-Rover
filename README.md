# UNIK4490-Rover
<img src="https://github.com/KvalheimRacing/UNIK4490-Rover/blob/master/Rover_2/rover_pic.PNG" width="400" height="340">

> 4 by 4 rover project

This is a joint collaboration between two groups studying [*UNIK4490 - Control of manipulators and mobile robots*](http://www.uio.no/studier/emner/matnat/its/UNIK4490/index-eng.html) at the University of Oslo.
In this project we are implementing controllers for path planning and velocity control of each wheel.

##### Known hardware on the rover;
| Name | Description |
| ----- | ---- |
|Hsiang Neng - hn-gh12-1634t 200rpm 30:1 geared |motor with encoder|
|Intel Nuc |computer|
|Some unknown teensy version| small arduino board for reading encoder data|

The folder "Rover_1" contains all software for the rover without lidar, and the folder "Rover_2" contains the software for the rover with lidar


#### Other useful information (pending)
- Rover 2 and/or 3 has wifi - login through ssh rosuser@10.10.0.1
- There is a teensy 3.1 for reading encoder data
- The motor driver on each rover controls both wheel at one side at the same time (meaning you control either left or right wheels)
- when running driver_pid.py you need to take the usb plug in and out the first time
- [Course repo](https://github.uio.no/UNIK4490/rover_setup)
