<img src="https://github.com/KvalheimRacing/UNIK4490-Rover/blob/master/Rover_2/rover_pic.PNG" width="500" height="440">

# UNIK4490-Rover
> A 4-by-4 rover project

This is a joint collaboration between two groups studying [*UNIK4490 - Control of manipulators and mobile robots*](http://www.uio.no/studier/emner/matnat/its/UNIK4490/index-eng.html) at the University of Oslo.
In this project we have implemented odometric localization, posture control and velocity control for a mobile robot.

![](https://github.com/KvalheimRacing/UNIK4490-Rover/blob/master/Rover_2/1m_test.gif)

You can read the project reports ![here(English - Group 2)](https://github.com/KvalheimRacing/UNIK4490-Rover/blob/master/Rover_2/Rover-Rapport.pdf) and ![here(Norwegian - Group 1).](https://github.com/KvalheimRacing/UNIK4490-Rover/blob/master/Rover_1/rapport/RobotOblig3.pdf)

The folder "Rover_1" contains all software for the rover without lidar, and the folder "Rover_2" contains the software for the rover with lidar

##### Some hardware on the rovers
| Name | Description |
| ----- | ---- |
|Hsiang Neng - hn-gh12-1634t 200rpm 30:1 geared |motor with encoder|
|Intel Nuc |computer|
|Teensy 3.1| small arduino board for reading encoder data|



#### Other useful information
- Rover 2 and 3 is set up with onboard wifi - login through ssh rosuser@10.10.0.1
- The motor driver on each rover controls both wheel at one side at the same time (meaning you control either left or right wheels)
- when running driver_pid.py you need to take the usb plug in and out the first time
- [Course repo](https://github.uio.no/UNIK4490/rover_setup)
