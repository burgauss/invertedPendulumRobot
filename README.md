# invertedPendulumRobot

This document intends to provide a clear description of the architecture used for the programming and commissioning of the inverted pendulum robot. Furthermore, it aims to help new participants in the project to get rapidly familiarized with the structure used in Arduino and ROS (Robot Operating System) for recent changes and implementations.
For a better description of the control parameter computation and the modeling of the robot, the reader can refer to the Studienarbeit Building and Programming of an Inverted Pendulum Robot (Malani).
This document will be divided into five chapters; this is the first chapter which includes the purpose of this document and an overview of the following chapters. The second chapter will contain an overview of the main architecture used for the robot, presenting the reader with a basic overview of the software architecture and the hardware, this chapter is directly related to chapter 6 of (Malani). Chapter three will focus on the commissioning of the robot for the first time. For this specific section, the software MobaXterm (MobaXterm, 2008) was used to connect with the Raspberry Pi using SSH. 
Chapter four presents the Arduino program, the libraries, and the microcontroller’s logic.
The Publisher-Subscriber Design pattern of ROS (ros, 2021) will be explained in chapter five. Moreover, the motors libraries and the PID cascade controller will be covered during the development of this same chapter.
Finally, chapter six presents the final thoughts for this project in the form of a conclusion and recommendations for future work and participants.

The microcontroller and the publisher subscriber code can be found on GitHub; see Arduino and raspberryPi, respectively.
