# Q-Baller - A Spherical Wheeled Robot (Ballbot) Project
Latest Update: 2017/2/27

Greetings! This is Jiamin Wang, currently a MS student majored in Mechanical Engineering at University of Missouri - Columbia. My current research areas include control and robotics.The Q-Baller Project is my Master's Research Project.

Since 2005, there have been many spectacular Ballbot designs (https://en.wikipedia.org/wiki/Ballbot). While admiring the great works by the leading researchers, I attempt to develop a Ballbot which happens to be the proposed research topic by my graduate advisor. I started from design in fall 2015 (the start of my graduate study), and later accomplished the mechanical prototype manufacture, electronic system application, dynamic analysis, control study and the embedded system development.

# Special Features about Q-Baller
The Q-Baller uses 4 Omni-Wheel friction drive systems. DC Motor is used to cut down the prototype cost and to test the theoretical possibility of accomplishing a low cost Ballbot without using Servo Motors.

The structure design adopts sheetmetal working techniques, resulting in a low manufacture cost.

Dynamic modeling is thorough which includes the characteristics of the DC motors. The inputs of the system are now Voltage, instead of torque which can be easily controlled if servo motors are adopted.

A contiunous gain-scheduled controller is designed for Q-Baller, which leads to some satisfying results.

Future works will include a more user-friendly and powerful HMI, experimentation of the application of advanced control methods and artificial intelligience algorithms.

# Project Timeline
2015/09: Completion of the Mechanical Design;

2015/11: Completion of the Mechanical Structure Prototype;

2016/01: Completion of the Preliminary Electronic System Design;

2016/02: Completion of the Dynamic Modeling, Basic Control Study and Behavior Planning;

2016/04: Completion of the Preliminary Simulation HMI (Idea Presentation);

2016/06: Corrected previous mistakes made in modeling and established a more detailed dynamic model;

2016/07: Completion of the Preliminary Electronic System Prototype;

2016/08: "Dynamic Modeling and Simulation of Q-Baller - A Spherical Wheeled Robot" accepted by ASIAN MMS 2016 & CCMMS 2016;

2016/10: Completion of the Preliminary Embedded System;

2016/11: Preliminary Electronic System did not work well due to the complexity of the system (too much noise). Therefore, a new electronic system planing is carried out to design a integrated robotic circuitry board;

2016/12: Completion of the JMechW Robotic Board controller design;

2017/01: Completion of the Electronic System Update;

2017/02: Updated the Embedded System (from STM32F103ZET6 to STM32F407VET6) and Improvement of the theoretic Gain-Scheduled LQR Controller (Through Delaunay Trangulation and Energy Level Operating Point Distribution).

# Challenges
The challenges I am facing during the project includes:

1. The Q-Baller Project has no financial support. For this reason, I have cut down the material cost of prototyping (excluding manufacturing labor) to within 600 USD and handmade many of the components.
2. As the individual researcher for this project, I have to take care of everything in the Q-Baller development including aspects from ME, EE and CS. Engineering workload has been kind of a burden.
3. The research resources are limited - equipments, environments and technical helps.

However, through challenges one can improve to a higher level and become tougher in life. After all, the project is fun so there is nothing really to complain about, haha.:)

# Resource Info

Q-Baller Design & Prototype:    The design and prototype of Q-Baller. The currently shown prototypes cannot satisfy control requirement. A future prototype update will take place very soon. When the project is finished, the 3D model and engineering drawing will be uploaded.

Q-Baller Documentations:    All the publications and thesis about Q-Baller. The thesis is planned to be finished by the end of 2017/03.

Q-Baller Embedded System:     The embedded system is developed for STM32F407VET6 embedded controller (the JMechW Robotic Board). Schematics and IO usage information are included, but the detailed PCB design will not be offered. The embedded system code is developed based on the open source code by OpenEdv (http://www.openedv.com/). The system will be tested at the beginning of 2017/03.

File Inclusion Level: Main>
                         PROJECT>
                                 UTILITY>
                                         DATA;
                                         ALGORITHM>
                                                   SYSTEM

Q-Baller Matlab - Modeling and Basic Controller Simulation:     The modeling and controller designer are included. Note that the Dynamic  System codes are designed through Obj-Oriented programming, which are included in ~/QBallerDynamicSystem/CLASS folders.

Q-Baller Matlab - Simulation & Experiment HMI (Idea Presentation):    The previously mentioned HMI, which include the 3D animation, real-time simulation, noise condition and support command based control and joystick control (xbox or playstation joysticks). It works well for the old modeling but has not yet been updated for the new dynamic model. The update and improvement will be made in the future.

At the current stage many works are not yet finished. There are more to come in the future.

Thank you for your time!






