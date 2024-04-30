# RBE1001_State_Machine
## Introduction
**A fully autonomous robot detects, collects, and deposits an orange, lemon, and lime from a tree to the corresponding baskets.** The robot utilizes a four-bar lift to reach the three different heights of tree branches and a claw to grab fruits one at a time. The control system starts in a home state and moves through 3 position states, a searching state, an approaching state, an arm lifting state, and a grabbing state. Due to the time constraint, the state machine currently is only tested for an orange, however, the program in its current state can be edited to collect a lemon and lime as well. 

Sensors used include ultrasonic rangefinders, reflectance sensors, and an IMU. Subsystems include line tracking, wall following, IMU turning, and a grabbing mechanism.


