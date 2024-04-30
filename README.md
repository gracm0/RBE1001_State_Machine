# RBE1001_Autonomous_Robot
## Introduction
**A fully autonomous robot detects, collects, and deposits an "orange", "lemon", and "lime" from a tree to the corresponding baskets.** The robot utilizes a four-bar lift to reach the three different heights of tree branches and a claw to grab fruits of varying widths one at a time. The control system starts in a home state and moves through 3 position states, a searching state, an approaching state, an arm lifting state, and a grabbing state. Due to the time constraint, the state machine currently is only tested for an orange, however, the program in its current state can be edited to collect a lemon and lime as well.

![Image of four-bar lift robot with the claw and arm extended.](https://github.com/gracm0/RBE1001_State_Machine/blob/59e361ae41d5f30b18c9b773c61a22c0619ca55b/IMG_9547.heic)

## System Integration
![Image of the state machine.](https://github.com/gracm0/RBE1001_State_Machine/blob/d2f7c373572d2b9203ddd38780ed6e4fe6dd485d/Screenshot%202024-04-29%20204121.png)
The Vex sensors used are the V5 3-Wire UltraSonic Range Finder, V5 3-Wire Line Tracker, V5 Inertial Sensor, and V5 Vision Sensor. The state machine's subsystems include line following, IMU turning, camera vision, grabbing mechanism, driving with the IMU, and color detection. The line following, IMU turning, and driving with the IMU all employ proportional control with the line following using proportional-derivative control.

![Drawn image of the "orchard" the robot traverses. Three rows of three trees of limes, lemons, and oranges and a row of three baskets.](https://github.com/gracm0/RBE1001_State_Machine/blob/101687a723c7e7c70114739d6cec970e7f080621/Screenshot%202024-04-29%20200817.png)

The line following subsystem uses the line trackers, which straddle the tape on the field, and the ultrasonic rangefinders on the front and back bumper of the robot. Using the rangefinders, when the robot reaches the specified distance, the robot will stop line following and go into the Searching State.

The camera vision employs the Vision sensor with trained signatures of each of the four fruits: lemon (yellow), lime (green), orange (orange), and grapefruit (pink). The robot will start collecting the fruit when an object of one of the four color signatures is detected and the object's height is greater than 40 pixels.

The robot goes into the Lift Arm State, which uses the coordinates of the object, provided by the camera vision, to center the robot claw with the fruit on the y-axis. Once the difference between the center y-coordinate of the fruit and the target y-value reaches the specified tolerance of 3, the robot goes into the Approaching State.

The robot uses the IMU to face the baskets and drives forward until the line in front of the baskets is detected by the line trackers. The driving subsystem uses the IMU to maintain its current heading in order to keep the robot driving in a straight line without any environmental factor’s aid, like line following or wall following require.

Once the baskets are reached, the robot checks each basket at the three known basket locations by using the camera vision and color code detection to determine the basket type. If the basket type is the same as the object collected, the robot will deposit the fruit in that basket.

## System Testing
To test the robot's ability to complete the task, we ran three trials for each basket position, varying whether we were grabbing a large or a small fruit, for a total of nine final test trials. Eight of the nine final trials proved to be successful. The one trial that went wrong was trial 5, which occurred because the fruit slipped in the claw when it was picked, causing it to block the camera. However, the rest of the trials worked as expected and we did not run into this issue again. The fruit was held on tightly and, despite the box being in different positions, the robot was successfully able to deposit the orange into the orange box.
