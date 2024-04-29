# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       linetrack.py                                                 #
# 	Author:       gracem                                                       #
# 	Created:      4/3/2024, 10:04:19 AM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

# Brain should be defined by default
brain=Brain()

print("Line tracking...")

left_motor = Motor(Ports.PORT1, True)
right_motor = Motor(Ports.PORT10)
drive_motors = MotorGroup(left_motor, right_motor)
front_sonar = Sonar(brain.three_wire_port.e)
back_sonar = Sonar(brain.three_wire_port.a)
line_left = Line(brain.three_wire_port.g)
line_right = Line(brain.three_wire_port.h)


# dist (double) distance from wall when robot stops linetracking in centimeters 
def linetrack(dist, front):
    base_effort = 40  # 80 / 40
    KP = 0.03 # 0.02 / 0.01
    KD = 0.96 # 0.18 / 0.96
    
    sleep(2000, MSEC)

    error_prev = line_left.reflectivity() - line_right.reflectivity()
    if(front): 
        while front_sonar.distance(DistanceUnits.CM) > dist:
            error = (line_left.reflectivity() - line_right.reflectivity())
            
            left_motor.spin(FORWARD, base_effort-error*KP+KD*((error-error_prev)/20), RPM)
            right_motor.spin(FORWARD, base_effort+error*KP+KD*((error-error_prev)/20), RPM)

            error_prev = error
    else: 
        while back_sonar.distance(DistanceUnits.CM) < dist:
            error = (line_left.reflectivity() - line_right.reflectivity())
            
            left_motor.spin(FORWARD, base_effort-error*KP+KD*((error-error_prev)/20), RPM)
            right_motor.spin(FORWARD, base_effort+error*KP+KD*((error-error_prev)/20), RPM)

            error_prev = error

    drive_motors.spin(FORWARD,0)
    sleep(1000, MSEC)
