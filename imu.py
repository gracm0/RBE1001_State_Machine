# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       imu.py                                                       #
# 	Author:       etoma                                                        #
# 	Created:      4/3/2024, 10:34:15 AM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

# Brain should be defined by default
brain=Brain()

print("imu...")

left_motor = Motor(Ports.PORT10, True)
right_motor = Motor(Ports.PORT1)
drivetrain = DriveTrain(left_motor, right_motor)
imu = Inertial(Ports.PORT9)

def calibrate():
    imu.calibrate()
    while imu.is_calibrating():
        sleep(50, MSEC)


def wrap(heading_d):
    theta = heading_d
    max = 180
    min = -180
    while theta > max or theta < min:
        if theta > max:
            theta -= 360
        else:
            theta += 360
    return theta


def imuEffort(heading_d):
    new_heading = wrap(heading_d)
    kp = 0.5
    # kd = 0.5
    effort = 10
    if(new_heading > 0):
        while effort >= 1.0:
            errorCurrent = new_heading - wrap(imu.heading())
            effort = kp * errorCurrent
            left_motor.spin(FORWARD, effort, RPM)
            right_motor.spin(REVERSE, effort, RPM)
    if(new_heading <= 0):
        while effort >= 1.0:
            error = wrap(imu.heading()) - new_heading
            effort = kp * error
            left_motor.spin(REVERSE, effort, RPM)
            right_motor.spin(FORWARD, effort, RPM)

    sleep(2000, MSEC)


