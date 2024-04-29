# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       gracem                                                       #
# 	Created:      4/10/2024, 10:35:55 AM                                       #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

# Brain should be defined by default
brain=Brain()

left_motor = Motor(Ports.PORT10)
right_motor = Motor(Ports.PORT1, True)
drivetrain = DriveTrain(left_motor, right_motor)
imu = Inertial(Ports.PORT3)
line_left = Line(brain.three_wire_port.c)
line_right = Line(brain.three_wire_port.d)
ultrasonic_back = Sonar(brain.three_wire_port.a)
ultrasonic_front = Sonar(brain.three_wire_port.e)
arm_motor_1 = Motor(Ports.PORT9)
arm_motor_2 = Motor(Ports.PORT2, True)
arm_motors = MotorGroup(arm_motor_1, arm_motor_2)
claw = Motor(Ports.PORT11)

Vision__LIME = Signature(1, -5859, -4897, -5378, -3423, -2783, -3103, 3.0, 0)
Vision__LEMON = Signature(2, 2349, 2731, 2540, -3683, -3351, -3517, 3.0, 0)
Vision__GRAPEFRUIT = Signature(3, 4969, 5253, 5111, 1151, 1561, 1356, 3.0, 0)
Vision__ORANG = Signature(4, 6481, 7393, 6937, -2397, -2133, -2265, 3.0, 0)

Vision_ORANG_BIN = Code(Vision__GRAPEFRUIT,Vision__ORANG)
Vision_LIME_BIN = Code(Vision__GRAPEFRUIT,Vision__LIME)
Vision_LEMON_BIN = Code(Vision__GRAPEFRUIT,Vision__LEMON)

Vision20 = Vision(Ports.PORT20, 50, Vision__LIME, Vision__LEMON, Vision__GRAPEFRUIT, Vision__ORANG, Vision_LEMON_BIN, Vision_LIME_BIN, Vision_ORANG_BIN)

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

def drive_to(dist, front):
    calibrate()

    base_effort = 30
    KP = 0.5

    if(front): 
        while ultrasonic_front.distance(DistanceUnits.CM) > dist:
            if not(wrap(imu.heading()) == 0):
                error = 0 - wrap(imu.heading())
                left_motor.spin(FORWARD, base_effort + error*KP)
                right_motor.spin(FORWARD, base_effort - error*KP)
            else:
                drivetrain.drive(FORWARD, base_effort)
        print(ultrasonic_front.distance(DistanceUnits.CM))
    else: 
        while ultrasonic_back.distance(DistanceUnits.CM) < dist:
            if not(wrap(imu.heading()) == 0):
                error = 0 - wrap(imu.heading())
                left_motor.spin(FORWARD, base_effort + error*KP)
                right_motor.spin(FORWARD, base_effort - error*KP)
            else:
                drivetrain.drive(FORWARD, base_effort)
        print(ultrasonic_back.distance(DistanceUnits.CM))

    drivetrain.stop()
    sleep(1000, MSEC)

# Assume Robot is at "home" (starting position) facing to the right (toward the boxes)
# fruit : Vision_LIME_BIN, Vision_LIME_LEMON, or Vision_LIME_ORANG
def findBasket(fruit):
    BOXES = [(60, False), (115, False), (60, True)]

    for x in BOXES:
        drive_to(x[0],x[1])
        imuEffort(90)
        # lower arm
        if(Vision20.take_snapshot(fruit)):
            return
        # raise arm 
        imuEffort(-90)

def dropOff():
    # raise arm
    drive_to(5, True)
    # open claw

def detect():
    print("Detecting...")
    x= True
    while x:
        objects_LIME = Vision20.take_snapshot(Vision__LIME)
        objects_LEMON = Vision20.take_snapshot(Vision__LEMON)
        objects_ORANG = Vision20.take_snapshot(Vision__ORANG)
        objects_GRAPEFRUIT = Vision20.take_snapshot(Vision__GRAPEFRUIT)

        objects_ORANG_BIN = Vision20.take_snapshot(Vision_ORANG_BIN)
        objects_LIME_BIN = Vision20.take_snapshot(Vision_LIME_BIN)
        objects_LEMON_BIN = Vision20.take_snapshot(Vision_LEMON_BIN)

        # if objects_LIME and Vision20.largest_object().height >= 40:
        #     print(" Lime detected")
        #     x=False
    
        # if objects_LEMON and Vision20.largest_object().height >= 40:
        #     print(" Lemon detected")
        #     x=False
        
        # if objects_ORANG and Vision20.largest_object().height >= 40:
        #     print(" Orange detected")
        #     x=False

        # if objects_GRAPEFRUIT and Vision20.largest_object().height >= 40:
        #     print(" Grapefruit detected")
        #     x=False

        if objects_ORANG_BIN:
            print(" Orange Bin detected")
            x=False
        
        if objects_LIME_BIN:
            print(" Lime Bin detected")
            x=False

        if objects_LEMON_BIN:
            print(" Lemon Bin detected")
            x=False
    
    
sleep(200)
detect()