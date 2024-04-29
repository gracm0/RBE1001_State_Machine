# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       touch_fruit.py                                               #
# 	Author:       gracem                                                       #
# 	Created:      4/10/2024, 10:35:55 AM                                       #
# 	Description:  Emil finds a lime and touches it                             #
#                                                                              #
# ---------------------------------------------------------------------------- #

from vex import *
import math

brain=Brain()

left_motor = Motor(Ports.PORT1, True)
right_motor = Motor(Ports.PORT10)
drive_motors = MotorGroup(left_motor, right_motor)
arm_motor = Motor(Ports.PORT8)
front_sonar = Sonar(brain.three_wire_port.e)
back_sonar = Sonar(brain.three_wire_port.a)
line_left = Line(brain.three_wire_port.g)
line_right = Line(brain.three_wire_port.h)
imu = Inertial(Ports.PORT9)
bumper = Bumper(brain.three_wire_port.c)

Vision20_LIME = Signature(1, -5859, -4897, -5378, -3423, -2783, -3103, 3.0, 0)
Vision20_LEMON = Signature(2, 2349, 2731, 2540, -3683, -3351, -3517, 3.0, 0)
Vision20_GRAPEFRUIT = Signature(3, 6765, 7255, 7010, 1073, 1319, 1196, 3.0, 0)
Vision20_ORANG = Signature(4, 6285, 7233, 6759, -2305, -2069, -2187, 3.0, 0)
Vision20 = Vision(Ports.PORT20, 50, Vision20_LIME, Vision20_LEMON, Vision20_GRAPEFRUIT, Vision20_ORANG)

def turn():
    left_motor.spin(FORWARD, 20)
    right_motor.spin(FORWARD, -20)

# Calculates the height of the center of the object from the center of the camera in cm
def calcYHeight():
    FOCAL_LENGTH = 236.25
    height_actual = 8 # in cm
    h = Vision20.largest_object().height
    x = Vision20.largest_object().centerX - (316/2)
    y = (212/2) - Vision20.largest_object().centerY

    D = (FOCAL_LENGTH*height_actual)/h
    H = h/FOCAL_LENGTH # objects apparent height in image in px
    # Calculate distance from object ot the center of the camera in px:
    D_c = math.sqrt(x**2+y**2)
    # Calculate the height of the object from the center of the camera:
    H_c = (H*D_c)/D

    return H_c

# IMU ----------------------------------
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

# Drive staight ---------------------------------

# Change KP as needed

# dist [double] - distance from wall in centimeters
# front [boolean] - true if using front sonar
def drive_to(dist, front):
    calibrate()

    base_effort = 30
    KP = 0.5

    if(front): 
        while front_sonar.distance(DistanceUnits.CM) > dist:
            if not(wrap(imu.heading()) == 0):
                error = 0 - wrap(imu.heading())
                left_motor.spin(FORWARD, base_effort + error*KP)
                right_motor.spin(FORWARD, base_effort - error*KP)
            else:
                drive_motors.spin(FORWARD, base_effort)
        print(front_sonar.distance(DistanceUnits.CM))
    else: 
        while back_sonar.distance(DistanceUnits.CM) < dist:
            if not(wrap(imu.heading()) == 0):
                error = 0 - wrap(imu.heading())
                left_motor.spin(FORWARD, base_effort + error*KP)
                right_motor.spin(FORWARD, base_effort - error*KP)
            else:
                drive_motors.spin(FORWARD, base_effort)
        print(back_sonar.distance(DistanceUnits.CM))

    
    drive_motors.spin(FORWARD,0)
    sleep(1000, MSEC)


# State Machine -------------------------------
ROBOT_IDLE = 0
ROBOT_SEARCHING = 1
ROBOT_CENTERING_X = 2
ROBOT_CENTERING_Y = 3
ROBOT_APPROACHING = 4

state = ROBOT_SEARCHING
print("Searching...")

# Vision resolution: 316x212
x_target = 316/2 # 130
# y_target = 2 # cm, guess
y_target = 106
h_target = 180

K_x = 0.2
# K_y = 0.9
K_y = 0.3
K_h = 0.2

# Drive to tree
# drive_to(180, False)
# drive_to(93, True)

# while True:
#     print(back_sonar.distance(DistanceUnits.CM))

stop = False

while not(stop):
    objects = Vision20.take_snapshot(Vision20_LIME)

    if objects and Vision20.largest_object().height >= 40:
        x_c = Vision20.largest_object().centerX
        y_c = Vision20.largest_object().centerY

        brain.screen.clear_screen()
        brain.screen.print_at("X", x=x_c, y=y_c)

        if state == ROBOT_SEARCHING:
            drive_motors.stop()
            sleep(2000)
            state = ROBOT_CENTERING_X
            print("Centering x...")

        if state == ROBOT_CENTERING_X:
            error = x_c - x_target
            turn_effort = K_x * error
            # print("x=",x_c)

            left_motor.spin(FORWARD, turn_effort)
            right_motor.spin(REVERSE, turn_effort)

            if abs(error) < 5:
                drive_motors.stop()
                sleep(2000)
                state = ROBOT_CENTERING_Y
                print("Centering y...")
        
        if state == ROBOT_CENTERING_Y:
            # y = calcYHeight()
            # error = y - y_target
            # print(y, y_target)
            error = y_c - y_target 
            effort = K_y * error
            

            # FORWARD moves arm down
            arm_motor.spin(FORWARD, effort)

            if abs(error) < 10:
                arm_motor.stop()
                sleep(100)
                state = ROBOT_APPROACHING
                print("Approaching...")
        
        if state == ROBOT_APPROACHING:
            error = h_target - Vision20.largest_object().height
            drive_effort = K_h * error
            # print("h=",Vision20.largest_object().height)

            drive_motors.spin(FORWARD, drive_effort)

            if abs(error) < 5:
                drive_motors.spin_for(FORWARD, 0.3, TURNS)
                drive_motors.stop()
                state = ROBOT_IDLE
                print("Idling...")

        if state == ROBOT_IDLE:
            stop = True              
            
    else:
        if state == ROBOT_APPROACHING or state == ROBOT_CENTERING_X or state == ROBOT_CENTERING_Y:
            state = ROBOT_SEARCHING
            print("Searching...")
        if state == ROBOT_SEARCHING:
            turn()