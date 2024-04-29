# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       etoma                                                        #
# 	Created:      4/22/2024, 10:55:27 AM                                       #
# 	Description:  Fully autonomous robot collects and deposits an orange,      #
#   lemon, and lime from a tree to their corresponding baskets.                #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Final project copy

# Library imports
from vex import *

# Brain should be defined by default
brain=Brain()

left_motor = Motor(Ports.PORT10, True)
right_motor = Motor(Ports.PORT1)
drivetrain = DriveTrain(left_motor, right_motor)
imu = Inertial(Ports.PORT3)
line_left = Line(brain.three_wire_port.c)
line_right = Line(brain.three_wire_port.d)
ultrasonic_back = Sonar(brain.three_wire_port.a)
ultrasonic_front = Sonar(brain.three_wire_port.e)
arm_motor_1 = Motor(Ports.PORT9, True)
arm_motor_2 = Motor(Ports.PORT2)
arm_motors = MotorGroup(arm_motor_1, arm_motor_2)
claw = Motor(Ports.PORT11, True)

# ------------------------------------------------------------------------------------------------------------------------

# Drives forward following line until given distance, using derivative control
# dist (double) : distance from wall when robot should stop linetracking, in centimeters
# front (boolean) : true if distance is measured from front bumper, false if measured from back bumper
def linetrack(dist, front):
    base_effort = 40 * 4
    KP = 0.03 * 4
    KD = 0.96 * 4
   
    sleep(1000, MSEC)

    error_prev = line_left.reflectivity() - line_right.reflectivity()
    if(front):
        while ultrasonic_front.distance(DistanceUnits.CM) > dist:
            error = (line_left.reflectivity() - line_right.reflectivity())
           
            left_motor.spin(FORWARD, base_effort-error*KP+KD*((error-error_prev)/20), RPM)
            right_motor.spin(FORWARD, base_effort+error*KP+KD*((error-error_prev)/20), RPM)

            error_prev = error
    else:
        while ultrasonic_back.distance(DistanceUnits.CM) < dist:
            error = (line_left.reflectivity() - line_right.reflectivity())
           
            left_motor.spin(FORWARD, base_effort-error*KP+KD*((error-error_prev)/20), RPM)
            right_motor.spin(FORWARD, base_effort+error*KP+KD*((error-error_prev)/20), RPM)

            error_prev = error
    
    left_motor.stop()
    right_motor.stop()

# Calibrates the IMU sensor, setting the current heading to 0 degrees
def calibrate():
    imu.calibrate()
    while imu.is_calibrating():
        sleep(50, MSEC)

# Wraps the given heading to an equal value between -180 degrees and 180 degrees
# heading_d (int) : the heading to be wrapped, in degrees
# RETURN theta (int) : the wrapped heading, in degrees
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

# Turns to the given heading, using proportional control
# heading_d (int) : the target heading to turn to, in degrees
def imuEffort(heading_d):
    new_heading = wrap(heading_d)
    kp = 1.5
    effort = 10
    if(new_heading > 0):
        while effort >= 1.0:
            errorCurrent = new_heading - wrap(imu.heading())
            effort = kp * errorCurrent
            left_motor.spin(FORWARD, effort, RPM)
            right_motor.spin(REVERSE, effort, RPM)
            if(abs(errorCurrent) < 3):
                 left_motor.stop()
                 right_motor.stop()
                 return
    if(new_heading <= 0):
        while effort >= 1.0:
            error = wrap(imu.heading()) - new_heading
            effort = kp * error
            left_motor.spin(REVERSE, effort, RPM)
            right_motor.spin(FORWARD, effort, RPM)
            if(abs(error) < 3):
                 left_motor.stop()
                 right_motor.stop()
                 return

    sleep(2000, MSEC)

# Drives in a straight line, using proportional control
# dist (double) : distance from wall when robot should stop driving, in centimeters
# front (boolean) : true if distance is measured from front bumper, false if measured from back bumper
def drive_to(dist, front):
    calibrate()

    base_effort = 60
    KP = 1.0

    if(front):
        while ultrasonic_front.distance(DistanceUnits.CM) > dist:
            if not(wrap(imu.heading()) == 0):
                error = 0 - wrap(imu.heading())
                left_motor.spin(FORWARD, base_effort + error*KP)
                right_motor.spin(FORWARD, base_effort - error*KP)
            else:
                drivetrain.drive(FORWARD, base_effort)
    else:
        while ultrasonic_back.distance(DistanceUnits.CM) < dist:
            if not(wrap(imu.heading()) == 0):
                error = 0 - wrap(imu.heading())
                left_motor.spin(FORWARD, base_effort + error*KP)
                right_motor.spin(FORWARD, base_effort - error*KP)
            else:
                drivetrain.drive(FORWARD, base_effort)

    drivetrain.stop()
    sleep(1000, MSEC)

# ---------------------------------------------------------------------------------------------------------------------------

# States
HOME = 0
ORANGE_POS = 1
LEMON_POS = 2
LIME_POS = 3
SEARCHING = 4
LIFT_ARM = 5
APPROACHING = 6
GRABBING = 7
DEPOSITING = 8
END = 9

# Starting conditions:
# Robot is at "home" - bottom left corner (next to the boxes), facing the closet
# Arm is at lowest position, without touching the rangefinder
# Claw is open to widest position
# Robot is on the line
class Finite_State_Machine():

    def __init__(self):
        self.current_state = HOME

        self.end_states = [END]

        self.orange_collected = False
        self.lemon_collected = False
        self.lime_collected = False

        self.target_height = 102.0
        self.target_center_x = 150.0
        self.target_center_y = 102.5

        self.kh = 1.0
        self.kx = 0.6
        self.k_arm = 0.8

        arm_motors.set_position(0, DEGREES)
        
        self.Vision_LIME = Signature(1, -5859, -4897, -5378, -3423, -2783, -3103, 3.0, 0)
        self.Vision_LEMON = Signature(2, 2349, 2731, 2540, -3683, -3351, -3517, 3.0, 0)
        self.Vision_GRAPEFRUIT = Signature(3, 4969, 5253, 5111, 1151, 1561, 1356, 3.0, 0)
        self.Vision_ORANG = Signature(4, 6481, 7393, 6937, -2397, -2133, -2265, 3.0, 0)
        self.lemon_basket = Code(self.Vision_LEMON, self.Vision_GRAPEFRUIT)
        self.orange_basket = Code(self.Vision_ORANG, self.Vision_GRAPEFRUIT)
        self.lime_basket = Code(self.Vision_LIME, self.Vision_GRAPEFRUIT)

        self.Vision = Vision(Ports.PORT20, 50, self.Vision_ORANG, self.Vision_LIME, self.Vision_LEMON, self.Vision_GRAPEFRUIT, self.lemon_basket, self.orange_basket, self.lime_basket)

        self.current_target_fruit = self.Vision_ORANG
        self.current_target_basket = self.orange_basket

        calibrate()
        self.task()

    # Assume Robot is at "home" (starting position) facing to the right (toward the boxes)
    # Drives to each of the three basket locations and checks the basket, stopping when it reaches the given fruit basket
    # fruit (Color Code): the color code for the target basket (lemon_basket, lime_basket, or orange_basket)
    def findBasket(self, fruit):
        BOXES = [(50, False), (105, False), (55, True)]

        for x in BOXES:
            drive_to(x[0],x[1])
            imuEffort(90)
            i = 0
            while i == 0:
                orange_bin_detected = self.Vision.take_snapshot(self.orange_basket)
                if(orange_bin_detected):
                     print("orange bin detected")
                     if(fruit == self.orange_basket):
                          return
                     else:
                          i = 1
                lemon_bin_detected = self.Vision.take_snapshot(self.lemon_basket)
                if(lemon_bin_detected):
                     print("lemon bin detected")
                     if(fruit == self.lemon_basket):
                          return
                     else:
                          i = 1
                lime_bin_detected = self.Vision.take_snapshot(self.lime_basket)
                if(lime_bin_detected):
                     print("lime bin detected")
                     if(fruit == self.lime_basket):
                          return
                     else:
                          i = 1
                print("Nothing detected")
                drivetrain.turn(RIGHT, 10, RPM)
            imuEffort(0)

    def task(self):
        while self.current_state not in self.end_states:
            self.on_event()

    def on_event(self):
        if(self.current_state == HOME):
                self.__HOME_handler()
        elif(self.current_state == ORANGE_POS):
                self.__ORANGE_POS_handler()
        elif(self.current_state == LEMON_POS):
                self.__LEMON_POS_handler()
        elif(self.current_state == LIME_POS):
                self.__LIME_POS_handler()
        elif(self.current_state == SEARCHING):
                self.__SEARCHING_handler()
        elif(self.current_state == LIFT_ARM):
                self.__LIFT_ARM_handler()
        elif(self.current_state == APPROACHING):
                self.__APPROACHING_handler()
        elif(self.current_state == GRABBING):
                self.__GRABBING_handler()
        elif(self.current_state == DEPOSITING):
                self.__DEPOSITING_handler()
    
    def __HOME_handler(self):
        print("home")
        if not self.orange_collected:
            self.current_state = ORANGE_POS
            self.current_target_fruit = self.Vision_ORANG
        elif not self.lemon_collected:
            self.current_state = LEMON_POS
            self.current_target_fruit = self.Vision_ORANG
        elif not self.lime_collected:
            self.current_state = LIME_POS
            self.current_target_fruit = self.Vision_ORANG
    
    # Drives to orange trees
    def __ORANGE_POS_handler(self):
        linetrack(70, False)
        self.current_state = SEARCHING

    # Drives to lemon trees
    def __LEMON_POS_handler(self):
        linetrack(100, False)
        self.current_state = SEARCHING

    # Drives to lime trees
    def __LIME_POS_handler(self):
        linetrack(150, False)
        self.current_state = SEARCHING

    # Turns clockwise until a target fruit object is detected
    def __SEARCHING_handler(self):
        print("searching")
        # Arm is lifted to a position where fruit from any level is visible
        arm_motors.spin_for(FORWARD, 400, DEGREES, 40, RPM)
        i = 0
        while i == 0:
            if(not self.Vision.installed()):
                 print("camera is not working")
            object_detected = self.Vision.take_snapshot(self.current_target_fruit)
            drivetrain.turn(RIGHT, 30, RPM)
            if(object_detected and self.Vision.largest_object().height >= 40):
                i = 1 
        drivetrain.stop()
        self.current_state = LIFT_ARM

    # Raises arm and centers y, using proportional control
    def __LIFT_ARM_handler(self):
        self.Vision.take_snapshot(self.current_target_fruit)
        center_y = self.Vision.largest_object().centerY
        error = self.target_center_y - center_y
        arm_motors.spin(FORWARD, self.k_arm * error, RPM)
        if(abs(error) < 3):
            arm_motors.stop()
            self.current_state = APPROACHING

    # Drives forward while centering x and reaching the target height of the object, using proportional control
    def __APPROACHING_handler(self):
        self.Vision.take_snapshot(self.current_target_fruit)
        center_x = self.Vision.largest_object().centerX
        height = self.Vision.largest_object().height
        error_height = self.target_height - height
        error_center_x = self.target_center_x - center_x
        left_motor.spin(FORWARD, (self.kh * error_height) - (self.kx * error_center_x), RPM)
        right_motor.spin(FORWARD, (self.kh * error_height) + (self.kx * error_center_x), RPM)
        if (abs(error_height) < 10):
            left_motor.stop()
            right_motor.stop()
            while(abs(error_center_x) > 35):
                 self.Vision.take_snapshot(self.current_target_fruit)
                 center_x = self.Vision.largest_object().centerX
                 error_center_x = self.target_center_x - center_x
                 print(error_center_x * self.kx)
                 drivetrain.turn(LEFT, error_center_x * self.kx, RPM)
            drivetrain.stop()
            print("driving forward")
            drivetrain.drive_for(FORWARD, 40, INCHES, 45, RPM)
            self.current_state = GRABBING

    # Closes the claw
    def __GRABBING_handler(self):
        while claw.torque() < 0.5:
            print(claw.torque())
            claw.spin(FORWARD, 3, RPM)
        arm_motors.spin_to_position(0, DEGREES, 40, RPM)
        self.current_state = DEPOSITING

    # Drives to the basket locations and releases the fruit at the corresponding fruit basket
    def __DEPOSITING_handler(self):
        # Faces the baskets and drives until the line infront of the baskets
        imuEffort(-179)
        while line_left.reflectivity() < 60:
            print(line_left.reflectivity())
            drivetrain.drive(FORWARD, 60, RPM)
        drivetrain.stop()

        calibrate()
        drivetrain.drive_for(REVERSE, 30, INCHES, 60, RPM)

        imuEffort(-90)
        self.findBasket(self.current_target_basket)
        # Drops the fruit safely
        arm_motors.spin_for(FORWARD, 100, DEGREES, 60, RPM)
        distance_to_drive = ultrasonic_front.distance(MM)
        drivetrain.drive_for(FORWARD, distance_to_drive * 5 - 150, MM, 60, RPM)  
        arm_motors.spin_for(REVERSE, 100, DEGREES, 30, RPM)
        claw.spin_for(REVERSE, 20, DEGREES, 10, RPM)
        # Returns to starting position
        arm_motors.spin_for(FORWARD, 100, DEGREES, 30, RPM)
        drivetrain.drive_for(REVERSE, 50, INCHES, 60, RPM)
        if(not self.orange_collected):
             self.orange_collected = True
             self.current_state = HOME
        elif(not self.lemon_collected):
             self.lemon_collected = True
             self.current_state = HOME
        elif(not self.lime_collected):
             self.lime_collected = True
             self.current_state = END
             return
        imuEffort(179)
        drive_to(12, True)
        calibrate()
        imuEffort(90)
        calibrate()
        self.current_state = END # This statement was added for the demo to only collect an orange
        
machine = Finite_State_Machine()