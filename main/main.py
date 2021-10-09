# ###############    IMPORTS    ################    
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile



# ###############    CLASSES    ################    

# This object is the class of our robot 
    # this objects controls robots movement and measure
class Robot():

    # INIT
    def __init__(self,  EV3Brick, wheelDiameter,   axleTrack, right_motor_port,    left_motor_port,    ultrasonic_sensor_port, touch_sensor_port,   color_sensor_port, gyro_sensor_port):
        
        ################    Objects    ################    
        self.ev3                = EV3Brick
        self.speaker            = ev3.speaker

        ################    Motors    ################    
        self.right_motor        = Motor(right_motor_port)
        self.left_motor         = Motor(left_motor_port)
        self.motors             = DriveBase(self.left_motor, self.right_motor, wheel_diameter = wheelDiameter, axle_track = axleTrack)

        ################    Sensors    ################    
        self.distance_sensor    = None
        self.touch_sensor       = None
        self.color_sensor       = None
        self.gyro_sensor        = None



        ################    Variables    ################    
        self.correction = 0

        ################    Set Up    ################    

        #Defining sensor if they have a port
        if ultrasonic_sensor_port is not None:
            self.distance_sensor    = UltrasonicSensor(ultrasonic_sensor_port)
            self.beep()

        if touch_sensor_port is not None:
            self.touch_sensor       = TouchSensor(touch_sensor_port)
            self.beep()
            

        if color_sensor_port is not None:
            self.color_sensor       = ColorSensor(color_sensor_port)
            self.beep()

        if gyro_sensor_port is not None:
            self.gyro_sensor        = GyroSensor(gyro_sensor_port)
            self.beep()

        # if there is a gyro sensor reset the angle and give variable to correction
        if self.gyro_sensor is not None:
            self.reset_angle(0)
            self.correction = (0 - self.angle())


    
    # This method beeps the speaker
    def beep(self):
        self.speaker.beep()

    # This  method takes the robot forward by the given distance (mm)
    def forward(self, distance):
        if self.gyro_sensor is not None:
            self.correction = (0 - self.angle())
            self.motors.drive(distance, self.correction)
        else:
            self.motors.straight(distance)

    # This  method takes the robot backward by the given distance (mm)
    def backward(self, distance):
        if self.gyro_sensor is not None:
            self.correction = (0 - self.angle())
            self.motors.drive(-distance, self.correction)
        else:
            self.motors.straight(-distance)

    # This method rotates the robot to the left by the given angle
    def turn_left(self, angle):
        if self.gyro_sensor is not None:
            self.stop()
            self.motors.turn(angle)
            self.reset_angle(0)
            self.correction = (0 - self.angle())
        else:
            self.motors.turn(angle)


    # This method rotates the robot to the right by the given angle
    def turn_right(self, angle):
        if self.gyro_sensor is not None:
            self.stop()
            self.motors.turn(-angle)
            self.reset_angle(0)
            self.correction = (0 - self.angle())
        else:
            self.motors.turn(-angle)

    # This method rotates the robot to the right by the given angle
    def stop(self, angle):
        if self.gyro_sensor is not None:
            self.correction = (0 - self.angle())
            self.motors.stop()

    # This method measure the distance and returns it
    def distance(self):
        if self.distance_sensor is not None:
            return self.distance_sensor.distance()
        else:
            return None
    
    # This method return if touch sensor is pressed
    def is_pressed(self):
        if self.touch_sensor is not None:
            return self.touch_sensor.pressed()
        else:
            return None
    
    # This method return colors of measured color sensor value
    def color(self):
        if self.color_sensor is not None:
            return self.color_sensor.color()
        else:
            return None

    # This method return ambient light intensity of measured color sensor value
    def ambient(self):
        if self.color_sensor is not None:
            return self.color_sensor.ambient()
        else:
            return None

    # This method return tuple of reflections for red, green, and blue light of measured color sensor value
    def rgb(self):
        if self.color_sensor is not None:
            return self.color_sensor.rgb()
        else:
            return None

    # This method return angular velocity of gyro sensor
    def speed(self):
        if self.gyro_sensor is not None:
            return self.gyro_sensor.speed()
        else:
            return None

    # This method return rotation angle of gyro sensor
    def angle(self):
        if self.gyro_sensor is not None:
            return self.gyro_sensor.angle()
        else:
            return None

    # This method sets the rotation angle of the sensor to a desired value. We do it because if we dont we can't measure the speed of the
    def reset_angle(self, angle):
        if self.gyro_sensor is not None:
            self.gyro_sensor.reset_angle(angle)        

    
# ###############    OBJECTS    ################    

ev3 = EV3Brick()

right_motor_port        = Port.A
left_motor_port         = Port.D
ultrasonic_sensor_port  = Port.S1
touch_sensor_port       = None
color_sensor_port       = None
gyro_sensor_port        = Port.S4

robot = Robot(ev3, 55.5, 104, right_motor_port, left_motor_port, ultrasonic_sensor_port,  touch_sensor_port,    color_sensor_port,  gyro_sensor_port)


# ###############    MAIN    ################    

# Obstacle avoiding robot
while True:
    while robot.distance() >= 10:
        robot.forward(100)
    robot.beep()
    robot.turn_right(90)






