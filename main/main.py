################    IMPORTS    ################    
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile



################    CLASSES    ################    

# This object is the class of our robot 
    # this objects controls robots movement and measure
class Robot():

    # INIT
    def __init__(self,EV3Brick,wheelDiameter,axleTrack,right_motor_port,left_motor_port,ultrasonic_sensor_port,touch_sensor_port,color_sensor_port,gyro_sensor_port):
        
        #OBJECTS
        self.ev3                = EV3Brick
        self.speaker            = ev3.speaker

        #MOTORS
        self.right_motor        = Motor(right_motor_port)
        self.left_motor         = Motor(left_motor_port)
        self.motors             = DriveBase(self.left_motor, self.right_motor, wheel_diameter=wheelDiameter, axle_track=axleTrack)

        #SENSORS
        self.distance_sensor    = UltrasonicSensor(ultrasonic_sensor_port)
        self.touch_sensor       = TouchSensor(touch_sensor_port)
        self.color_sensor       = ColorSensor(color_sensor_port)
        self.gyro_sensor        = GyroSensor(gyro_sensor_port)

    
    # This method beeps the speaker
    def beep(self):
        self.speaker.beep()

    # This  method takes the robot forward by the given distance (mm)
    def forward(self,distance):
        self.motors.straight(distance)

    # This  method takes the robot backward by the given distance (mm)
    def backward(self,distance):
        self.motors.straight(-distance)

    # This method rotates the robot to the left by the given angle
    def turn_left(self,angle):
        self.motors.turn(angle)

    # This method rotates the robot to the right by the given angle
    def turn_right(self,angle):
        self.motors.turn(-angle)

    # This method measure the distance and returns it
    def distance(self):
        return self.distance_sensor.distance()
    
    # This method return if touch sensor is pressed
    def is_pressed(self):
        return self.touch_sensor.is_pressed
    
    # This method return colors of measured color sensor value
    def color(self):
        return self.color_sensor.color()

    # This method return ambient light intensity of measured color sensor value
    def ambient(self):
        return self.color_sensor.ambient()

    # This method return tuple of reflections for red, green, and blue light of measured color sensor value
    def rgb(self):
        return self.color_sensor.rgb()

    # This method return angular velocity of gyro sensor
    def speed(self):
        return self.gyro_sensor.speed()

    # This method return rotation angle of gyro sensor
    def angle(self):
        return self.gyro_sensor.angle()

    # This method sets the rotation angle of the sensor to a desired value. We do it because if we dont we can't measure the speed of the
    def reset_angle():
        self.gyro_sensor.reset_angle()
        


    
################    OBJECTS    ################    

ev3 = EV3Brick()

right_motor_port        = Port.A
left_motor_port         = Port.B
ultrasonic_sensor_port  = Port.S1
touch_sensor_port       = Port.S2
color_sensor_port       = Port.S3
gyro_sensor_port        = Port.S4

robot = Robot(ev3, 55.5, 104, right_motor_port, left_motor_port, ultrasonic_sensor_port,  touch_sensor_port,    color_sensor_port,  gyro_sensor_port)


################    MAIN    ################    

robot.beep()



