#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import random



# ###############    CLASSES    ################    

# This object is the class of our robot 
    # this objects controls robots movement and measure
class Robot():

    # INIT
    def __init__ ( self ,  EV3Brick, wheelDiameter,   axleTrack, right_motor_port,    left_motor_port,    ultrasonic_sensor_port, touch_sensor_port,   color_sensor_port,  gyro_sensor_port):
        
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
    
        print(type(ultrasonic_sensor_port))
        if ultrasonic_sensor_port is not None:
            self.distance_sensor    = UltrasonicSensor(ultrasonic_sensor_port)
            self.speaker.beep()

        print(type(touch_sensor_port))
        if touch_sensor_port is not None:
            self.touch_sensor       = TouchSensor(touch_sensor_port)
            self.speaker.beep()
            
        print(type(color_sensor_port))
        if color_sensor_port is not None:
            self.color_sensor       = ColorSensor(color_sensor_port)
            self.speaker.beep()

        print(type(gyro_sensor_port))
        if gyro_sensor_port is not None:
            self.gyro_sensor        = GyroSensor(gyro_sensor_port)
            self.speaker.beep()

        print(type(ultrasonic_sensor_port))
        # if there is a gyro sensor reset the angle and give variable to correction
        if self.gyro_sensor is not None:
            self.reset_angle(0)
            self.correction = self.angle()*2

        self.notes_frequencies = [261,277,311,329,349,369,392,415,440,466,493]
        # STARTING SPEAKER
        self.speaker.set_volume(50, which='_all_')


        for i in self.notes_frequencies:
            self.speaker.beep(frequency=i, duration=15)

        self.speaker.set_volume(100, which='_all_')

        







        self.speaker.set_volume(100, which='_all_')




    
    # This method beeps the speaker
    def beep(self):
        self.speaker.beep()

    # This  method takes the robot forward by the given distance (mm)
    def forward(self, distance ,speed):
        if self.gyro_sensor is not None:
            self.motors_reset()
            while self.motors_distance() < distance:
                print(self.motors_distance())
                self.correction = self.angle()*2 
                self.motors.drive(speed, self.correction)
        else:
            self.motors.straight(distance)

    # This  method takes the robot backward by the given distance (mm)
    def backward(self, distance , speed):
        if self.gyro_sensor is not None:
            self.motors_reset()
            while abs(self.motors_distance()) < distance:
                print(self.motors_distance())
                self.correction = self.angle()*2 
                self.motors.drive(-speed, self.correction)
        else:
            self.motors.straight(-distance)

    # This method rotates the robot to the left by the given angle
    def turn_left(self, angle , speed):
        if self.gyro_sensor is not None:
            self.motors.stop()
            while abs(self.angle()) < angle:
                self.left_motor.run(speed)
                self.right_motor.run(-speed)
            self.right_motor.brake()
            self.left_motor.brake()
            print(self.angle())
            self.reset_angle(0)
            self.correction = self.angle()*2
        else:
            self.stop()
            self.motors.turn(angle)


    # This method rotates the robot to the right by the given angle
    def turn_right(self, angle , speed):
        if self.gyro_sensor is not None:
            self.motors.stop()
            while abs(self.angle()) < angle:
                self.left_motor.run(-speed)
                self.right_motor.run(speed)
            self.right_motor.brake()
            self.left_motor.brake()
            print(self.angle())
            self.reset_angle(0)
            self.correction = self.angle()*2

        else:
            self.motors.turn(-angle)

    # This method rotates the robot to the right by the given angle
    def stop(self, angle):
        if self.gyro_sensor is not None:
            self.correction = self.angle()*2
            self.left_motor.brake()
            self.right_motor.brake()
            self.motors.stop()
        else:
            self.left_motor.brake()
            self.right_motor.brake()

    # This method measure the distance and returns it
    def distance(self):
        if self.distance_sensor is not None:
            return self.distance_sensor.distance()
        else:
            return None
    
    # This method returns the measured motors distance
    def motors_distance(self):
        return self.motors.distance()
        
    # This method resets motors mesauring
    def motors_reset(self):
        return self.motors.reset()
        
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

    # This method return reflection of measured color sensor value
    def reflection(self):
        if self.color_sensor is not None:
            return self.color_sensor.reflection()
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
        elif self.gyro_sensor is None:
            return None

    # This method sets the rotation angle of the sensor to a desired value. We do it because if we dont we can't measure the speed of the
    def reset_angle(self, angle):
        if self.gyro_sensor is not None:
            self.gyro_sensor.reset_angle(angle)    

    # this method plays Dünyanın Sonuna Doğmuşum
    def dunyanın_sonuna_dogmusum(self , duration, volume):

        self.speaker.set_volume(volume,"_all_")
        # Do
        # self.speaker.beep(frequency=261, duration=duration)
        # # Do #        
        # self.speaker.beep(frequency=277, duration=duration)
        # # Re        
        # self.speaker.beep(frequency=293, duration=duration)
        # # Re #        
        # self.speaker.beep(frequency=311, duration=duration)
        # # Mi        
        # self.speaker.beep(frequency=329, duration=duration)
        # # Fa        
        # self.speaker.beep(frequency=349, duration=duration)
        # # Fa #        
        # self.speaker.beep(frequency=369, duration=duration)
        # # Sol        
        # self.speaker.beep(frequency=392, duration=duration)
        # # Sol #      
        # self.speaker.beep(frequency=415, duration=duration)
        # # La        
        # self.speaker.beep(frequency=440, duration=duration)
        # # La #      
        # self.speaker.beep(frequency=466, duration=duration)
        # # Si        
        # self.speaker.beep(frequency=493, duration=duration)

        self.speaker.beep(frequency=293, duration=duration)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=440, duration=duration)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=392, duration=duration/2)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=349, duration=duration)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=329, duration=duration/2)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=349, duration=duration)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=392, duration=duration)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=349, duration=duration/2)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=329, duration=duration)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=293, duration=duration/2)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=293, duration=duration)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=440, duration=duration)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=392, duration=duration/2)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=349, duration=duration)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=329, duration=duration/2)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=349, duration=duration)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=392, duration=duration*2)
        self.speaker.beep(0,duration)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=293, duration=duration)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=440, duration=duration)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=392, duration=duration/2)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=349, duration=duration)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=329, duration=duration/2)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=349, duration=duration)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=392, duration=duration)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=349, duration=duration/2)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=329, duration=duration)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=293, duration=duration/2)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=293, duration=duration)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=329, duration=duration/2)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=349, duration=duration/2)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=329, duration=duration/2)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=293, duration=duration/2)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=261, duration=duration)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=329, duration=duration)
        self.speaker.beep(0,duration/50)
        self.speaker.beep(frequency=293, duration=duration)
        self.speaker.beep(0,duration/50)
        self.speaker.set_volume(100,"_all_")



    # This method turns the robot into an obstacle avoiding robot.s
    def avoid_obstacle(self, distance, speed):
        if self.distance_sensor is not None:
            if self.distance() <= 15:
                self.speaker.set_volume(50, which='_all_')
                self.speaker.beep(frequency=200,duration=100)
                self.speaker.set_volume(100, which='_all_')
                self.forward(distance,speed)
            else:
                self.speaker.beep(frequency=440,duration=200)
                while self.distance() < 20:
                    self.turn_right(5)
        else:
            return None

    # This method turns the robot into an line follower robot
    def follow_line(self, distance, speed):
        if self.color_sensor is not None:
            self.motors_reset()
            while self.motors_distance() < distance:
                light_correction = (30 - self.reflection())*2
                self.motors.drive(speed, light_correction)
        else:
            return None


    
# ###############    OBJECTS    ################    

ev3 = EV3Brick()

right_motor_port        = Port.C
left_motor_port         = Port.D
ultrasonic_sensor_port  = None
touch_sensor_port       = None
color_sensor_port       = None
gyro_sensor_port        = Port.S4

robot = Robot(ev3, 55.5, 190, right_motor_port, left_motor_port, ultrasonic_sensor_port,  touch_sensor_port,    color_sensor_port,  gyro_sensor_port)


# ###############    MAIN    ################    


while True:
    robot.forward(100,100)