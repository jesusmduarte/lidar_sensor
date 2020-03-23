"""lidar_read_sensor controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Lidar
from controller import LidarPoint
from controller import Motor
from controller import PositionSensor
from controller import Keyboard
from time import sleep
import os

import math
import numpy as np


wheel_radius = .195 #m
velocity = 1 #m/s


global status
status = "stop mode"
# create the Robot instance.
robot = Robot()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
#motors
leftMotor = robot.getMotor('left wheel')
rightMotor = robot.getMotor('right wheel')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
#keyboard
keyboard = Keyboard() #it could be keyboard = Keyboard()
keyboard.LEFT = 314
keyboard.UP = 315
keyboard.RIGHT = 316
keyboard.DOWN = 317
keyboard.A = 65
keyboard.M = 77
keyboard.I = 73
keyboard.L = 76
keyboard.Q = 81
keyboard.enable(timestep)
#Lidar
Lidar1 = robot.getLidar('Hokuyo URG-04LX')
Lidar1.enable(timestep)
#Lidar1.enablePointCloud()

#functions

def ascii_to_char(ascii_value):
    if ascii_value !=-1 and ascii_value < 256:
        return chr(ascii_value)

def lineartoangular(velocity):
    angular_velocity = velocity / wheel_radius
    res = angular_velocity
    return res

def up(velocity):
    leftMotor.setVelocity(lineartoangular(velocity))
    rightMotor.setVelocity(lineartoangular(velocity))
def down(velocity):
    leftMotor.setVelocity(lineartoangular(-velocity))
    rightMotor.setVelocity(lineartoangular(-velocity))
def left(velocity):
    leftMotor.setVelocity(lineartoangular(-velocity))
    rightMotor.setVelocity(0)
def right(velocity):
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(lineartoangular(-velocity))
def stop(velocity):
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)

def scan_lidar():
    Lidar1.enablePointCloud()
    #Lidar_range = Lidar1.getRangeImage()
    #print ("lidar distances in m:",Lidar_range[0:5])
    Points = Lidar1.getPointCloud()
    Points_Propierties = LidarPoint()
    if isinstance(Points, list):
        print("your object is a list !")
    else:
        print("your object is not a list")
    NumberofPoints = Lidar1.getNumberOfPoints()
    print("number of points:",NumberofPoints)
    Lidar_fov = Lidar1.getFov()
    print("lidar FOv:",Lidar_fov)
    Lidar_frecuency = Lidar1.getFrequency()
    print("the frecuency is:",Lidar_frecuency)
    Lidar_h_r = Lidar1.getHorizontalResolution()
    print("the getHorizontalResolution is:", Lidar_h_r)

def clear_screen():
    Lidar1.disablePointCloud()
    os.popen('sh /home/jesus/Documents/webots/pioner3/controllers/clean_function.sh')

def stop_mode(ascii_value):
    global status
    #print("stop mode")
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)

    if ascii_value== keyboard.A:
        status = keyboard.A
    if ascii_value== keyboard.M:
        status = keyboard.M



#modes of operation
def manual_mode(ascii_value):
    global status
    #moves
    if ascii_value== keyboard.UP:
        up(velocity)
    elif ascii_value== keyboard.DOWN:
        down(velocity)
    elif ascii_value== keyboard.LEFT:
        left(velocity)
    elif ascii_value== keyboard.RIGHT:
        right(velocity)
    else:
        stop(velocity)
    #options
    if ascii_value== keyboard.A:
        status = keyboard.A
    elif ascii_value== keyboard.I:
        status = "stop mode"
    elif ascii_value == keyboard.L:
        scan_lidar()
    elif ascii_value == keyboard.Q:
        clear_screen()


def autonomous_mode(ascii_value):
    global status
    #print("autonomous mode")
    #sleep(2)
    if ascii_value== keyboard.I:
        status = keyboard.I
    if ascii_value== keyboard.M:
        status = keyboard.M
#switcher
def control(ascii_value):
    global status
    swithc = {
    "stop mode":stop_mode,
    keyboard.M:manual_mode,
    keyboard.I:stop_mode,
    keyboard.A:autonomous_mode,
    }
    function = swithc.get(status,lambda:None)
    #print (swithc.get(status))
    return function(ascii_value)


#LidarPoint1 = LidarPoint()

#print (Lidar_range)
#@property
#LidarPoint.x
#LidarPoint.y
#LidarPoint.z
#LidarPoint.layer_id
#LidarPoint.ime


# set up the motor speeds at 10% of the MAX_SPEED.

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:

    ascii_value = keyboard.getKey()
    ch = ascii_to_char(ascii_value)
    control(ascii_value)
    #print(ascii_value)
    #if ascii_value == keyboard.M:
        #print(keyboard.M)
    #if status == "manual mode":
            #contro(status)





    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
