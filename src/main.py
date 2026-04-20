#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from time import sleep
import math
import sys


ev3 = EV3Brick()

gripper = Motor(Port.A)  
arm = Motor(Port.B)      
base = Motor(Port.C)     
conveyer = Motor(Port.D)

arm_home = TouchSensor(Port.S2)
base_home = TouchSensor(Port.S1)
color_sensor = ColorSensor(Port.S3)


L0 = 40.0
L1 = 50.0   
L2 = 95.0   
L3 = 185.0  

L12 = math.sqrt(L1**2 + L2**2 - 2*L1*L2*math.cos(math.radians(135)))
L_Arm = L12 + L3

gear_base = 36.0 / 12.0  
gear_arm = 40.0 / 8.0    

current_base_angle = 0


def log(message):
    print(message)


def inverse_kinematics_theta1(x, y):
    theta1 = math.degrees(math.atan2(y, x))
    return theta1


def inverse_kinematics_theta2(x, z):
    dz = z - L0
    dz = max(min(dz, L_Arm), -L_Arm)
    theta = math.asin(dz / L_Arm)
    return -math.degrees(theta)


def home_joint(motor, sensor, direction, home_offset, speed=100):
    motor.run(direction * speed)
    while not sensor.pressed():
        wait(10)
    motor.stop(Stop.BRAKE)
    motor.run_angle(-direction * speed, home_offset, Stop.HOLD)
    motor.reset_angle(0)


def perform_homing():
    ev3.screen.clear()
    ev3.screen.draw_text(10, 50, "Homing...")
    ev3.speaker.say("Homing")
    
    home_joint(motor=base, sensor=base_home, direction=1, home_offset=120 * gear_base)
    home_joint(motor=arm, sensor=arm_home, direction=-1, home_offset=15)
    
    base.reset_angle(0)
    arm.reset_angle(0)
    
    gripper.run_angle(200, -90, Stop.HOLD)
    gripper.reset_angle(0)

    ev3.screen.clear()
    ev3.screen.draw_text(10, 50, "Home Complete")
    ev3.speaker.say("Homing done")
    
    wait(1000)


def gripper_open():
    ev3.screen.clear()
    ev3.screen.draw_text(10, 50, "Opening gripper")
    gripper.run_angle(90, 90, Stop.HOLD)
    wait(500)


def gripper_close():
    ev3.screen.clear()
    ev3.screen.draw_text(10, 50, "Closing gripper")
    gripper.run_angle(90, -90, Stop.HOLD)
    wait(500)


def move_arm_to(x, z, speed=120):
    global current_base_angle
    
    if current_base_angle != 0:
        y = x * math.tan(math.radians(current_base_angle))
    else:
        y = 0
    
    theta2 = inverse_kinematics_theta2(x, z)
    motor_angle = theta2 * gear_arm
    
    arm.run_angle(speed, motor_angle - arm.angle(), Stop.HOLD)


def move_base_to(x, y, speed=150):
    global current_base_angle
    
    theta1 = inverse_kinematics_theta1(x, y)
    motor_target = theta1 * gear_base
    
    base.run_angle(speed, motor_target - base.angle(), Stop.HOLD)
    current_base_angle = theta1


SAMPLES = 20
TOL = 20


def read_average_color():
    r_total, g_total, b_total = 0, 0, 0
    
    for _ in range(SAMPLES):
        r, g, b = color_sensor.rgb()
        r_total += r
        g_total += g
        b_total += b
        sleep(0.05)

    return (
        r_total // SAMPLES,
        g_total // SAMPLES,
        b_total // SAMPLES
    )


def is_within_tolerance(measured, reference, tol=TOL):
    return (abs(measured[0] - reference[0]) <= tol and
            abs(measured[1] - reference[1]) <= tol and
            abs(measured[2] - reference[2]) <= tol)


ev3.screen.clear()
ev3.screen.draw_text(10, 50, "Starting...")
wait(500)

perform_homing()

ev3.screen.clear()
ev3.screen.draw_text(10, 30, "Calibration")
ev3.screen.draw_text(10, 60, "Place BLUE ball")
ev3.speaker.say("Calibrating RGB values")
ev3.speaker.say("Place BLUE ball")
sleep(10)
BLUE = read_average_color()
ev3.speaker.beep()

ev3.screen.clear()
ev3.screen.draw_text(10, 30, "Calibration")
ev3.screen.draw_text(10, 60, "Place BLACK ball")
ev3.speaker.say("Place BLACK ball")
sleep(10)
BLACK = read_average_color()
ev3.speaker.beep()

ev3.screen.clear()
ev3.screen.draw_text(10, 30, "Calibration")
ev3.screen.draw_text(10, 60, "Place RED ball")
ev3.speaker.say("Place Red ball")
sleep(10)
RED = read_average_color()
ev3.speaker.beep()

ev3.screen.clear()
ev3.screen.draw_text(10, 30, "Calibration")
ev3.screen.draw_text(10, 60, "Place GREEN ball")
ev3.speaker.say("Place GREEN ball")
sleep(10)
GREEN = read_average_color()
ev3.speaker.beep()

ev3.screen.clear()
ev3.screen.draw_text(10, 50, "Calibration Done")
ev3.speaker.say("Calibration complete")

wait(2000)

ev3.screen.clear()
ev3.screen.draw_text(10, 50, "Starting Process")
ev3.speaker.say("Starting process")
wait(1000)

for i in range(40):
    ev3.screen.clear()
    ev3.screen.draw_text(10, 30, "Ball " + str(i+1) + " of 4")
    ev3.screen.draw_text(10, 60, "Place ball now")
    ev3.speaker.say("Place ball " + str(i+1))
    
    wait(1000)
    
    color = color_sensor.color()
    rgb = color_sensor.rgb()
    
    ev3.screen.clear()
    ev3.screen.draw_text(10, 30, "Detected:")
    ev3.screen.draw_text(10, 60, str(color))
    ev3.screen.draw_text(10, 90, "RGB:" + str(rgb[0]) + "," + str(rgb[1]) + "," + str(rgb[2]))
    wait(1000)
    
    if color == Color.BLACK:
        ev3.screen.clear()
        ev3.screen.draw_text(10, 50, "BLACK ball")
        ev3.speaker.say("Color is Black")
        
        conveyer.stop()
        conveyer.run_angle(80, -700, Stop.HOLD)
        
    elif color == Color.GREEN:
        ev3.screen.clear()
        ev3.screen.draw_text(10, 50, "GREEN ball")
        ev3.speaker.say("Color is Green")
        
        conveyer.stop()
        conveyer.run_angle(80, 70, Stop.HOLD)
        
    elif color == Color.RED:
        ev3.screen.clear()
        ev3.screen.draw_text(10, 50, "RED ball")
        ev3.speaker.say("Color is red")
        
        conveyer.stop()
        conveyer.run_angle(80, -300, Stop.HOLD)
        
        try:
            gripper_open()
            
            ev3.screen.clear()
            ev3.screen.draw_text(10, 50, "Moving to pick")
            wait(500)
            move_arm_to(-110, -180)
            
            wait(500)
            gripper_close()
            
            ev3.screen.clear()
            ev3.screen.draw_text(10, 50, "Lifting")
            wait(500)
            move_arm_to(50, 50)
            
            ev3.screen.clear()
            ev3.screen.draw_text(10, 50, "Rotating base")
            wait(500)
            move_base_to(0, 50)
            
            ev3.screen.clear()
            ev3.screen.draw_text(10, 50, "Lowering")
            wait(500)
            move_arm_to(-200, -250)
            
            wait(500)
            gripper_open()
            
            ev3.screen.clear()
            ev3.screen.draw_text(10, 50, "Returning")
            wait(500)
            move_arm_to(50, 50)
            
            wait(500)
            perform_homing()
            
            ev3.screen.clear()
            ev3.screen.draw_text(10, 50, "RED complete")
            ev3.speaker.beep()
            wait(1000)
            
        except Exception as e:
            ev3.screen.clear()
            ev3.screen.draw_text(10, 50, "ERROR!")
            ev3.speaker.say("Error occurred")
            wait(2000)
            
    elif color == Color.BLUE:
        ev3.screen.clear()
        ev3.screen.draw_text(10, 50, "BLUE ball")
        ev3.speaker.say("Color is blue")
        
        conveyer.stop()
        conveyer.run_angle(80, -300, Stop.HOLD)
        
        try:
            gripper_open()
            
            ev3.screen.clear()
            ev3.screen.draw_text(10, 50, "Moving to pick")
            wait(500)
            move_arm_to(-110, -180)
            
            wait(500)
            gripper_close()
            
            ev3.screen.clear()
            ev3.screen.draw_text(10, 50, "Lifting")
            wait(500)
            move_arm_to(50, 50)
            
            ev3.screen.clear()
            ev3.screen.draw_text(10, 50, "Rotating base")
            wait(500)
            move_base_to(0, -50)
            
            ev3.screen.clear()
            ev3.screen.draw_text(10, 50, "Lowering")
            wait(500)
            move_arm_to(-200, -250)
            
            wait(500)
            gripper_open()
            
            ev3.screen.clear()
            ev3.screen.draw_text(10, 50, "Returning")
            wait(500)
            move_arm_to(50, 30)
            
            wait(500)
            perform_homing()
            
            ev3.screen.clear()
            ev3.screen.draw_text(10, 50, "BLUE complete")
            ev3.speaker.beep()
            wait(1000)
            
        except Exception as e:
            ev3.screen.clear()
            ev3.screen.draw_text(10, 50, "ERROR!")
            ev3.speaker.say("Error occurred")
            wait(2000)
            
    else:
        ev3.screen.clear()
        ev3.screen.draw_text(10, 30, "UNKNOWN color")
        ev3.screen.draw_text(10, 60, "Value: " + str(color))
        ev3.screen.draw_text(10, 90, "Skipping...")
        ev3.speaker.say("Unknown color")
        wait(2000)

conveyer.stop()
ev3.screen.clear()
ev3.screen.draw_text(10, 50, "ALL DONE!")
ev3.speaker.say("PROCESS COMPLETE")

wait(3000)