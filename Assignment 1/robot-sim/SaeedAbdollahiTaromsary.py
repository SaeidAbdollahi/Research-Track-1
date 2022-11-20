from __future__ import print_function

import time
import random
from sr.robot import *

"""
Assignment 1 Python Script 

Student Name: Saeed Abdollahi Taromsari
Student Number: S5397691

Program Description: 
	Move the robot in the environmet to detect and move objects so that every silver object sit near the gold object
"""

"""Global Variables:"""
""" float: Threshold for the control of the linear distance"""
a_th = 2.0

""" float: Threshold for the control of the orientation"""
d_th = 0.8

""" instance of the class Robot"""
R = Robot()

""" Select the object state: 0 or 1 """
object_state=0;

"""Custom Functions:"""
def drive(speed, seconds):
    """
    Function for setting a linear velocity
    
    Args: speed (int): the speed of the wheels
	  seconds (int): the time interval
    """
    speed=speed*5
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0

def turn(speed, seconds):
    """
    Function for setting an angular velocity
    
    Args: speed (int): the speed of the wheels
	  seconds (int): the time interval
    """
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = -speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0


def find_silver():
	#Function to detect silver object
	dist =100
	for token in R.see():
		if token.dist < dist and token.info.marker_type is MARKER_TOKEN_SILVER: # found a silver object
			print("A silver object found!")
			dist=token.dist
			rot_y=token.rot_y
	if dist==100:
		return -1, -1
    	else:
   		return dist, rot_y
   		
def find_gold():
	#Function to detect gold object
	dist =100
	for token in R.see():
		if token.dist < dist and token.info.marker_type is MARKER_TOKEN_GOLD: # found a gold object
			print("A gold object found!")
			dist=token.dist
			rot_y=token.rot_y
	if dist==100:
		return -1, -1
    	else:
   		return dist, rot_y
   		

def moveToObject(obj_dis,obj_rot):
	#A function to move the robot to the target
	if obj_dis <0.4:
    		print("Found it!")
        	test=R.grab() # if we are close to the token, we grab it.
		print("Gotcha!")
		object_state = 1;
	elif -a_th<= obj_rot <= a_th: # if the robot is well aligned with the token, we go forward
       		print("Ah, here we are!.")
       		drive(10, 0.5)
       		object_state = 0;
	elif obj_rot < -a_th: # if the robot is not well aligned with the token, we move it on the left or on the right
		print("Left a bit...")
       		turn(-2, 0.5)
       		object_state = 0;
	elif obj_rot > a_th:
        	print("Right a bit...")
       		turn(+2, 0.5)
       		object_state = 0;
	return object_state
	
print("*******************Robot Task Starts*******************")
while 1:
	#Step 1: Try to find a silver object
	print("Robot Message: Try to find a silver object!")
	dist_silver,rot_silver = find_silver()
	if dist_silver == -1:
		#If no silver object found, do a random movement!
		print("Robot Message: If no silver object found, do a random movement!")
		drive(-10, 0.5)
		turn(8 + random.randint(-3, 3), 1)
	if object_state == 0:
		#Step 2: A a silver object found, move robot to the object
		print("Robot Message: A a silver object found, move robot to the object!")
		object_state = moveToObject(dist_silver,rot_silver)
	elif object_state == 1:
		while 1:
			#Step 3: A silver object found, now try to find a gold object
			print("Robot Message: A silver object found, now try to find a gold object!")
			dist_gold,rot_gold = find_gold()
			if dist_gold == -1:
				#If no gold object found, do a random movement!
				print("Robot Message: If no gold object found, do a random movement!")
				drive(8 + random.randint(-3, 3), 1)
				turn(+10, 1)

			#Step 4: A a gold object found, move robot to the object
			print("Robot Message: A a gold object found, move robot to the object!")
			moveToObject(dist_gold,rot_gold)

			if dist_gold <d_th:
				#Step 5: The robot puts the silver objects near the gold object, now release the silver object
				print("Robot Message: The robot puts the silver objects near the gold object, now release the silver object!")
				R.release()
				drive(-10, 0.5)
				turn(8 + random.randint(0, 8), 5)
				object_state = 0

				#Okay, now try to do another task
				print("Okay, now try to do another task!")
				break
				
