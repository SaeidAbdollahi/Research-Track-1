#! /usr/bin/env python3

"""
    Kinematic Monitor Node

        Description:
            1-Subscribes to the 'odometer' topic to receive the custom OdoSensor message

            2-Prints the 'distance to the target' and 'average speed' of the robot

            3-Publishes these data as a custom 'KinematicData' message

    Author: Saeed Abdollahi Taromsari,   S.N: 5397691      
"""


import sys
import rospy
import math
from geometry_msgs.msg import Point
from robot_sim.msg import OdoSensor, KinematicData


"""
OdoSensor custom message

    Description:
        A custom message to send the odometry data

    Message Type:
        KinematicData: 
            float32 distance
            float32 vel_x_avrg
            float32 vel_y_avrg
"""
kinematicData = KinematicData()
kinematicData.distance = 0
kinematicData.vel_x_avrg = 0
kinematicData.vel_y_avrg = 0

#Data type to save the target position
desiredPosition = Point()

#Queue used to store list of velocities so that we get the average
velXData = []
velYData = []

#Sum of the velocity data for x and y direction
sumVx = 0
sumVy = 0

#Period of averaging
period = 10


"""
movingAverage function

    Description:
        Compute the moving average of the velocity data

    Arguments:   
        vx: Current velocity in the x direction
        vy: Current veloity in the y direction
"""  
def movingAverage(vx, vy):
    global sumVx, sumVy, velXData, velYData, period

    #Add new velocity to the sum
    sumVx += vx
    sumVy += vy

    #Add the velocity to the list
    velXData.append(vx)
    velYData.append(vy)

    #Updating size so that length of data set should be equal to period as a normal mean has
    if (len(velXData) > period):
        sumVx -= velXData.pop(0)
        sumVy -= velYData.pop(0)
        return [sumVx / period, sumVy/period] 

    return [0, 0]

    
"""
updateKinematicInfo function

    Description:
        Compute the 'distance to the target' and 'average speed' of the robot 

    Arguments:   
        odoMsg: A message of type 'OdoSensor'
"""  
def updateKinematicInfo(odoMsg):
    global processData

    #Read the target position from the parameter server
    desiredPosition.x = rospy.get_param('des_pos_x')
    desiredPosition.y = rospy.get_param('des_pos_y')

    #Compute 'distance to the target'
    kinematicData.distance = math.sqrt(pow(desiredPosition.y - odoMsg.y, 2) + pow(desiredPosition.x - odoMsg.x, 2))

    #Compute the 'average velocity' by using the moving average algorithm for the last 10 velocity data
    averageVelocities = movingAverage(odoMsg.vel_x, odoMsg.vel_y)

    #Update the 'KinematicData' custom message
    kinematicData.vel_x_avrg = averageVelocities[0]
    kinematicData.vel_y_avrg = averageVelocities[1]


"""
main function

    Description:
        The start point of the node

    Arguments:   
        None
"""   
if __name__ == "__main__":

    #Configs the print rate for printing the kinematic data on the console
    printRate = rospy.get_param('print_rate')

    try:#Trys to create a ROS node
        rospy.init_node('kinematic_monitor')

        #Creates a subscriber to receive the 'OdoSensor' custom message
        kinematicSubscriber = rospy.Subscriber("odometer", OdoSensor, updateKinematicInfo)
        #Creates a publisher to publish the odometry data by a 'KinematicData' custom message
        kinematicPublisher = rospy.Publisher('kinematic_monitor', KinematicData, queue_size=10) 

        #Sets the clock rate fo the current node            
        rate = rospy.Rate(printRate)

        #The infinite loop of the programe
        while not rospy.is_shutdown():
            if kinematicData:#If kinematicData is available, publish the message 
                kinematicPublisher.publish(kinematicData)
                print("Distance to Taget: %.3f, Average Vx, Vy: %.3f, %.3f"%(kinematicData.distance, kinematicData.vel_x_avrg, kinematicData.vel_y_avrg ))
                #Sleep for a predefined time
                rate.sleep()                
    except rospy.ROSInterruptException:#If the process failed, show an error message
        print("program interrupted before completion", file=sys.stderr)
       
  
    



            

        