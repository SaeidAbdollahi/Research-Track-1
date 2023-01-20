#! /usr/bin/env python3

"""
    Odometer Node

        Description:
            1-Subscribes to the '/odom' topic to receive the Odometry message

            2-Publishes the odometry data as a 'OdoSensor 'custom message

    Author: Saeed Abdollahi Taromsari,   S.N: 5397691      
"""

import sys
import rospy
from nav_msgs.msg import Odometry
from robot_sim.msg import OdoSensor

"""
OdoSensor custom message

    Description:
        A custom message to send the odometry data

    Message Type:
        OdoSensor: 
            float32 x
            float32 y
            float32 vel_x
            float32 vel_y
"""
odoMsg = OdoSensor()



"""
setOdoMessage function

    Description:
        It is the 'odoMessageSubscriber' handler

    Message Type:
        Odometry: 
                pose:
                    pose:
                        position:
                            x
                            y
                twist:
                    twist:
                        linear:
                            x
                            y

    Arguments:   
        msg: The odometry data published on the '/odom' topic 
"""
def setOdoMessage(msg):
    global odoMsg

    #Fills the custom odoMsg object by the odemetry data
    odoMsg.x = msg.pose.pose.position.x
    odoMsg.y = msg.pose.pose.position.y
    odoMsg.vel_x = msg.twist.twist.linear.x
    odoMsg.vel_y = msg.twist.twist.linear.y


"""
main function

    Description:
        The start point of the node

    Arguments:   
        None
"""    
if __name__ == "__main__":

    try:#Trys to create a ROS node
        rospy.init_node('odemeter')

        #Creates a subscriber to receive the 'Odometry' message
        odoMessageSubscriber = rospy.Subscriber("odom", Odometry, setOdoMessage)
        #Creates a publisher to publish the odometry data by a 'OdoSensor' custom message
        odoMessagePublisher = rospy.Publisher('odometer', OdoSensor, queue_size=10)

        #Sets the clock rate fo the current node
        rate = rospy.Rate(10) # 10hz

        #The infinite loop of the programe
        while not rospy.is_shutdown():
            if odoMsg:#If odoMsg is available, publish the message 
                odoMessagePublisher.publish(odoMsg)
                #Sleep for a predefined time
                rate.sleep()

    except rospy.ROSInterruptException:#If the process failed, show an error message
        print("program interrupted before completion", file=sys.stderr)


            

        