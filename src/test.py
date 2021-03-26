#!/usr/bin/env python  
import rospy
import math
import geometry_msgs.msg
from sensor_msgs.msg import Imu

def callback(data):
    print(data.linear_acceleration.x)



if __name__ == '__main__':
    
    rospy.init_node('Test')
    sub = rospy.Subscriber('/camera/accel/sample', Imu, callback)
    rospy.spin()

    

