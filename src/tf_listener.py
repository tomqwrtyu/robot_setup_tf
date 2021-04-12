#!/usr/bin/env python  
import rospy
import subprocess #shell command
import math
import tf
import geometry_msgs.msg
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32MultiArray
from robot_setup_tf.srv import *

class CPTransform:
    def __init__(self):
        self.listener = 0

    def _request_handler(self,request):
        response = point_transformResponse()
        lspoint = PointStamped()
        lspoint.header.frame_id = "base_Camera"
        lspoint.header.stamp = rospy.Time()
        lspoint.point.x = request.cp_pos[0]
        lspoint.point.y = request.cp_pos[1]
        lspoint.point.z = request.cp_pos[2]
        try:
            base_point = PointStamped()
            base_point = self.listener.transformPoint("base_link", lspoint)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("Point translation failed.")

        response.tf_pos = [base_point.point.x,base_point.point.y,base_point.point.z]
        return response

    def start(self):
        rospy.init_node('CameraPointsTransformer')
        rospy.loginfo('Transform service is on.')
        service = rospy.Service('point_transform', point_transform, self._request_handler)
        self.listener = tf.TransformListener()
        subprocess.call(["rosrun", "beacon_cam", "ros_detection_yolov4_async.py"])


if __name__ == '__main__':
    
    transformer = CPTransform()
    transformer.start()
    rospy.spin()

    

