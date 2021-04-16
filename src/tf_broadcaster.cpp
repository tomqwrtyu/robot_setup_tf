
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

using namespace std;
#include "cmath"
#include "ctime"
#include "bits/stdc++.h"



int main(int argc, char **argv) {

	ros::init(argc, argv, "robot_tf_publisher");

	ros::NodeHandle n;

	ros::Rate r(100);

	tf::TransformBroadcaster broadcaster;
	float dpitch;
	float droll;
	float dyaw;
	n.getParam("dP",dpitch);
	n.getParam("dR",droll);
	n.getParam("dY",dyaw);
	float x ,y ,z, dx, dy, dz;
        double last = time(0);
        n.getParam("dx",dx);
	n.getParam("dy",dy);
	n.getParam("dz",dz);
	float pitch, roll, yaw;
	// rotate by x ,y ,z axis
        float base_pitch = (0 * 3.14) / 180;
         dpitch = (dpitch * 3.14) / 180;
	pitch = base_pitch + dpitch;
        float base_roll = (270 * 3.14) / 180;
         droll = (droll * 3.14) / 180;
	roll = base_roll + droll;
        float base_yaw = (0 * 3.14) / 180;
        dyaw = (dyaw * 3.14) / 180;
	yaw = base_yaw + dyaw;
        
        x = 1000.0;
        y = 0.0;
        z = 480.0;
        

        tf::Quaternion rotation;
        rotation.setRPY(roll,pitch,yaw);

        //w = cos(roll / 2)*cos(pitch / 2)*cos(yaw / 2) + sin(roll / 2)*sin(pitch / 2)*sin(yaw / 2);
        //x = sin(roll / 2)*cos(pitch / 2)*cos(yaw / 2) - cos(roll / 2)*sin(pitch / 2)*sin(yaw / 2);
        //y = cos(roll / 2)*sin(pitch / 2)*cos(yaw / 2) + sin(roll / 2)*cos(pitch / 2)*sin(yaw / 2);
        //z = cos(roll / 2)*cos(pitch / 2)*sin(yaw / 2) - sin(roll / 2)*sin(pitch / 2)*sin(yaw / 2);
        //if(time(0) - last >= 1){
	//ROS_INFO("R = %f,P = %f,Y = %f",roll,pitch,yaw);
        //last = time(0);
        //}
	//ROS_INFO("x = %f",x);
	//ROS_INFO("y = %f",y);
	//ROS_INFO("z = %f",z);
	//ROS_INFO("w = %f",w);
        

	while (n.ok()) {
        n.getParam("dx",dx);
	n.getParam("dy",dy);
	n.getParam("dz",dz);
	n.getParam("dP",dpitch);
	n.getParam("dR",droll);
	n.getParam("dY",dyaw);
        float base_pitch = (0 * 3.14) / 180;
         dpitch = (dpitch * 3.14) / 180;
	pitch = base_pitch + dpitch;
        float base_roll = (270 * 3.14) / 180;
         droll = (droll * 3.14) / 180;
	roll = base_roll + droll;
        float base_yaw = (0 * 3.14) / 180;
        dyaw = (dyaw * 3.14) / 180;
	yaw = base_yaw + dyaw;
	rotation.setRPY(roll,pitch,yaw);
		broadcaster.sendTransform(

				tf::StampedTransform(

						tf::Transform(rotation, tf::Vector3( x+dx, y+dy, z+dz)),

						ros::Time::now(),"base_link", "base_Camera"));

		r.sleep();

	}

}

