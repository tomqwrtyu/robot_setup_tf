
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

using namespace std;
#include "cmath"



int main(int argc, char **argv) {

	ros::init(argc, argv, "robot_tf_publisher");

	ros::NodeHandle n;

	ros::Rate r(100);

	tf::TransformBroadcaster broadcaster;

	float x ,y ,z,w;
	
	float pitch, roll, yaw;
	n.getParam("roll",roll);
	n.getParam("pitch",pitch);
	n.getParam("yaw",yaw);
	double X,Y,Z;
	n.getParam("X",X);
	n.getParam("Y",Y);
	n.getParam("Z",Z);
	// rotate by x ,y ,z axis
        float base_pitch = (pitch * 3.14) / 180;
        float dpitch = (0 * 3.14) / 180;
	pitch = base_pitch + dpitch;
        float base_roll = (roll * 3.14) / 180;
        float droll = (0 * 3.14) / 180;
	roll = base_roll + droll;
        float base_yaw = (yaw * 3.14) / 180;
        float dyaw = (0 * 3.14) / 180;
	yaw = base_yaw + dyaw;

        tf::Quaternion rotation;
        rotation.setRPY(roll,pitch,yaw);

        w = cos(roll / 2)*cos(pitch / 2)*cos(yaw / 2) + sin(roll / 2)*sin(pitch / 2)*sin(yaw / 2);
        x = sin(roll / 2)*cos(pitch / 2)*cos(yaw / 2) - cos(roll / 2)*sin(pitch / 2)*sin(yaw / 2);
        y = cos(roll / 2)*sin(pitch / 2)*cos(yaw / 2) + sin(roll / 2)*cos(pitch / 2)*sin(yaw / 2);
        z = cos(roll / 2)*cos(pitch / 2)*sin(yaw / 2) - sin(roll / 2)*sin(pitch / 2)*sin(yaw / 2);
	// ROS_INFO("x = %f",pitch);
	// ROS_INFO("y = %f",roll);
	// ROS_INFO("z = %f",yaw);
	ROS_INFO("x = %f",x);
	ROS_INFO("y = %f",y);
	ROS_INFO("z = %f",z);
	ROS_INFO("w = %f",w);
        

	while (n.ok()) {
			n.getParam("roll",roll);
	n.getParam("pitch",pitch);
	n.getParam("yaw",yaw);
   base_pitch = (pitch * 3.14) / 180;
      dpitch = (0 * 3.14) / 180;
	pitch = base_pitch + dpitch;
      base_roll = (roll * 3.14) / 180;
       droll = (0 * 3.14) / 180;
	roll = base_roll + droll;
        base_yaw = (yaw * 3.14) / 180;
        dyaw = (0 * 3.14) / 180;
	yaw = base_yaw + dyaw;

        tf::Quaternion rotation;
        rotation.setRPY(roll,pitch,yaw);

        w = cos(roll / 2)*cos(pitch / 2)*cos(yaw / 2) + sin(roll / 2)*sin(pitch / 2)*sin(yaw / 2);
        x = sin(roll / 2)*cos(pitch / 2)*cos(yaw / 2) - cos(roll / 2)*sin(pitch / 2)*sin(yaw / 2);
        y = cos(roll / 2)*sin(pitch / 2)*cos(yaw / 2) + sin(roll / 2)*cos(pitch / 2)*sin(yaw / 2);
        z = cos(roll / 2)*cos(pitch / 2)*sin(yaw / 2) - sin(roll / 2)*sin(pitch / 2)*sin(yaw / 2);
	n.getParam("X",X);
	n.getParam("Y",Y);
	n.getParam("Z",Z);
		broadcaster.sendTransform(

				tf::StampedTransform(

						tf::Transform(tf::Quaternion(x,y,z,w), tf::Vector3(X, Y, Z)),

						ros::Time::now(),"base_link", "base_Camera"));

		r.sleep();

	}

}

