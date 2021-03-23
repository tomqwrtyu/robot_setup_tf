
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

using namespace std;
#include "cmath"



int main(int argc, char **argv) {

	ros::init(argc, argv, "robot_tf_publisher");

	ros::NodeHandle n;

	ros::Rate r(100);

	tf::TransformBroadcaster broadcaster;

	float x ,y ,z, dx, dy, dz;

	float pitch, roll, yaw;
	// rotate by x ,y ,z axis
        float base_pitch = (0 * 3.14) / 180;
        float dpitch = (0 * 3.14) / 180;
	pitch = base_pitch + dpitch;
        float base_roll = (270 * 3.14) / 180;
        float droll = (0 * 3.14) / 180;
	roll = base_roll + droll;
        float base_yaw = (0 * 3.14) / 180;
        float dyaw = (0 * 3.14) / 180;
	yaw = base_yaw + dyaw;
        
        x = 1000.0;
        y = 0.0;
        z = 700.0;
        dx = -3.35;
        dy = 0.0;
        dz = 0.0;
        

        tf::Quaternion rotation;
        rotation.setRPY(roll,pitch,yaw);

        //w = cos(roll / 2)*cos(pitch / 2)*cos(yaw / 2) + sin(roll / 2)*sin(pitch / 2)*sin(yaw / 2);
        //x = sin(roll / 2)*cos(pitch / 2)*cos(yaw / 2) - cos(roll / 2)*sin(pitch / 2)*sin(yaw / 2);
        //y = cos(roll / 2)*sin(pitch / 2)*cos(yaw / 2) + sin(roll / 2)*cos(pitch / 2)*sin(yaw / 2);
        //z = cos(roll / 2)*cos(pitch / 2)*sin(yaw / 2) - sin(roll / 2)*sin(pitch / 2)*sin(yaw / 2);
	// ROS_INFO("x = %f",pitch);
	// ROS_INFO("y = %f",roll);
	// ROS_INFO("z = %f",yaw);
	//ROS_INFO("x = %f",x);
	//ROS_INFO("y = %f",y);
	//ROS_INFO("z = %f",z);
	//ROS_INFO("w = %f",w);
        

	while (n.ok()) {

		broadcaster.sendTransform(

				tf::StampedTransform(

						tf::Transform(rotation, tf::Vector3( x+dx, y+dy, z+dz)),

						ros::Time::now(),"base_link", "base_Camera"));

		r.sleep();

	}

}

