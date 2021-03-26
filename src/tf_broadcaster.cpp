
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

#include "sensor_msgs/Imu.h"

using namespace std;

#include "cmath"

class rpy_calib {
	ros::NodeHandle n;
	ros::Subscriber sub;
	tf::Quaternion rotation;
        tf::TransformBroadcaster broadcaster;
	public:

	    float R;
	    float P;
	    float Y;

	    float accel_x;
	    float accel_y;
	    float accel_z;
	    float x ,y ,z, dx, dy, dz;


	rpy_calib(){
	    sub = n.subscribe("camera/accel/sample", 1000, &rpy_calib::accel_callback,this);
       	    x = 1000.0;
       	    y = 0.0;
       	    z = 700.0;
       	    dx = -3.35;
       	    dy = 0.0;
       	    dz = 0.0;
	}
        void accel_callback(const sensor_msgs::Imu::ConstPtr& data){
            accel_x = 0 - data->linear_acceleration.x;
            accel_y = 0 - data->linear_acceleration.y;
            accel_z = 0 - data->linear_acceleration.z;
            R = 180 * atan(accel_y/sqrt(accel_x * accel_x + accel_z * accel_z)) / M_PI + 270 * M_PI / 180;
            P = 180 * atan(accel_x/sqrt(accel_y * accel_y + accel_z * accel_z)) / M_PI;
            Y = 180 * atan(accel_z/sqrt(accel_x * accel_x + accel_z * accel_z)) / M_PI;
            ROS_INFO("R: %f",R);
            ROS_INFO("P: %f",P);
            ROS_INFO("Y: %f",Y);
            rotation.setRPY(R,P,Y);
            broadcaster.sendTransform(
				tf::StampedTransform(
						tf::Transform(rotation, tf::Vector3( x+dx, y+dy, z+dz)),
						ros::Time::now(),"base_link", "base_Camera"));
        }

    
};




int main(int argc, char **argv) {

	ros::init(argc, argv, "robot_tf_publisher");

	

        rpy_calib server;



        ros::spin();

}

