
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

#include "sensor_msgs/Imu.h"

using namespace std;

#include "cmath"
#include "ctime"

class rpy_calib {
	ros::NodeHandle n;
	ros::Subscriber sub;
	tf::Quaternion rotation;
        tf::TransformBroadcaster broadcaster;
	public:

	    float R,Rd;
	    float P,Pd;
	    float Y,Yd;
            time_t last = time(0);

	    float accel_x;
	    float accel_y;
	    float accel_z;
	    float x ,y ,z, dx, dy, dz;


	rpy_calib()
        {
	    sub = n.subscribe("camera/accel/sample", 1000, &rpy_calib::accel_callback,this);
       	    x = 1000.0;
       	    y = 0.0;
       	    z = 700.0;
       	    dx = -3.35;
       	    dy = 0.0;
       	    dz = 0.0;
	}
        void accel_callback(const sensor_msgs::Imu::ConstPtr& data)
        {
            accel_x = 0 - data->linear_acceleration.x;
            accel_y = 0 - data->linear_acceleration.y;
            accel_z = 0 - data->linear_acceleration.z;
            R = roundf((0 - atan2(accel_y,sqrt(accel_x * accel_x + accel_z * accel_z))) * 100) / 100;
            P = roundf((0 - atan2(accel_x,sqrt(accel_y * accel_y + accel_z * accel_z))) * 100) / 100;
            if(roundf(accel_y) == 0){
                Y = 0;
            }
            else {
                //Y = 0 - atan(accel_z/sqrt(accel_x * accel_x + accel_y * accel_y));
                Y = roundf((0 - atan2(accel_y, accel_z)) * 100) / 100;
            }
            if(time(0) - last >= 1){
            Rd = 180 * R / M_PI;
            Pd = 180 * P / M_PI;
            Yd = 180 * Y / M_PI;
            ROS_INFO("R: %.2f,P: %.2f,Y: %.2f",Rd,Pd,Yd);
            last = time(0);
            }
            rotation.setRPY(R,P,0);
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

