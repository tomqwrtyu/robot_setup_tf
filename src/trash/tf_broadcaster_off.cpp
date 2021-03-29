
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

#include "sensor_msgs/Imu.h"

using namespace std;

#include "cmath"
#include <mutex>
#include "ctime"

class float3
{
    public:
    float x,y,z;
    void add(float dx,float dy,float dz)
    {
        x = x + dx;
        y = y + dy;
        z = z + dz;
    }
};

class rpy_calib {
	ros::NodeHandle n;
	ros::Subscriber accel_sub;
	ros::Subscriber gyro_sub;
	tf::Quaternion rotation;
        tf::TransformBroadcaster broadcaster;
        // theta is the angle of camera rotation in x, y and z components
        float3 theta;
        std::mutex theta_mtx;
        /* alpha indicates the part that gyro and accelerometer take in computation of theta; higher alpha gives more weight to gyro, but too high
        values cause drift; lower alpha gives more weight to accelerometer, which is more sensitive to disturbances */
        float alpha = 0.98f;
        bool firstGyro = true;
        bool firstAccel = true;
        // Keeps the arrival time of previous gyro frame
        double last_ts_gyro;
        float x,y,z,dx,dy,dz;
	public:
	rpy_calib(){
	    accel_sub = n.subscribe("camera/accel/sample", 1000, &rpy_calib::accel_callback,this);
            gyro_sub = n.subscribe("camera/gyro/sample", 1000, &rpy_calib::gyro_callback,this);
       	    x = 1000.0;
       	    y = 0.0;
       	    z = 700.0;
       	    dx = -3.35;
       	    dy = 0.0;
       	    dz = 0.0;
	}
        void accel_callback(const sensor_msgs::Imu::ConstPtr& data)
        {
            // Holds the angle as calculated from accelerometer data
            float3 accel_angle;
            float accel_data_x = data->linear_acceleration.x;
            float accel_data_y = data->linear_acceleration.y;
            float accel_data_z = data->linear_acceleration.z;
   
            // Calculate rotation angle from accelerometer data
            accel_angle.z = atan2(accel_data_y, accel_data_z);
            accel_angle.x = atan2(accel_data_x, sqrt(accel_data_y * accel_data_y + accel_data_z * accel_data_z));
    
            // If it is the first iteration, set initial pose of camera according to accelerometer data (note the different handling for Y axis)
            std::lock_guard<std::mutex> lock(theta_mtx);
            if (firstAccel)
            {
                firstAccel = false;
                theta = accel_angle;
                // Since we can't infer the angle around Y axis using accelerometer data, we'll use PI as a convetion for the initial pose
                theta.y = M_PI;
            }
            else
            {
                /* 
                Apply Complementary Filter:
                    - high-pass filter = theta * alpha:  allows short-duration signals to pass through while filtering out signals
                      that are steady over time, is used to cancel out drift.
                    - low-pass filter = accel * (1- alpha): lets through long term changes, filtering out short term fluctuations 
                */
                theta.x = theta.x * alpha + accel_angle.x * (1 - alpha);
                theta.z = theta.z * alpha + accel_angle.z * (1 - alpha);
            }
            rotation.setRPY(theta.x, theta.y, theta.z);
            ROS_INFO("R: %.2f,P: %.2f,Y: %.2f",theta.x*180/M_PI, theta.y*180/M_PI, theta.z*180/M_PI);
            broadcaster.sendTransform(
				tf::StampedTransform(
						tf::Transform(rotation, tf::Vector3( x+dx, y+dy, z+dz)),
						ros::Time::now(),"base_link", "base_Camera"));
        }
        void gyro_callback(const sensor_msgs::Imu::ConstPtr& data)
        {
            double ts;
            ts = data->header.stamp.sec;
            if (firstGyro) // On the first iteration, use only data from accelerometer to set the camera's initial position
            {
                firstGyro = false;
                last_ts_gyro = ts;
                return;
            }
            // Holds the change in angle, as calculated from gyro
            float3 gyro_angle;
 
            // Initialize gyro_angle with data from gyro
            gyro_angle.x = data->angular_velocity.x; // Pitch
            gyro_angle.y = data->angular_velocity.y; // Yaw
            gyro_angle.z = data->angular_velocity.z; // Roll
 
            // Compute the difference between arrival times of previous and current gyro frames
            double dt_gyro = (ts - last_ts_gyro) / 1000.0;
            last_ts_gyro = ts;
 
            // Change in angle equals gyro measures * time passed since last measurement
            gyro_angle.x = gyro_angle.x * static_cast<float>(dt_gyro);
            gyro_angle.y = gyro_angle.y * static_cast<float>(dt_gyro);
            gyro_angle.z = gyro_angle.z * static_cast<float>(dt_gyro);
 
            // Apply the calculated change of angle to the current angle (theta)
            std::lock_guard<std::mutex> lock(theta_mtx);
            theta.add(-gyro_angle.z, -gyro_angle.y, gyro_angle.x);
     }
    
};



int main(int argc, char **argv) {

	ros::init(argc, argv, "robot_tf_publisher");

	

        rpy_calib server;



        ros::spin();

}

