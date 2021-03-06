
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

#include "sensor_msgs/Imu.h"

using namespace std;

#include "cmath"
#include "ctime"
#include "bits/stdc++.h"

class rpy_calib {

	
	public:
	ros::NodeHandle nh;
	ros::Subscriber sub;
	tf::Quaternion rotation;
        tf::TransformBroadcaster broadcaster;
            int calib_count;
	    float R,Rd;
	    float P,Pd;
	    float Y,Yd,Yaw;
		float R_array[99];
		float P_array[99];
		float Y_array[99];
            time_t last = time(0);
            int is_calibrated;
	    float accel_x;
	    float accel_y;
	    float accel_z;
	    float x ,y ,z;


	rpy_calib()
        {
	        sub = nh.subscribe("camera/accel/sample", 1000, &rpy_calib::accel_callback,this);
		x = 970.0;
		y = -66.0;
		z = 480.0;
		Yaw = -0.2 * M_PI / 180;
			calib_count = 0;
			is_calibrated = 0;
			
	}
	bool nh_check()
	{
		if(nh.ok())
		{
			return true;
		}
		return false;
	}    
    void accel_callback(const sensor_msgs::Imu::ConstPtr& data)
    {
        if(is_calibrated == 1)
	{
	    broadcaster.sendTransform(
	        tf::StampedTransform(
		tf::Transform(rotation, tf::Vector3( x, y, z)),
		ros::Time::now(),"base_link", "base_Camera"));
            return;
	}
        accel_x = data->linear_acceleration.x;
        accel_y = data->linear_acceleration.y;
        accel_z = data->linear_acceleration.z;
        R = roundf((M_PI - atan2(accel_y,sqrt(accel_x * accel_x + accel_z * accel_z))) * 100) / 100;
        P = roundf(0 - atan2(accel_x,sqrt(accel_y * accel_y + accel_z * accel_z)) * 100) / 100;
        if(roundf(accel_y) == 0){
            Y = 0;
        }
        else {
            //Y = 0 - atan(accel_z/sqrt(accel_x * accel_x + accel_y * accel_y));
            Y = roundf((atan2(accel_y, accel_z)) * (-100)) / 100;
        }
        if(time(0) - last >= 1)
		{
            Rd = 180 * R / M_PI;
            Pd = 180 * P / M_PI;
            Yd = 180 * Y / M_PI;
            ROS_INFO("R: %.2f,P: %.2f,Y: %.2f",Rd,Pd,Yd);
            last = time(0);
        }
        calibration_count();
        rotation.setRPY(R,P,Yaw);
        broadcaster.sendTransform(
	            tf::StampedTransform(
				tf::Transform(rotation, tf::Vector3( x, y, z)),
				ros::Time::now(),"base_link", "base_Camera"));
    }

	void send_tf()
	{
		if(is_calibrated == 1)
		{
		    broadcaster.sendTransform(
			tf::StampedTransform(
			tf::Transform(rotation, tf::Vector3( x, y, z)),
			ros::Time::now(),"base_link", "base_Camera"));
		    return;
		}
	}	

	void calibration_count()
	{
		if(calib_count < 99 && is_calibrated == 0)
		{
                        R_array[calib_count] = R;
                        P_array[calib_count] = P;
                        Y_array[calib_count] = Y;
                        calib_count += 1;
		}
		else
		{
			is_calibrated = 1;
			R = caculate_median(R_array);
			P = caculate_median(P_array);
			Y = caculate_median(Y_array);
			Rd = 180 * R / M_PI;
         		Pd = 180 * P / M_PI;
          	        Yd = 180 * Y / M_PI;
                        ROS_INFO("Calibration done! R: %.2f,P: %.2f,Y: %.2f",Rd,Pd,Yd);
                        system("rosnode kill /camera/realsense2_camera_manager");
                        system("rosnode kill /camera/realsense2_camera");
                        //system("rosrun beacon_cam ros_detection_yolov4_async.py");
		}
		
	}

	float caculate_median(float a[])
	{
		int n = sizeof(&a)/sizeof(a[0]);

	        QuickSort(a, 0, n - 1);

		if(n % 2 !=0)
		{
			return a[n / 2];
		}
		else
		{
			return (a[n / 2] + a[n / 2 - 1]) / 2;
		}
	}
	
    float QuickSortOnce(float a[], int low, int high)
	{
		// ???????????????????????????
		float pivot = a[low];
		int i = low, j = high;

		while (i < j)
		{
			// ?????????????????????????????????pivot????????????
			while (a[j] >= pivot && i < j)
			{
				j--;
			}

			// ???????????????j??????????????????????????????????????????pivot????????????
			// ???????????????
			a[i] = a[j];

			// ?????????????????????????????????pivot????????????
			while (a[i] <= pivot && i < j)
			{
				i++;
			}

			// ???????????????i??????????????????????????????????????????pivot????????????
			// ???????????????
			a[j] = a[i];
		}

		// ??????while??????,????????????,?????????i=j????????????
		// i??????j???????????????????????????????????????????????????????????????????????????????????????
		a[i] = pivot;

		return i;
	}

	void QuickSort(float a[], int low, int high)
	{
		if (low >= high)
		{
			return;
		}

		float pivot = QuickSortOnce(a, low, high);

		// ?????????????????????????????????
		QuickSort(a, low, pivot - 1);

		// ?????????????????????????????????
		QuickSort(a, pivot + 1, high);
	}
};



int main(int argc, char **argv) {

	ros::init(argc, argv, "tf_broadcaster");

	

        rpy_calib server;

        time_t start = time(0); 
        time_t last_time = time(0);
	while(last_time - start <= 5)
	{
		ros::spinOnce();
		last_time = time(0);
	}

        while(server.nh_check())
	{
		server.send_tf();
	}
}

