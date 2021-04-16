
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
		// 將首元素作為樞軸。
		float pivot = a[low];
		int i = low, j = high;

		while (i < j)
		{
			// 從右到左，尋找首個小於pivot的元素。
			while (a[j] >= pivot && i < j)
			{
				j--;
			}

			// 執行到此，j已指向從右端起首個小於或等於pivot的元素。
			// 執行替換。
			a[i] = a[j];

			// 從左到右，尋找首個大於pivot的元素。
			while (a[i] <= pivot && i < j)
			{
				i++;
			}

			// 執行到此，i已指向從左端起首個大於或等於pivot的元素。
			// 執行替換。
			a[j] = a[i];
		}

		// 退出while迴圈,執行至此,必定是i=j的情況。
		// i（或j）指向的即是樞軸的位置，定位該趟排序的樞軸並將該位置返回。
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

		// 對樞軸的左端進行排序。
		QuickSort(a, low, pivot - 1);

		// 對樞軸的右端進行排序。
		QuickSort(a, pivot + 1, high);
	}
};



int main(int argc, char **argv) {

	ros::init(argc, argv, "robot_tf_publisher");

	

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

