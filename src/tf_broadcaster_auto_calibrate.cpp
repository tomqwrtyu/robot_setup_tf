
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

#include "sensor_msgs/Imu.h"

using namespace std;

#include "cmath"
#include "ctime"
#include "bits/stdc++.h"

class rpy_calib {
	ros::NodeHandle n;
	ros::Subscriber sub;
	tf::Quaternion rotation;
    tf::TransformBroadcaster broadcaster;
	int calib_count;
	bool is_calibrated;
	public:

	    double R,Rd;
	    double P,Pd;
	    double Y,Yd;
		double R_array[25];
		double P_array[25];
		double Y_array[25];
            time_t last = time(0);

	    double accel_x;
	    double accel_y;
	    double accel_z;
	    double x ,y ,z, dx, dy, dz;


	rpy_calib()
        {
	        sub = n.subscribe("camera/accel/sample", 1000, &rpy_calib::accel_callback,this);
       	    x = 1000.0;
       	    y = 0.0;
       	    z = 700.0;
       	    dx = -3.35;
       	    dy = 0.0;
       	    dz = 0.0;
			calib_count = 0;
			is_calibrated = False;
			
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
        if(time(0) - last >= 1)
		{
            Rd = 180 * R / M_PI;
            Pd = 180 * P / M_PI;
            Yd = 180 * Y / M_PI;
            ROS_INFO("R: %.2f,P: %.2f,Y: %.2f",Rd,Pd,Yd);
            last = time(0);
        }
		calibration_count();
        rotation.setRPY(R,P,0);
        broadcaster.sendTransform(
	            tf::StampedTransform(
				tf::Transform(rotation, tf::Vector3( x+dx, y+dy, z+dz)),
				ros::Time::now(),"base_link", "base_Camera"));
    }
	void sendtf()
	{
		if(is_calibrated)
		{
			broadcaster.sendTransform(
	            tf::StampedTransform(
				tf::Transform(rotation, tf::Vector3( x+dx, y+dy, z+dz)),
				ros::Time::now(),"base_link", "base_Camera"));
		}
	}
		
	void calibration_count()
	{
		if(calib_count <= 25)
		{
			calib_count += 1;
		}
		else
		{
			is_calibrated = True;
			R = caculate_median(R_array);
			P = caculate_median(P_array);
			Y = caculate_median(Y_array);
		}
		
	}
	
	double caculate_median(double a[])
	{
		n = sizeof a/sizeof a[0]
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
	
    double QuickSortOnce(double a[], int low, int high)
	{
		// 將首元素作為樞軸。
		double pivot = a[low];
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

	void QuickSort(double a[], int low, int high)
	{
		if (low >= high)
		{
			return;
		}

		double pivot = QuickSortOnce(a, low, high);

		// 對樞軸的左端進行排序。
		QuickSort(a, low, pivot - 1);

		// 對樞軸的右端進行排序。
		QuickSort(a, pivot + 1, high);
	}
};



int main(int argc, char **argv) {

	ros::init(argc, argv, "robot_tf_publisher");

	

        rpy_calib server;


        server.sendtf();


        ros::spin();

}

