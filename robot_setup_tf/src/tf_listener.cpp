#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/String.h>
#include "std_msgs/Float32MultiArray.h"
#include <tf/transform_listener.h>

/*
class CPTransform {
public:
    int transformPoint(const tf::TransformListener& listener);
    void CPRCallback(const std_msgs::Float32MultiArray::ConstPtr& pts);
private:
    float cam_p[3] = {0.0,0.0,0.0};
    float transformed_p[3] = {};
    ros::NodeHandle n;
    tf::TransformListener listener(ros::Duration(0.01));
    ros::Publisher TransformedPointsPub = n.advertise<std_msgs::Float32MultiArray>("T.P.Transmitter", 1000);
    ros::Subscriber CameraPointsSub = n.subscribe("C.P.Reciever", 1000, CPTransform::CPRCallback);
    ros::Timer timer = n.createTimer(ros::Duration(0.01), boost::bind(&CPTransform::transformPoint, boost::ref(listener)));
};

int CPTransform::transformPoint(const tf::TransformListener& listener)
{
  geometry_msgs::PointStamped laser_point;
  laser_point.header.frame_id = "base_Camera";
  //we'll just use the most recent transform available for our simple example
  laser_point.header.stamp = ros::Time();
  //just an arbitrary point in space
  float zero_array[3] = {0.0,0.0,0.0};
  if (std::equal(std::begin(cam_p), std::end(cam_p), std::begin(zero_array)))
  {
      return 0
  }
  laser_point.point.x = cam_p[0];
  laser_point.point.y = cam_p[1];
  laser_point.point.z = cam_p[2];
  try{
    geometry_msgs::PointStamped base_point;
    listener.transformPoint("base_link", laser_point, base_point);
    float tpts[3] = {base_point.point.x,base_point.point.y,base_point.point.z};
    TransformedPointsPub.publish(tpts);
    ROS_INFO("base_camera: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
        laser_point.point.x, laser_point.point.y, laser_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  }

  catch(tf::TransformException& ex){

    ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());

  }
  return 0
}

void CPTransform::CPRCallback(const std_msgs::Float32MultiArray::ConstPtr& pts)
{
    cam_p[0] = pts->data.at(0);
    cam_p[1] = pts->data.at(1);
    cam_p[2] = pts->data.at(2);
}

int main(int argc, char** argv){

  //we'll transform a point once every second

  ros::init(argc, argv, "robot_tf_listener");

  CPTransform transformer;

  ros::spin();

}
*/

    
