#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
//#include <opencv2/aruco.hpp>
//#include <opencv2/opencv.hpp>
#include <sensor_msgs/PointCloud2.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
#include <camera_info_manager/camera_info_manager.h>
#include "PID.h"
#include <chrono>
#include <thread>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <thread>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
 
using namespace cv;
//using namespace pcl;
using namespace std;
cv_bridge::CvImagePtr cv_ptr;
float linx = 0, liny = 0, linz = 0, angZ = 0;

void Callback(const geometry_msgs::PoseStamped &msg)
{
	linx = msg.pose.position.x;
	liny = msg.pose.position.y;
	linz = msg.pose.position.z;
	
}

int main(int _argc, char **_argv)
{
 
  ros::init(_argc, _argv, "publish_velocity");
  ros::NodeHandle nh;
  ros::NodeHandle it;
  image_transport::Subscriber sub = it.subscribe("/ardrone/bottom/image_raw", 1, imageCallback); //subcriptor
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  ros::Publisher posepub = nh.advertise<geometry_msgs::PoseStamped>("/position/pos", 1000);
  ros::Publisher refpub = nh.advertise<geometry_msgs::PoseStamped>("/reference", 1000);
 
  ros::AsyncSpinner spinner(4);
  spinner.start();
  
   float i;
  std::cout << "enter z altitude: ";
  std::cin >> i;
 
  
  
  PID px(0.8, 0.0, 0.02, -5, 5, -20, 20);
  PID py(0.8, 0.0, 0.02, -5, 5, -20, 20);
  PID pz(0.8, 0.0, 0.02, -5, 5, -20, 20);
  px.reference(0);
  py.reference(0);
 
  if (i != 0)
    pz.reference(i);
 
  auto t0 = chrono::steady_clock::now();
  while (ros::ok())
  {
    
    
    auto t1 = chrono::steady_clock::now();
    auto rosTime = ros::Time::now();
    
   sleep(50)
 
      float incT = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() / 1000.0f;
      std::cout << incT << std::endl;
      t0 = t1;
      float ux = px.update(vec_translations[0][0], incT);
      float uy = py.update(vec_translations[0][1], incT);
      float uz = pz.update(vec_translations[0][2], incT);
 
      
      
      geometry_msgs::Twist msg;
      msg.linear.x = -uy;
      msg.linear.z = uz;
      msg.linear.y = ux;
 
      geometry_msgs::PoseStamped msgref;
      msgref.header.stamp = rosTime;
      msgref.pose.position.x = px.reference();
      msgref.pose.position.z = pz.reference();
      msgref.pose.position.y = py.reference();
 
      geometry_msgs::PoseStamped msgpos;
      msgpos.header.stamp = rosTime;
      msgpos.pose.position.x = -vec_translations[0][1];
      msgpos.pose.position.z = vec_translations[0][2];
      msgpos.pose.position.y = vec_translations[0][0];
 
      pub.publish(msg);
      posepub.publish(msgpos);
      refpub.publish(msgref);
      
    }
    
    
  }
  return (0);
}
