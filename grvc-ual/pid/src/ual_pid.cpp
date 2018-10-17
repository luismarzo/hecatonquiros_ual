#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <stdlib.h>
#include <sensor_msgs/PointCloud2.h>
#include <camera_info_manager/camera_info_manager.h>
#include "PID.h"
#include <chrono>
#include <thread>
#include <image_transport/image_transport.h>
#include <thread>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "ardrone_autonomy/Navdata.h"
#include <uav_abstraction_layer/ual.h>
#include "std_msgs/Float64.h"

#include <thread>

using namespace std;
float linx = 0, liny = 0, linz = 0, angZ = 0;
int velCount, velCount100ms;
float reference_z = 2;

void Callback(const geometry_msgs::PoseStamped &msg)
{
	linx = msg.pose.position.x;
	liny = msg.pose.position.y;
	linz = msg.pose.position.z;
	angZ = msg.pose.orientation.z;
	//std::cout<<"pipe_pose with linx:"<<linx<<", liny:"<<liny<<", angz:"<<angZ<<std::endl;
}

void VelCallback(const geometry_msgs::TwistConstPtr vel)
{
	velCount++;
	velCount100ms++;
}

int main(int _argc, char **_argv)
{

	//INIT UAL
	grvc::ual::UAL ual(_argc, _argv);
	while (!ual.isReady() && ros::ok())
	{
		std::cout << "UAL not ready!" << std::endl;
		sleep(1);
	}

	// waypoint.header.frame_id = "map";
	// waypoint.pose.position.x = x_pos;
	// waypoint.pose.position.y = y_pos;
	// waypoint.pose.position.z = flight_level;
	// waypoint.pose.orientation.x = 0;
	// waypoint.pose.orientation.y = 0;
	// waypoint.pose.orientation.z = 0;
	// waypoint.pose.orientation.w = 1;

	// Topics Init
	ros::init(_argc, _argv, "publish_velocity");
	ros::NodeHandle nh;

	ros::Subscriber sub2 = nh.subscribe("/pipe_pose", 10, Callback);
	ros::AsyncSpinner spinner(4);
	spinner.start();

	PID px(1, 0.0, 0., -0.5, 0.5, -20, 20);
	PID py(1, 0.0, 0., -0.5, 0.5, -20, 20);
	PID pz(1, 0.0, 0., -0.5, 0.5, -20, 20);

	px.reference(0);
	py.reference(0);
	pz.reference(3);

	px.enableRosInterface("/mav_controller/pid_x");
	py.enableRosInterface("/mav_controller/pid_y");
	pz.enableRosInterface("/mav_controller/pid_z");

	bool pid = false;
	double flight_level = 0;
	double x_pos = 0;
	double y_pos = 0;
	grvc::ual::Waypoint waypoint;

	std::thread zRefThread(
		[&]() {
			char key;
			for (;;)
			{
				std::cout << "Land:[l], Takeoff:[t],  Go_waypoint:[g], PID:[p]: ";
				std::cin >> key;

				if (key == 'l')
				{
					ual.land();
				}
				if (key == 't')
				{
					std::cout << "Height: ";
					std::cin >> flight_level;
					ual.takeOff(flight_level);
				}
				if (key == 'r')
				{
					// std::cout << "Z reference:";
					// std::cin >> reference_z;
					// if (reference_z > 0)
					// {
					// 	pz.reference(reference_z);
					// }
				}
				if (key == 'g')
				{
					std::cout << "X position:";
					std::cin >> x_pos;
					std::cout << "Y position:";
					std::cin >> y_pos;
					std::cout << "Z position:";
					std::cin >> flight_level;

					waypoint.header.frame_id = "map";
					waypoint.pose.position.x = x_pos;
					waypoint.pose.position.y = y_pos;
					waypoint.pose.position.z = flight_level;
					waypoint.pose.orientation.x = 0;
					waypoint.pose.orientation.y = 0;
					waypoint.pose.orientation.z = 0;
					waypoint.pose.orientation.w = 1;
					ual.goToWaypoint(waypoint);
				}
			}
		});
	while (1)
	{
	}
	// auto t0 = chrono::steady_clock::now();
	// while (ros::ok())
	// {
	// 	if (pid == true)
	// 	{
	// 		auto t1 = chrono::steady_clock::now();
	// 		auto rosTime = ros::Time::now();

	// 		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	// 		float incT = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() / 1000.0f;
	// 		//std::cout << incT << std::endl;
	// 		t0 = t1;
	// 		if (reference_z > 0)
	// 		{
	// 			float ux = px.update(linx, incT);
	// 			float uy = py.update(liny, incT);
	// 			float uz = pz.update(linz, incT);

	// 			geometry_msgs::TwistStamped msg;
	// 			msg.twist.linear.x = uy;
	// 			msg.twist.linear.y = ux;
	// 			msg.twist.linear.z = uz;
	// 			msg.twist.angular.z = 0;
	// 			msg.twist.angular.x = 0;
	// 			msg.twist.angular.y = 0;

	// 			ual.setVelocity(msg);
	// 		}
	// 	}
	// }
	// return (0);
}