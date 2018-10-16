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
    
    // Topics Init
    ros::init(_argc, _argv, "docker_pid");
    ros::NodeHandle nh;

    ros::ServiceClient s_takeoff = nh.serviceClient<uav_abstraction_layer::TakeOff>("/uav_1/ual/take_off");
    ros::ServiceClient s_go_to_waypoint = nh.serviceClient<uav_abstraction_layer::GoToWaypoint>("/uav_1/ual/go_to_waypoint");
    ros::ServiceClient s_land = nh.serviceClient<uav_abstraction_layer::Land>("/uav_1/ual/land");

    ros::Subscriber sub2 = nh.subscribe("/pipe_pose", 10, Callback);
    ros::AsyncSpinner spinner(4);
    spinner.start();

    uav_abstraction_layer::TakeOff takeoff;
    float height;

    uav_abstraction_layer::GoToWaypoint go_to_waypoint;
    float x, y, z;

    uav_abstraction_layer::Land land;
    float rx, ry, rz;

    std::thread zRefThread(
		[&]() {
        char key;
        for (;;)
        {

            std::cout << "Land:[l], Takeoff:[t],  Go_waypoint:[g], PID:[p], Exit:[e]: ";
            std::cin >> key;

            if (key == 'l')
            {
                s_land.call(land);
            }
            if (key == 't')
            {
                std::cout << "Height: ";
                std::cin >> height;
                takeoff.request.height = height;
                takeoff.request.blocking = true;
                s_takeoff.call(takeoff);
            }
            if (key == 'g')
            {
                std::cout << "x: ";
                std::cin >> x;
                std::cout << "y: ";
                std::cin >> y;
                std::cout << "z: ";
                std::cin >> z;
                go_to_waypoint.request.waypoint.header.frame_id = "";
                go_to_waypoint.request.waypoint.pose.position.x = x;
                go_to_waypoint.request.waypoint.pose.position.y = y;
                go_to_waypoint.request.waypoint.pose.position.z = z;
                go_to_waypoint.request.waypoint.pose.orientation.x = 0;
                go_to_waypoint.request.waypoint.pose.orientation.y = 0;
                go_to_waypoint.request.waypoint.pose.orientation.z = 0;
                go_to_waypoint.request.blocking = true;
                s_go_to_waypoint.call(go_to_waypoint);
            }
            if (key == 'p')
            {

                std::cout << "x reference: ";
                std::cin >> rx;
                std::cout << "y reference: ";
                std::cin >> ry;
                std::cout << "z reference: ";
                std::cin >> rz;

                PID px(1, 0.0, 0., -0.5, 0.5, -20, 20);
                PID py(1, 0.0, 0., -0.5, 0.5, -20, 20);
                PID pz(1, 0.0, 0., -0.5, 0.5, -20, 20);

                px.reference(rx);
                py.reference(ry);
                pz.reference(rz);

                px.enableRosInterface("/mav_controller/pid_x");
                py.enableRosInterface("/mav_controller/pid_y");
                pz.enableRosInterface("/mav_controller/pid_z");
            }
            // if (key == 'e')
            // {
            //     break;
            // }
        }
        });

	auto t0 = chrono::steady_clock::now();
	while (ros::ok())
	{

    }


return 0;
}