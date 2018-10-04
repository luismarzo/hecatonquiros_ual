#include "ros/ros.h"
#include "heca_plugins/Set_position.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Set_position_client");
  if (argc != 7)
  {
    ROS_INFO("usage: Give me the position [x,y,z,roll,pitch,yaw]");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<heca_plugins::Set_position>("Set_position");
  heca_plugins::Set_position srv;
  srv.request.x = atoll(argv[1]);
  srv.request.y = atoll(argv[2]);
  srv.request.z = atoll(argv[3]);
  srv.request.roll = atoll(argv[4]);
  srv.request.pitch = atoll(argv[5]);
  srv.request.yaw = atoll(argv[6]);
  if (client.call(srv))
  {
    ROS_INFO("Pos: [%ld,%ld,%ld,%ld,%ld,%ld]", (long int)srv.response.resu1,
						(long int)srv.response.resu2,
						(long int)srv.response.resu3,
						(long int)srv.response.resu4,
						(long int)srv.response.resu5,
						(long int)srv.response.resu6);
  }
  else
  {
    ROS_ERROR("Failed to call service ");
    return 1;
  }

  return 0;
}
