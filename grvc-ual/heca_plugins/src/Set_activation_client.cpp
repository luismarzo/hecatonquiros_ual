#include "ros/ros.h"

#include "heca_plugins/Set_activation.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Set_activation_client");
  if (argc != 2)
  {
    ROS_INFO("Error en arguentos de entrada");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<heca_plugins::Set_activation>("Set_activation");
  heca_plugins::Set_activation srv;
  srv.request.activation = atoll(argv[1]);
  if (client.call(srv))
  {
    ROS_INFO("resultado: %ld", (long int)srv.response.resu);
  }
  else
  {
    ROS_ERROR("Failed to call service Set_activation_client");
    return 1;
  }

  return 0;
}
