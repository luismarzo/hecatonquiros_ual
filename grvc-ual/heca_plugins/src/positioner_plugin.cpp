#include "boost/bind.hpp"
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "geometry_msgs/PoseStamped.h"
#include "stdio.h"
#include <fstream>

#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/ContactManager.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/Link.hh"
#include <gazebo/physics/Inertial.hh>

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include "heca_plugins/Set_position.h"
#include "heca_plugins/Set_activation.h"
#include <iostream>
#include <iomanip>
//#include <LinkConfig.hh>

namespace gazebo
{
class positioner_plugin : public ModelPlugin
{
  public:
	void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
	{

		this->model = _parent;
		this->world = this->model->GetWorld();
		this->model_uav = this->world->GetModel("aeroarms_hecatonquiros_positioner_gripper_1");
		this->number = this->world->GetModelCount();
		this->nombres = this->world->GetModels();
		this->nombre_mundo = this->world->GetName();
		static_position = model_uav->GetLink("right/Positioner_Fija_BASE")->GetWorldPose();
		position_docker_begin = model_uav->GetLink("right/Positioner_HAND")->GetWorldPose();
		//   this->jointR1_ = this->model->GetJoint("r1");
		this->link_nombres = this->model_uav->GetLinks();
		this->link_positioner = this->model_uav->GetLink("right/Positioner_Fija_BASE");
		//   this->link_positioner->GetInertial()->SetMass(4); // Cambia la masa a 800000
		//   this ->link_positioner ->UpdateMass();

		// archivo.open("Debugador_positionernuevo.txt");
		// archivo << "\n"
		// 		<< this->model_uav->GetName()
		// 		<< "\t" << model->GetName()
		// 		<< "\t" << this->number
		// 		<< "\t" << this->link_nombres[1]->GetName()
		// 		<< std::endl;

		//Server
		if (!ros::isInitialized())
		{
			ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
							 << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
			return;
		}

		ROS_INFO("\n\nPluging activated!\n");
		ros::NodeHandle n;
		service = n.advertiseService("Set_position", &positioner_plugin::Model_pose, this);
		autoservice = n.advertiseService("Set_autoposition", &positioner_plugin::Model_autopose, this);
		activationservice = n.advertiseService("Set_activation", &positioner_plugin::activation, this);
		pub = n.advertise<geometry_msgs::PoseStamped>("/pipe_pose", 1000);
		pipe_msg.pose.position.x = 0;
		pipe_msg.pose.position.y = 0;
		pipe_msg.pose.orientation.z = 0;

		pub.publish(pipe_msg);

		ros::spinOnce();
		ROS_INFO("Ready.");
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&positioner_plugin::OnUpdate, this, _1));
	}

  public:
	void OnUpdate(const common::UpdateInfo &_info)
	{

		position_docker_begin = model_uav->GetLink("base_link")->GetWorldPose();
		relative_position_docker = position_docker_begin - static_position;
		pipe_msg.pose.position.x = relative_position_docker.pos.x;
		pipe_msg.pose.position.y = relative_position_docker.pos.y;
		pipe_msg.pose.orientation.z = relative_position_docker.rot.z;

		pub.publish(pipe_msg);

		if (set_activation == 1)
		{
			//mover el dron entero
			//math::Pose staticPose(posx,posy,posz,roll,pitch,yaw);
			//this->model_uav->SetWorldPose(staticPose);
			//Setlinkstatic
			math::Pose staticPose(static_position);
			this->link_positioner->SetWorldPose(staticPose);
		}
		else
		{
			static_position = model_uav->GetLink("right/Positioner_Fija_BASE")->GetWorldPose();

			//ros::Rate loop_rate(10);
			//ros::spinOnce();
			//loop_rate.sleep();
		}
	}

	bool Model_pose(heca_plugins::Set_position::Request &req,
					heca_plugins::Set_position::Response &res)
	{
		posx = res.resu1 = req.x;
		posy = res.resu2 = req.y;
		posz = res.resu3 = req.z;
		roll = res.resu4 = req.roll;
		pitch = res.resu5 = req.pitch;
		yaw = res.resu6 = req.yaw;
		//ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
		ROS_INFO("sending back response: [%ld,%ld,%ld,%ld,%ld,%ld]", (long int)res.resu1,
				 (long int)res.resu2,
				 (long int)res.resu3,
				 (long int)res.resu4,
				 (long int)res.resu5,
				 (long int)res.resu6);

		return true;
	}

	bool Model_autopose(heca_plugins::Set_position::Request &req,
						heca_plugins::Set_position::Response &res)
	{

		res.resu1 = 10;
		res.resu2 = 20;
		res.resu3 = 30;
		res.resu4 = 40;
		res.resu5 = 50;
		res.resu6 = 60;
		ROS_INFO("sending auto back response: [%ld,%ld,%ld,%ld,%ld,%ld]", (long int)res.resu1,
				 (long int)res.resu2,
				 (long int)res.resu3,
				 (long int)res.resu4,
				 (long int)res.resu5,
				 (long int)res.resu6);
		return true;
	}

	bool activation(heca_plugins::Set_activation::Request &req,
					heca_plugins::Set_activation::Response &res)
	{
		set_activation = res.resu = req.activation;
		ROS_INFO("request: activation=%ld, ", (long int)req.activation);
		ROS_INFO("sending back response: [%ld]", (long int)res.resu);
		return true;
	}

  private:
	physics::ModelPtr model;
	physics::ModelPtr model_uav;
	physics::WorldPtr world;
	event::ConnectionPtr updateConnection;
	std::ofstream archivo;
	physics::JointPtr jointR1_;
	physics::LinkPtr link_positioner;
	physics::InertialPtr mass;
	ignition::math::Pose3d Pos;
	ros::ServiceServer service;
	ros::ServiceServer autoservice;
	ros::ServiceServer activationservice;
	ros::Publisher pub;
	long int posx = 0, posy = 0, posz = 0, roll = 0, pitch = 0, yaw = 0;
	unsigned int number;
	physics::Model_V nombres;
	physics::Link_V link_nombres;
	std::string nombre_mundo;
	long int set_activation = 0;
	math::Pose static_position;
	math::Pose position_docker_begin;
	math::Pose relative_position_docker;
	geometry_msgs::PoseStamped pipe_msg;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(positioner_plugin)
} // namespace gazebo
