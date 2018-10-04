//----------------------------------------------------------------------------------------------------------------------
// GRVC UAL
//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
// 
// Copyright (c) 2016 GRVC University of Seville
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------
#include <uav_abstraction_layer/ual.h>
#include <uav_abstraction_layer/GoToWaypointGeo.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_srvs/Empty.h>

using namespace uav_abstraction_layer;

namespace grvc { namespace ual {

UAL::UAL(int _argc, char** _argv) {
    // Start ROS if not initialized
    if (!ros::isInitialized()) {
        // Init ros node
        ros::init(_argc, _argv, "ual");
    }
    this->init();
}

UAL::UAL() {
    // Error if ROS is not initialized
    if (!ros::isInitialized()) {
        // Init ros node
        ROS_ERROR("UAL needs ROS to be initialized. Initialize ROS before creating UAL object or use UAL(int _argc, char** _argv) constructor.");
        exit(0);
    }
    this->init();
}

void UAL::init() {
    // Create backend first of all, inits ros node
    backend_ = Backend::createBackend();
    // Get params
    ros::NodeHandle pnh("~");
    pnh.param<int>("uav_id", robot_id_, 1);
    pnh.param<std::string>("ns_prefix", ns_prefix_, "uav_");

    // Start server if explicitly asked
    std::string server_mode;
    pnh.param<std::string>("ual_server", server_mode, "on");
    // TODO: Consider other modes?
    if (server_mode == "on") {
        server_thread_ = std::thread([this]() {
            std::string ual_ns =  this->ns_prefix_ + std::to_string(this->robot_id_) + "/ual";
            std::string take_off_srv = ual_ns + "/take_off";
            std::string land_srv = ual_ns + "/land";
            std::string go_to_waypoint_srv = ual_ns + "/go_to_waypoint";
            std::string go_to_waypoint_geo_srv = ual_ns + "/go_to_waypoint_geo";
            std::string set_velocity_srv = ual_ns + "/set_velocity";
            std::string recover_from_manual_srv = ual_ns + "/recover_from_manual";
            std::string set_home_srv = ual_ns + "/set_home";
            std::string pose_topic = ual_ns + "/pose";
            std::string velocity_topic = ual_ns + "/velocity";
            std::string state_topic = ual_ns + "/state";

            ros::NodeHandle nh;
            ros::ServiceServer take_off_service =
                nh.advertiseService<TakeOff::Request, TakeOff::Response>(
                take_off_srv,
                [this](TakeOff::Request &req, TakeOff::Response &res) {
                return this->takeOff(req.height, req.blocking);
            });
            ros::ServiceServer land_service =
                nh.advertiseService<Land::Request, Land::Response>(
                land_srv,
                [this](Land::Request &req, Land::Response &res) {
                return this->land(req.blocking);
            });
            ros::ServiceServer go_to_waypoint_service =
                nh.advertiseService<GoToWaypoint::Request, GoToWaypoint::Response>(
                go_to_waypoint_srv,
                [this](GoToWaypoint::Request &req, GoToWaypoint::Response &res) {
                return this->goToWaypoint(req.waypoint, req.blocking);
            });
            ros::ServiceServer go_to_waypoint_geo_service =
                nh.advertiseService<GoToWaypointGeo::Request, GoToWaypointGeo::Response>(
                go_to_waypoint_geo_srv,
                [this](GoToWaypointGeo::Request &req, GoToWaypointGeo::Response &res) {
                return this->goToWaypointGeo(req.waypoint, req.blocking);
            });
            ros::ServiceServer set_velocity_service =
                nh.advertiseService<SetVelocity::Request, SetVelocity::Response>(
                set_velocity_srv,
                [this](SetVelocity::Request &req, SetVelocity::Response &res) {
                return this->setVelocity(req.velocity);
            });
            ros::ServiceServer recover_from_manual_service =
                nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(
                recover_from_manual_srv,
                [this](std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
                return this->recoverFromManual();
            });
            ros::ServiceServer set_home_service =
                nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(
                set_home_srv,
                [this](std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
                return this->setHome();
            });
            ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>(pose_topic, 10);
            ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>(velocity_topic, 10);
            ros::Publisher state_pub = nh.advertise<std_msgs::String>(state_topic, 10);
            static tf2_ros::TransformBroadcaster tf_pub;

            // Publish @ 10Hz
            ros::Rate loop_rate(10);
            while (ros::ok()) {
                pose_pub.publish(this->pose());
                velocity_pub.publish(this->velocity());
                state_pub.publish(this->state());
                tf_pub.sendTransform(this->transform());
                loop_rate.sleep();
            }
        });
    }
}

UAL::~UAL() {
    if (!backend_->isIdle()) { backend_->abort(); }
    if (running_thread_.joinable()) { running_thread_.join(); }
    if (server_thread_.joinable()) { server_thread_.join(); }
}

bool UAL::goToWaypoint(const Waypoint& _wp, bool _blocking) {
    // Check required state
    if (state_ != FLYING) {
        return false;
    }
    // Override any previous FLYING function
    if (!backend_->isIdle()) { backend_->abort(false); }

    if (_blocking) {
        if (!backend_->threadSafeCall(&Backend::goToWaypoint, _wp)) {
            ROS_INFO("Blocking goToWaypoint rejected!");
            return false;
        }
    } else {
        if (running_thread_.joinable()) running_thread_.join();
        // Call function on a thread:
        running_thread_ = std::thread ([this, _wp]() {
            if (!this->backend_->threadSafeCall(&Backend::goToWaypoint, _wp)) {
                ROS_INFO("Non-blocking goToWaypoint rejected!");
            }
        });
    }
    return true;
}
bool UAL::goToWaypointGeo(const WaypointGeo& _wp, bool _blocking) {
    // Check required state
    if (state_ != FLYING) {
        return false;
    }
    // Override any previous FLYING function
    if (!backend_->isIdle()) { backend_->abort(false); }

    if (_blocking) {
        if (!backend_->threadSafeCall(&Backend::goToWaypointGeo, _wp)) {
            ROS_INFO("Blocking goToWaypoint rejected!");
            return false;
        }
    } else {
        if (running_thread_.joinable()) running_thread_.join();
        // Call function on a thread:
        running_thread_ = std::thread ([this, _wp]() {
            if (!this->backend_->threadSafeCall(&Backend::goToWaypointGeo, _wp)) {
                ROS_INFO("Non-blocking goToWaypoint rejected!");
            }
        });
    }
    return true;
}

bool UAL::takeOff(double _height, bool _blocking) {
    // Check required state
    if (state_ != LANDED) {
        return false;
    }
    state_ = TAKING_OFF;

    if (_blocking) {
        if (!backend_->threadSafeCall(&Backend::takeOff, _height)) {
            ROS_INFO("Blocking takeOff rejected!");
            return false;
        }
        state_ = FLYING;
    } else {
        if (running_thread_.joinable()) running_thread_.join();
        // Call function on a thread:
        running_thread_ = std::thread ([this, _height]() {
            if (!this->backend_->threadSafeCall(&Backend::takeOff, _height)) {
                ROS_INFO("Non-blocking takeOff rejected!");
            }
            this->state_ = FLYING;
        });
    }
    return true;
}

bool UAL::land(bool _blocking) {
    // Check required state
    if (state_ != FLYING) {
        return false;
    }
    // Override any previous FLYING function
    if (!backend_->isIdle()) { backend_->abort(); }
    state_ = LANDING;

    if (_blocking) {
        if (!backend_->threadSafeCall(&Backend::land)) {
            ROS_INFO("Blocking land rejected!");
            return false;
        }
        state_ = LANDED;
    } else {
        if (running_thread_.joinable()) running_thread_.join();
        // Call function on a thread:
        running_thread_ = std::thread ([this]() {
            if (!this->backend_->threadSafeCall(&Backend::land)) {
                ROS_INFO("Non-blocking land rejected!");
            }
            this->state_ = LANDED;
        });
    }
    return true;
}

bool UAL::setVelocity(const Velocity& _vel) {
    // Check required state
    if (state_ != FLYING) {
        return false;
    }
    // Override any previous FLYING function
    if (!backend_->isIdle()) { backend_->abort(); }

    // Function is non-blocking in backend TODO: non-thread-safe-call?
    backend_->threadSafeCall(&Backend::setVelocity, _vel);
    return true;
}

bool UAL::recoverFromManual() {
    // Check required state. TODO: consequences of not checking it!
    // if (state_ != FLYING) {
    //     return false;
    // }
    // Override any previous FLYING function
    if (!backend_->isIdle()) { backend_->abort(); }

    // Direct call! TODO: Check nobody explodes!
    backend_->recoverFromManual();

    // Force state. TODO: ckeck consequences of doing this!
    state_ = FLYING;

    return true;
}

std_msgs::String UAL::state() {
    std_msgs::String output;
    switch (state_) {
        case LANDED:
            output.data = "LANDED";
            break;
        case TAKING_OFF:
            output.data = "TAKING_OFF";
            break;
        case FLYING:
            output.data = "FLYING";
            break;
        case LANDING:
            output.data = "LANDING";
            break;
        default:
            output.data= "unknown";
    }
    return output;
}

bool UAL::setHome() {
    // Check required state
    if (state_ != LANDED) {
        return false;
    }
    backend_->setHome();

    return true;
}

}}	// namespace grvc::ual
