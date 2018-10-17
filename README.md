# grvc-ual MODIFICATION

In order to use it:

Start  demo:
    roslaunch uav_abstraction_layer test_server.launch

Take_off, Go_to_waypoint and call the PID:
    rosrun pid docker_pid

    (The entire length of the positioner is x: 0 y: 0 z: 0.4, so a good reference would be x: 0 y: 0 z: 0.3 for example) 
    
Move the arms (for example arm_2):
    rostopic pub -1 /aeroarms/right/arm_2_microservo_joint_position_controller/command std_msgs/Float64 "data: -0.8"
   
Activate the docker plugin:
    rosservice call /Set_activation "activation: 1"

Now the positioner is entirely free, if you want to control one or more joints, go to arm_description/urdf/positioner_gripper.xacro. In the end, discoment the transmissions.

