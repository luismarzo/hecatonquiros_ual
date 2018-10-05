# grvc-ual MODIFICATION

In order to use it:

    Start  demo:
        roslaunch uav_abstraction_layer test_server.launch
        
    Move the arms (for example arm_2):
        rostopic pub -1 /aeroarms/right/arm_2_microservo_joint_position_controller/command std_msgs/Float64 "data: -0.8"
       
    Activate the docker plugin:
        rosservice call /Set_activation "activation: 1"
        


