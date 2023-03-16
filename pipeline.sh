#!/bin/bash

for ((i=0; i<50; i=i++));
do
    gnome-terminal -t "start_gazebo" -- bash -c "source devel/setup.bash;roslaunch panda_gazebo put_robot_in_world.launch; exec bash"
    sleep 5s

    gnome-terminal -t "start_recording_rosout" -- bash -c "source devel/setup.bash;rosbag record -o ./data_v2/rosout.bag /rosout __name:=ros_out"
    sleep 2s

    gnome-terminal -t "start_recording_states" -- bash -c "source devel/setup.bash;rosbag record -o ./data_v2/faulty_joint_states.bag /joint_states __name:=joint_state_r"
    sleep 1s

    gnome-terminal -t "start_recording_joint_states_real" -- bash -c "source devel/setup.bash;rosbag record -o ./data_v2/real_joint_states.bag /joint_states_fake __name:=joint_state_f"
    sleep 1s

    gnome-terminal -t "start_recording_joint_states_faulty" -- bash -c "source devel/setup.bash;rosbag record -o ./data_v2/faulty_robot_state.bag /pose_state __name:=pose_state_r"
    sleep 1s

    gnome-terminal -t "start_recording_fault_msg" -- bash -c "source devel/setup.bash;rosbag record -o ./data_v2/faults.bag /fault_msg __name:=fault_msg_r"
    sleep 1s

    gnome-terminal -t "start_injecting" -- bash -c "source devel/setup.bash;roslaunch robot_fi_tool rand_fault_injector.launch; exec bash"
    sleep 10s

    gnome-terminal -t "start_monitoring_collision" -- bash -c "source devel/setup.bash;rosrun robot_fi_tool check_collision.py; exec bash"
    sleep 2s

    gnome-terminal -t "start_recording_collision" -- bash -c "source devel/setup.bash;rosbag record -o ./data_v2/check_collision.bag /collision_model __name:=colllision_r"
    sleep 1s

    gnome-terminal -t "start_recording_fault_flag" -- bash -c "source devel/setup.bash;rosbag record -o ./data_v2/fault_flag.bag /fault_flag __name:=fault_flag_r"
    sleep 2s

    gnome-terminal -t "start_recording_fault_state" -- bash -c "source devel/setup.bash;rosbag record -o ./data_v2/fault_state.bag /fault_state __name:=fault_state_r"
    sleep 2s

    gnome-terminal -t "start_recording_fault_effect" -- bash -c "source devel/setup.bash;rosbag record -o ./data_v2/fault_effect.bag /fault_effect __name:=fault_effect_r"
    sleep 2s

    gnome-terminal -t "start_playing" -- bash -c "source devel/setup.bash;rosrun moveit_tutorials pick_place_tutorial; exec bash"
    sleep 600s    

    gnome-terminal -t "kill_node_1" -- bash -c "source devel/setup.bash;rosnode kill /fault_flag_r;rosnode kill /fault_msg_r;rosnode kill /pose_state_r;rosnode kill /joint_state_f;rosnode kill /joint_state_r;rosnode kill /ros_out;rosnode kill /colllision_r;rosnode kill fault_state_r;rosnode kill fault_effect_r;exec bash"
    sleep 20s

    gnome-terminal -t "end_everything" -- bash -c "ps aux | grep ros | awk '{print \$2}' | xargs kill -9;ps aux | grep rviz | awk '{print \$2}' | xargs kill -9;ps aux | grep test.sh | awk '{print \$2}'| xargs kill -9;exec bash"
    # gnome-terminal -t "end_everything" -- bash -c "ps aux | grep ros | awk '{print \$2}'; exec bash"
    sleep 60s

done


