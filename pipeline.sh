#!/bin/bash

for ((i=0; i<50; i=i++));
do
    gnome-terminal -t "start_gazebo" -- bash -c "source devel/setup.bash;roslaunch panda_gazebo put_robot_in_world.launch; exec bash"
    sleep 5s

    gnome-terminal -t "start_recording_rosout" -- bash -c "source devel/setup.bash;rosbag record -o ./data/rosout.bag /rosout"
    sleep 2s

    gnome-terminal -t "start_recording_states" -- bash -c "source devel/setup.bash;rosbag record -o ./data/faulty_joint_states.bag /joint_states"
    sleep 1s

    gnome-terminal -t "start_recording_joint_states_real" -- bash -c "source devel/setup.bash;rosbag record -o ./data/real_joint_states.bag /joint_states_fake"
    sleep 1s

    gnome-terminal -t "start_recording_joint_states_faulty" -- bash -c "source devel/setup.bash;rosbag record -o ./data/faulty_robot_state.bag /pose_state"
    sleep 1s

    gnome-terminal -t "start_recording_fault_msg" -- bash -c "source devel/setup.bash;rosbag record -o ./data/faults.bag /fault_msg"
    sleep 1s

    gnome-terminal -t "start_injecting" -- bash -c "source devel/setup.bash;roslaunch robot_fi_tool rand_fault_injector.launch; exec bash"
    sleep 10s

    gnome-terminal -t "start_monitoring_collision" -- bash -c "source devel/setup.bash;rosrun robot_fi_tool check_collision.py; exec bash"
    sleep 2s

    gnome-terminal -t "start_recording_collision" -- bash -c "source devel/setup.bash;rosbag record -o ./data/check_collision.bag /collision_model"
    sleep 1s

    gnome-terminal -t "start_recording_fault_flag" -- bash -c "source devel/setup.bash;rosbag record -o ./data/fault_flag.bag /fault_flag"
    sleep 2s

    gnome-terminal -t "start_playing" -- bash -c "source devel/setup.bash;rosrun moveit_tutorials pick_place_tutorial; exec bash"
    sleep 600s    

    gnome-terminal -t "end_everything" -- bash -c "ps aux | grep ros | awk '{print \$2}' | xargs kill -9;ps aux | grep rviz | awk '{print \$2}' | xargs kill -9;ps aux | grep test.sh | awk '{print \$2}'| xargs kill -9;exec bash"
    # gnome-terminal -t "end_everything" -- bash -c "ps aux | grep ros | awk '{print \$2}'; exec bash"
    sleep 60s

done


