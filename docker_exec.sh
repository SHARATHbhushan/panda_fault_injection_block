gnome-terminal -t "start_gazebo" -- bash -c "source devel/setup.bash;roslaunch panda_gazebo put_robot_in_world.launch; exec bash"
sleep 10s

gnome-terminal -t "start_injecting" -- bash -c "source devel/setup.bash;roslaunch robot_fi_tool fault_module.launch; exec bash"
sleep 5s

gnome-terminal -t "start_gui" -- bash -c "source devel/setup.bash;rosrun robot_fi_tool fib_gui_v2.py; exec bash"
sleep 5s