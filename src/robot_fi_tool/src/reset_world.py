#! /usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import numpy as np
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from robot_fi_tool.msg import faultmsg
import random
from std_srvs.srv import Empty


class reset_world_init:
    def __init__(self):
        self.sub = rospy.Subscriber("iterations", Int32, self.callback)
        self.wait = rospy.wait_for_service('/gazebo/reset_world')
        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.wait2 = rospy.wait_for_service('/gazebo/reset_simulation')
        self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.goal_state_subscriber = rospy.Subscriber("goal_msg", Bool, self.goal_callback)
        self.state_subscriber = rospy.Subscriber("pose_state", Int32,self.state_callback)


    def goal_callback(self, data):
        self.goal = data.data
    
    def state_callback(self, data):
        self.state = data.data

    def callback(self, data):
        if data.data == 6:
            if self.state == 9:
                if self.goal == True:
                    rospy.sleep(10)
                    self.reset_world()
                    self.reset_simulation()
        else:
            pass


if __name__ == '__main__':
    rospy.init_node('reset_world_init')
    reset_world_init()
    rospy.spin()
        


