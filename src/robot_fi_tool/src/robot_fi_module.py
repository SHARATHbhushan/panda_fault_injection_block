#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import numpy as np
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from robot_fi_tool.msg import faultmsg

'''
desired_state : 
  1-hover_pose
  2-pick_pose_down
  3-pick
  4-pick_pose_up
  5-hover_place_pose
  6-place_pose_down
  7-open_Hand
  8-place_pose_up
  9-init_pose
'''
'''
desire_joint 
  1 - panda_joint1
  2 - panda_joint2
  3 - panda_joint3
  4 - panda_joint4
  5 - panda_joint5
  6 - panda_joint6
  7 - panda_joint7
  8 - panda_finger_joint1
  9 - panda_finger_joint2
''' 
'''
desired_fault
  1 - noise
  2 - stuck_at
  3 - package_drop
  4 - offset
'''


class firos:


    def __init__(self):
        #noise = np.random.normal(10,1,1)
        #print(noise)
        self.goal_state_subscriber = rospy.Subscriber("goal_msg", Bool, self.goal_callback)
        self.joint_state_publisher = rospy.Publisher("joint_states", JointState, queue_size=10)
        self.fault_status = rospy.Publisher("fault_status", Bool, queue_size=10)
        self.joint_state_fake_subscriber = rospy.Subscriber("joint_states_fake", JointState, self.callback)
        self.state_subscriber = rospy.Subscriber("pose_state", Int32,self.state_callback)
        self.fault_msg_subscriber = rospy.Subscriber("fault_msg", faultmsg, self.fault_callback)
        self.publish_time = rospy.Publisher("time_to_inject", Int32, queue_size=10)
        self.goal = False
        self.desired_state = 0
        self.desired_joint = 0
        self.desired_fault = 0
        self.fault_val = 0
        self.joint_val_list = []
        self.fault_val_list = []
        self.state = 0
        self.fault_status.publish(False)

    def fault_callback(self,fault_msg):
        self.desired_state = fault_msg.pose
        self.desired_joint = fault_msg.joint
        self.desired_fault = fault_msg.fault
        self.desired_time = fault_msg.time
        self.desired_time_label = fault_msg.time_label
        self.desired_offset = fault_msg.offset
        self.fault_val_list = []
        #self.publish_time.publish(self.desired_time)
        self.fault_status.publish(True)


    def state_callback(self,state):
        self.state = state.data
        self.fault_val_list = []
        self.fault_status.publish(False)
        

    def goal_callback(self,goal):   
        self.goal = goal.data

    def callback(self,data):       
        self.joint_data = data      
        self.list_joint_data = list(self.joint_data.position)  
        #print(data.position[1])
        self.joint = self.desired_joint
        if self.desired_fault == 0:
            self.fault_val = 0
        if self.desired_fault == 1:
            self.fault_val = np.random.normal(1,1,1)[0]
        if self.desired_fault == 2:
            self.fault_val_list.append(self.list_joint_data[self.joint]) #to be defined
        if self.desired_fault == 3:
            self.fault_val = -self.list_joint_data[self.joint]
        if self.desired_fault == 4:
            self.fault_val = 5

        if self.goal == True:
            if self.state == self.desired_state:
                #print(self.list_joint_data)
                self.fault_status.publish(True)
                if self.desired_fault == 2:
                    self.list_joint_data[self.joint] = self.fault_val_list[0]
                else:
                    self.list_joint_data[self.joint] = self.list_joint_data[self.joint] + self.fault_val
                #print(self.list_joint_data[self.joint])
                
                #print("noise error injected")
                self.joint_data.position = tuple(self.list_joint_data)
                #self.joint_data.position[1] = error_data
                #self.joint_state_publisher.publish(self.joint_data)
                #self.joint_state_publisher.publish(data)
                self.goal = False
        #print(self.joint_data)
        self.joint_state_publisher.publish(self.joint_data)
        #self.goal = True

if __name__ == '__main__':
    rospy.init_node('FIB')
    firos()
    rospy.spin()