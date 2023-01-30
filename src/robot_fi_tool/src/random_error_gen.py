import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import numpy as np
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from robot_fi_tool.msg import faultmsg
import random

class firos_rand:


    def __init__(self):
        #noise = np.random.normal(10,1,1)
        #print(noise)
        self.fault_list = [" ","noise", "stuck_at", "package_drop", "offset"]
        self.joint_list = [" ", "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7", "panda_finger_joint1", "panda_finger_joint2"]
        self.state_list = [" ", "hover_pose", "pick_pose_down", "pick_pose_down", "pick", "pick_pose_up", "hover_place_pose", "place_pose_down", "open_Hand", "place_pose_up", "init_pose"]
        self.goal_state_subscriber = rospy.Subscriber("goal_msg", Bool, self.goal_callback)
        self.goal_state_subscriber = rospy.Subscriber("iterations", Int32, self.iter_callback)
        self.random_fault_publisher = rospy.Publisher("fault_msg", faultmsg, queue_size=10)
        self.state_subscriber = rospy.Subscriber("pose_state", Int32,self.state_callback)
        self.goal = False
        self.desired_state = 0
        self.desired_joint = 0
        self.desired_fault = 0



    def state_callback(self,state):
        self.state = state.data
        

    def goal_callback(self,goal):   
        self.goal = goal.data
        #print(rand_msg.fault)
        #print(rand_msg.joint)
        #print(rand_msg.pose)
        
    def iter_callback(self,data):
        rand_msg = faultmsg()
        self.desired_joint = random.randint(0, 8)
        self.desired_state = random.randint(1, 9)
        self.desired_fault = random.randint(1, 4)
        rand_msg.fault = self.desired_fault
        rand_msg.joint = self.desired_joint
        rand_msg.pose = self.desired_state
        print(self.fault_list[self.desired_fault] + " Fault is being injected at state " + self.state_list[self.desired_state] + " in joint " + self.joint_list[self.desired_joint])
        print("fault : ", self.desired_fault)
        print("joint : ", self.desired_joint)
        print("state : ", self.desired_state)
        self.random_fault_publisher.publish(rand_msg)
                

if __name__ == '__main__':
    rospy.init_node('random_fault_gen')
    firos_rand()
    rospy.spin()