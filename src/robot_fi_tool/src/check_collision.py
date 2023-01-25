#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import numpy as np
from std_msgs.msg import Bool
from gazebo_msgs.msg import ModelStates
class firos:


    def __init__(self):
        #noise = np.random.normal(10,1,1)
        #print(noise)
        self.goal_state_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.collision_callback)
        self.joint_state_publisher = rospy.Publisher("/collision_model", String, queue_size=10)
        
    
    def collision_callback(self,data): 
        object_1 = data.name[1]
        object_2 = data.name[2]
        object_3 = data.name[3]
        object_1 = data.name[4]
        object_2 = data.name[5]
        object_3 = data.name[6]
        object_1 = data.name[7]
        object_2 = data.name[8]
        object_3 = data.name[9]
        collision_1 = data.pose[2].position.x
        #print(collision)

if __name__ == '__main__':
    rospy.init_node('gazebo_collision')
    firos()
    rospy.spin()