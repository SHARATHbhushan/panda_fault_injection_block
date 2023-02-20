#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import numpy as np
from std_msgs.msg import Bool

from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState
class firos:


    def __init__(self):
        #noise = np.random.normal(10,1,1)
        #print(noise)
        self.collision_1 = []
        self.i = 0
        self.goal_state_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.collision_callback)
        self.joint_state_publisher = rospy.Publisher("/collision_model", String, queue_size=10)
        
    
    def collision_callback(self,data): 
        object_1 = data.name[1]
        object_2 = data.name[2]
        object_3 = data.name[3]
        object_4 = data.name[4]
        object_5 = data.name[5]
        object_6 = data.name[6]
        object_7 = data.name[7]
        object_8 = data.name[8]
        object_9 = data.name[9]
        object_10 = data.name[10]
        object_11 = data.name[11]
        object_12 = data.name[12]
        print(object_12)
        self.collision_1.append(data.pose[12].position)
 

        """  
        collision_x_2 = data.pose[2].position.x
        collision_x_3 = data.pose[3].position.x
        collision_x_4 = data.pose[4].position.x
        collision_x_5 = data.pose[5].position.x
        collision_x_6 = data.pose[6].position.x
        collision_x_7 = data.pose[7].position.x
        collision_x_8 = data.pose[8].position.x
        collision_X_9 = data.pose[9].position.x 
        """
        
        if self.collision_1[self.i] != self.collision_1[self.i - 1]:
            print("collision detected")
        self.i = self.i + 1

if __name__ == '__main__':
    rospy.init_node('gazebo_collision')
    firos()
    rospy.spin()





