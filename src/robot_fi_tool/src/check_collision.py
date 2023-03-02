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
        self.person = []
        self.platform = []
        self.tangram_1 = []
        self.tangram_2 = []
        self.tangram_3 = []
        self.tangram_4 = []
        self.tangram_5 = []
        self.tangram_6 = []
        self.tangram_7 = []
        self.flat_back = []
        self.flat_right = []
        self.i = 0
        self.goal_state_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.collision_callback)
        self.collision_pub = rospy.Publisher("/collision_model", String, queue_size=10)
        self.person_state = False
        self.platform_state = False
        self.tangram_1_state = False
        self.tangram_2_state = False
        self.tangram_3_state = False
        self.tangram_4_state = False
        self.tangram_5_state = False
        self.tangram_6_state = False
        self.tangram_7_state = False
        self.flat_back_state = False
        self.flat_right_state = False

    
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
        #object_12 = data.name[12]
        #print(object_1)
        
        self.person.append(data.pose[12].position)
        self.platform.append(data.pose[1].position)
        self.flat_back.append(data.pose[9].position)
        self.flat_right.append(data.pose[10].position)
        self.tangram_1.append(data.twist[2].linear.x)
        self.tangram_2.append(data.twist[3].linear.x)
        self.tangram_3.append(data.twist[4].linear.x)
        self.tangram_4.append(data.twist[5].linear.x)
        self.tangram_5.append(data.twist[6].linear.x)
        self.tangram_6.append(data.twist[7].linear.x)
        self.tangram_7.append(data.twist[8].linear.x)

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
        if abs(self.tangram_1[self.i]) > 10:
            if self.tangram_1_state == False:
                print("tangram_1 fall detected")

                self.tangram_1_state = True
        
        if abs(self.tangram_2[self.i]) > 10:
            if self.tangram_2_state == False:
                print("tangram_2 fall detected")
                self.tangram_2_state = True
        
        if abs(self.tangram_3[self.i]) > 10:
            if self.tangram_3_state == False:
                print("tangram_3 fall detected")
                self.tangram_3_state = True
        
        if abs(self.tangram_4[self.i]) > 10:
            if self.tangram_4_state == False:
                print("tangram_4 fall detected")
                self.tangram_4_state = True

        if abs(self.tangram_5[self.i]) > 10:
            if self.tangram_5_state == False:
                print("tangram_5 fall detected")
                self.tangram_5_state = True

        if abs(self.tangram_6[self.i]) > 10:
            if self.tangram_6_state == False:
                print("tangram_6 fall detected")
                self.tangram_6_state = True
        
        if abs(self.tangram_7[self.i]) > 10:
            if self.tangram_7_state == False:
                print("tangram_7 fall detected")
                self.tangram_7_state = True
        

        if self.person[self.i] != self.person[self.i - 1]:
            if self.person_state == False:
                print("human collision detected")
                self.person_state = True

        if self.platform[self.i] != self.platform[self.i - 1]:
            if self.platform_state == False:
                print("platform collision detected")
                self.platform_state = True

 
        if self.flat_back[self.i] != self.flat_back[self.i - 1]:
            if self.flat_back_state == False:
                print("flat_back collision detected")
                self.flat_back_state = True


        if self.flat_right[self.i] != self.flat_right[self.i - 1]:
            if self.flat_right_state == False:
                print("flat_right collision detected")
                self.flat_right_state = True



        self.i = self.i + 1
        



if __name__ == '__main__':
    rospy.init_node('gazebo_collision')
    firos()
    rospy.spin()





