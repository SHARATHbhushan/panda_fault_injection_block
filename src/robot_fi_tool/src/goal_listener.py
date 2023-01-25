#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
class goal_listener:
    def __init__(self):
        rospy.Subscriber("goal_state", Bool, self.callback)
        rospy.Subscriber("joint_states", JointState, self.publisher)
        self.pub = rospy.Publisher('goal_msg', Bool, queue_size=10)
        self.rate = rospy.Rate(30) # 10hz
        self.flag = False

        
    def callback(self,data):
        #print(data.data)
        if data.data == False:
            self.flag = False
        if data.data == True:
            self.flag = True
        #self.publisher(data.data,self.flag)



    def publisher(self,data):
        self.goal_msg = Bool()
        #print("hi")
        if data:    
            if self.flag == True:
                self.goal_msg.data = True
                self.pub.publish(self.goal_msg)
                #self.rate.sleep()
                #if self.flag == False:
                #    break
            else:
                self.goal_msg.data = False
                self.pub.publish(self.goal_msg)
                #self.rate.sleep()
                #if self.flag == True:
                #    break

if __name__ == '__main__':
    rospy.init_node('listener')
    goal_listener()
    rospy.spin()