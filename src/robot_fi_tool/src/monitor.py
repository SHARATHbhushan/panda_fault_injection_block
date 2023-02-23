import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import numpy as np
from std_msgs.msg import Bool
from robot_fi_tool.msg import faultmsg
import rosbag


class monitor:
    def __init__(self):
        self.collision_sub = rospy.Subscriber("collsion_model", String, self.collision_callback)
        self.fault_sub = rospy.Subscriber("fault_status", Bool, self.fault_callback)
        self.fault_msg_sub = rospy.Subscriber("fault_msg", faultmsg, self.fault_msg_callback)
