#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import String

def main():
    def callback(data):
        rospy.loginfo(data)     
    node_name = "input_translator"
    topic = "/robot/input"    
        
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo("%s is now running", node_name)
    
    rospy.Subscriber(topic, String, callback)
    
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass