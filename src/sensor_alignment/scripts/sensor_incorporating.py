#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import message_filters
from sensor_alignment.msg import Sensors_topic

def main():
    def callback(data1, data2):
        msg = Sensors_topic()
        msg.Laser_reading = data1
        msg.odometry = data2
        pub.publish(msg)
        
    node_name = "input_sensors_alignment"
    

    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo("%s is now running", node_name)
    
    # Log data of the laser topic
    topics = ['/scan_multi', '/robot/robotnik_base_control/odom']
    types = [LaserScan, Odometry]

    subs = [message_filters.Subscriber(topic, mtype) for topic, mtype in zip(topics, types)]
    ts = message_filters.ApproximateTimeSynchronizer(subs, 10, 0.01, allow_headerless=True)
    ts.registerCallback(callback)
    pub = rospy.Publisher('sensor_alignment', Sensors_topic, queue_size=10)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
