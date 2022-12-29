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
    def callback(data1, data2, data3):
        msg = Sensors_topic()
        msg.Laser_reading = data1
        msg.odometry = data3
        # compine ranges in one range in the same axis 
        msg.Laser_reading.ranges = data1.ranges + data2.ranges
        msg.Laser_reading.intensities = data1.intensities + data2.intensities
        pub.publish(msg)
        
    node_name = "input_sensors_alignment"
    

    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo("%s is now running", node_name)
    
    
    topics = ['/robot/front_laser/scan', '/robot/rear_laser/scan', '/robot/robotnik_base_control/odom']
    types = [LaserScan, LaserScan, Odometry]

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