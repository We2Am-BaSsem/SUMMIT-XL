#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import message_filters
from sensor_alignment.msg import Sensors_topic
from nav_msgs.msg import Odometry, OccupancyGrid
from tf2_msgs.msg import TFMessage
def main():
    def callback(data1, data2):
        if data2.transforms[0].header.frame_id == 'robot_odom' and data2.transforms[0].child_frame_id == 'robot_base_footprint':
            #if the robot intersect with the objects
            if data1.Laser_reading.ranges[ int(len(data1.Laser_reading.ranges)/4) ] < 0.5:
                rospy.loginfo("I heard %s",data1)
                #publish the position of the robot
                #pub.publish(data2.transforms[0].transform.translation)
            

                        

    node_name = "mapping_poses"
    

    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo("%s is now running", node_name)
    
    
    topics = ['/sensor_alignment','/tf']
    types = [Sensors_topic,TFMessage]

    subs = [message_filters.Subscriber(topic, mtype) for topic, mtype in zip(topics, types)]
    ts = message_filters.ApproximateTimeSynchronizer(subs, 10, 0.01, allow_headerless=True)
    ts.registerCallback(callback)
    pub = rospy.Publisher('Mapping', Sensors_topic, queue_size=10)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass