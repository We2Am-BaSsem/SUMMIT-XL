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
import math
def main():            
    def create_map():
        #create the map
        map = OccupancyGrid()
        map.header.frame_id = 'map'
        map.header.stamp = rospy.rostime.Time()
        map.header.seq = 0
        map.info.resolution = 1
        map.info.width = 4992
        map.info.height = 4992
        map.info.origin.position.x = 0
        map.info.origin.position.y = 0
        map.info.origin.position.z = 0
        map.info.origin.orientation.x = 0
        map.info.origin.orientation.y = 0
        map.info.origin.orientation.z = 0
        map.info.origin.orientation.w = 1
        map.data = [0] * (map.info.width * map.info.height)
        return map
    
    def callback(data1, data2):
        if data2.transforms[0].header.frame_id == 'robot_odom' and data2.transforms[0].child_frame_id == 'robot_base_footprint':
        # Loop through the laser readings and update the map
            for i in range(len(data1.Laser_reading.ranges)):
                # Get the angle of the laser reading
                angle = data1.Laser_reading.angle_min + i * data1.Laser_reading.angle_increment
                # Get the distance of the laser reading
                distance = data1.Laser_reading.ranges[i]
                # Calculate the x and y coordinates of the laser reading
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)
                # Convert the x and y coordinates to map coordinates using the current pose of the robot
                x_map = int((x + data2.transforms[0].transform.translation.x) / my_map.info.resolution)
                y_map = int((y + data2.transforms[0].transform.translation.y) / my_map.info.resolution)
                # Update the map
                my_map.data[x_map + y_map * my_map.info.width] = 100
            # Convert the map to a robot map
            my_map.header.stamp = rospy.Time.now()
            # Publish the map
            pub.publish(my_map)

    my_map = create_map()
    node_name = "mapping_poses"
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo("%s is now running", node_name)
    topics = ['/sensor_alignment','/tf']
    types = [Sensors_topic,TFMessage]
    subs = [message_filters.Subscriber(topic, mtype) for topic, mtype in zip(topics, types)]
    ts = message_filters.ApproximateTimeSynchronizer(subs, 10, 0.01, allow_headerless=True)
    ts.registerCallback(callback)
    pub = rospy.Publisher('Mapping', OccupancyGrid, queue_size=10)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass