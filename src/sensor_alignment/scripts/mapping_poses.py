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
import numpy as np
import math
def main():            
    def create_map():
        #create the map
        map = OccupancyGrid()
        map.header.frame_id = 'robot_map'
        map.header.stamp = rospy.rostime.Time()
        map.header.seq = 0
        map.info.resolution = 0.02
        map.info.width = 4992
        map.info.height = 4992
        map.info.origin.position.x = 0.03 - map.info.resolution * map.info.width/2
        map.info.origin.position.y = 0.032 - map.info.resolution * map.info.height/2
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
                # Calculate the angle of the laser reading in the robot's frame
                angle = data1.Laser_reading.angle_min + i * data1.Laser_reading.angle_increment
                # Calculate the angle of the laser reading in the map's frame
                angle = angle + data2.transforms[0].transform.rotation.z + math.radians(90)
                # Calculate the distance of the laser reading in x,y in the robot's frame
                x = data1.Laser_reading.ranges[i] * math.cos(angle)
                y = data1.Laser_reading.ranges[i] * math.sin(angle)
                # Calculate the distance of the laser reading in x,y in the map's frame
                x = x + data2.transforms[0].transform.translation.x
                y = y + data2.transforms[0].transform.translation.y
                # store it as the endPoint 
                endx = x
                endy = y
                # Calculate the index of the map cell that the laser reading corresponds to
                index = int((x - my_map.info.origin.position.x) / my_map.info.resolution) + int((y - my_map.info.origin.position.y) / my_map.info.resolution) * my_map.info.width
                # increment the hits list for the cell
                hits_map[index] = hits_map[index] + 1
                # loop through all cells between the robot and the laser reading
                for x_d in np.arange(data2.transforms[0].transform.translation.x, endx, my_map.info.resolution):
                    for y_d in np.arange(data2.transforms[0].transform.translation.y, endy, my_map.info.resolution):
                        # Calculate the index of the map cell that the laser reading corresponds to
                        index = int((x_d - my_map.info.origin.position.x) / my_map.info.resolution) + int((y_d - my_map.info.origin.position.y) / my_map.info.resolution) * my_map.info.width
                        # increment the misses list for the cell
                        misses_map[index] = misses_map[index] + 1
            
            # Update the map data to be probability of occupancy
            for i in range(len(my_map.data)):
                if hits_map[i] + misses_map[i] > 0:
                    my_map.data[i] = int(100 * (hits_map[i] / (hits_map[i] + misses_map[i])))
                else:
                    my_map.data[i] = 0
            # Update the map header
            my_map.header.seq = my_map.header.seq + 1
            my_map.header.stamp = rospy.rostime.Time()
            # Publish the map
            pub.publish(my_map)

    # Create a hits list to count the number of times a cell has been hit
    hits_map = np.zeros((4992*4992))
    # Create a misses map to count the number of times a cell has been missed
    misses_map = np.zeros((4992*4992))
    # Create the map
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