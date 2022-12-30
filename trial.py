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
import tf

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
        robotX = data1.odometry.pose.pose.position.x - (my_map.info.resolution * my_map.info.width/2)
        robotX /= my_map.info.resolution
        robotY = data1.odometry.pose.pose.position.y - (my_map.info.resolution * my_map.info.height/2)
        robotY /= my_map.info.resolution
        robotOrientationQuaterinion = data1.odometry.pose.pose.orientation
        _,_, robotOrientation = tf.transformations.euler_from_quaternion([robotOrientationQuaterinion.x, robotOrientationQuaterinion.y, robotOrientationQuaterinion.z, robotOrientationQuaterinion.w])
        angles = np.arange(0, 2 * np.pi, 0.5 * np.pi / 180)
        ranges = np.array(data1.Laser_reading.ranges)
        # Loop through the laser readings and update the map
        for i in range(len(data1.Laser_reading.ranges)):
            # If the laser reading is out of range, skip it
            if data1.Laser_reading.ranges[i] > data1.Laser_reading.range_max or data1.Laser_reading.ranges[i] < data1.Laser_reading.range_min:
                continue
            try:
                # Calculate the distance
                dist = ranges[i] / my_map.info.resolution
                # Calculate the angle of the laser reading in the robot's frame
                angle = angles[i] + robotOrientation + math.radians(180)
                # Get the end point of the laser reading
                endx = robotX + dist * math.cos(angle)
                endy = robotY + dist * math.sin(angle)
                # Get the index of the map cell that the laser reading corresponds to
                index = int(endx) + int(endy) * my_map.info.width
                # increment the hits list for the cell
                hits_map[index] += 1
                # Print the max and average hits_map
                
                # loop through all cells between the robot and the laser reading
                dx = (endx - robotX) / dist    
                dy = (endy - robotY) / dist
                xs = np.arange(robotX, endx+dx, dx).astype(int)
                ys = np.arange(robotY, endy+dy, dy).astype(int)
                minlen = min(len(xs), len(ys))
                xs = xs[:minlen]
                ys = ys[:minlen]
                misses_map[xs + ys * my_map.info.width] += 1
                # Print the average and maximum misses
            except Exception as e:
                rospy.loginfo("Error")
                rospy.loginfo(angle)        
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