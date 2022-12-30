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
import tf
my_map = None
pub = None


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
    map.data = [-1] * (map.info.width * map.info.height)
    return map

def callback(data):
    global my_map, pub
    # Loop through the laser readings and update the map
    robotX = data.odometry.pose.pose.position.x
    robotY = data.odometry.pose.pose.position.y
    robotOrientation = data.odometry.pose.pose.orientation
    _, _, robotOrientation = tf.transformations.euler_from_quaternion([robotOrientation.x, robotOrientation.y, robotOrientation.z, robotOrientation.w])
    for i in range(len(data.Laser_reading.ranges)):
        if data.Laser_reading.ranges[i] >= data.Laser_reading.range_max or data.Laser_reading.ranges[i] <= data.Laser_reading.range_min:
            continue
        # Calculate the angle of the laser reading
        angle = data.Laser_reading.angle_min + i * data.Laser_reading.angle_increment
        # Calculate the x and y coordinates of the laser reading
        endx = data.Laser_reading.ranges[i] * math.cos(angle + robotOrientation) + robotX
        endy = data.Laser_reading.ranges[i] * math.sin(angle + robotOrientation) + robotY
        # Calculate the map coordinates of the laser reading
        mapX = int((endx - my_map.info.origin.position.x) / my_map.info.resolution)
        mapY = int((endy - my_map.info.origin.position.y) / my_map.info.resolution)
        # Update the map
        if mapX >= 0 and mapX < my_map.info.width and mapY >= 0 and mapY < my_map.info.height:
            my_map.data[mapY * my_map.info.width + mapX] = 100
        # Loop through the points between the robot and the laser reading and set it to 0
        for j in range(1, int(data.Laser_reading.ranges[i] / my_map.info.resolution)):
            x = j * my_map.info.resolution * math.cos(angle + robotOrientation) + robotX
            y = j * my_map.info.resolution * math.sin(angle + robotOrientation) + robotY
            mapX = int((x - my_map.info.origin.position.x) / my_map.info.resolution)
            mapY = int((y - my_map.info.origin.position.y) / my_map.info.resolution)
            if mapX >= 0 and mapX < my_map.info.width and mapY >= 0 and mapY < my_map.info.height:
                my_map.data[mapY * my_map.info.width + mapX] = 0
    my_map.header.seq = my_map.header.seq + 1
    my_map.header.stamp = rospy.rostime.Time()
    # Publish the map
    rospy.loginfo("Publishing map")
    rospy.loginfo("Robot x: %f, y: %f, orientation: %f", robotX, robotY, robotOrientation)
    pub.publish(my_map)

def main():
    global my_map, pub
    my_map = create_map()
    node_name = "mapping_poses"
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo("%s is now running", node_name)
    topic = '/sensor_alignment'
    sub = rospy.Subscriber(topic, Sensors_topic, callback)
    pub = rospy.Publisher('Mapping', OccupancyGrid, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass