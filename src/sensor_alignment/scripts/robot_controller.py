#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from pynput import keyboard
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

msg = Twist()
pub = None
msg.linear.x = 0.0
msg.linear.y = 0.0
msg.linear.z = 0.0
msg.angular.x = 0.0
msg.angular.y = 0.0
msg.angular.z = 0.0


# the following bunch of code just to listen to user's inputs
# create a callback function to handle user's input then create
# a keyboard listener. It stops when the callback function return false. 
def on_press(key):
    global msg, pub
    rospy.loginfo('pressed ')	
    if key == keyboard.Key.esc or key == 'q':
        return False  # stop listener
    try:
        user_input = key.char  # single-char keys
    except:
        user_input = key.name  # other keys
    #if user_input not in ['a', 'd', 'w', 's']:
    #    return False
    if user_input == 'a':
        # left
        msg.angular.z = 2.0
    if user_input == 'd':
        # left
        msg.angular.z = -2.0
    if user_input == 'w':
        # forward
        msg.linear.x = 2.0
    if user_input == 's':
        # backward
        msg.linear.x = -2.0
    return False  # stop listener
    
    
    
def reset_message():
    global msg
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0



def main():
    global msg, pub
    node_name = "input_listener"
    topic = "/robot/cmd_vel"
        
    # initiate the node and create a publisher
    rospy.init_node(node_name, anonymous=True)
    pub = rospy.Publisher(topic, Twist, queue_size=10)
    rospy.loginfo("%s is now running", node_name)
    
    rate = rospy.Rate(2)
    
    while not rospy.is_shutdown():
        listener = keyboard.Listener(on_press=on_press)
        listener.start()  # start to listen on a separate thread
        listener.join()  # remove if main thread is polling self.keys
        rospy.loginfo('publishing: linear: [%i, %i, %i], angular:  [%i, %i, %i]',msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z)
        pub.publish(msg)
        reset_message()
        rate.sleep()
        pub.publish(msg)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
