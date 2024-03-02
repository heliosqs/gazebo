#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

def callback(msg):
    print('X: %s, Y: %s' % (msg.pose.pose.position.x, msg.pose.pose.position.y))

def laser_callback(msg):
    print('Min distance: %s' % min(msg.ranges))

def main():
    #sub = rospy.Subscriber('/odom', Odometry, callback)
    sub = rospy.Subscriber('/laser/scan', LaserScan, laser_callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('exercise_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = Twist()
        msg.linear.x = 0.3
        msg.angular.z = 0.3
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass