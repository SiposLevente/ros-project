import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

from os import system


class platypous_controller:

    def __init__(self):
        self.listener()
        rospy.sleep(1)
        self.publisher()

    def publisher(self):
        self.twist_pub = rospy.Publisher(
            '/cmd_vel/nav', Twist, queue_size=10)

    def listener(self):
        rospy.init_node('platypous_controller', anonymous=True)
        self.subscribe_odometry = rospy.Subscriber(
            "/driver/wheel_odometry", Odometry, self.wheelTwistOdometry)
        self.subscribe_laser = rospy.Subscriber(
            "/scan", LaserScan, self.laserScan)
        self.subscribe_slam = rospy.Subscriber(
            "/map", OccupancyGrid, self.slam)

    def wheelTwistOdometry(self, msg):
        self.wheelTwistOdometry_data = msg

    def laserScan(self, msg):
        self.laserScan_data = msg

    def slam(self, msg):
        self.slam_data = msg
        print(msg)

    def move_straight(self, speed_m_per_s, time_sec, forward=True):
        vel_msg = Twist()
        if forward:
            vel_msg.linear.x = speed_m_per_s
        else:
            vel_msg.linear.x = -speed_m_per_s
        self.twist_pub.publish(vel_msg)

        rate = rospy.Rate(100)
        t0 = rospy.Time().now().to_sec()
        while (rospy.Time().now().to_sec() - t0 <= time_sec) and not (rospy.is_shutdown()):
            self.twist_pub.publish(vel_msg)
            rate.sleep()
        vel_msg.linear.x = 0
        self.twist_pub.publish(vel_msg)

    def rotate(self, degrees_per_sec, time_sec, forward=True):
        vel_msg = Twist()
        if forward:
            vel_msg.angular.z = degrees_per_sec

        else:
            vel_msg.angular.z = -degrees_per_sec

        self.twist_pub.publish(vel_msg)

        rate = rospy.Rate(100)
        t0 = rospy.Time().now().to_sec()
        while (rospy.Time().now().to_sec() - t0 <= time_sec) and not (rospy.is_shutdown()):
            self.twist_pub.publish(vel_msg)
            rate.sleep()
        vel_msg.angular.z = 0
        self.twist_pub.publish(vel_msg)


if __name__ == '__main__':
    # Init
    pc = platypous_controller()
    # print(pc.wheelTwistOdometry_data)
    toggler = False
    direction = 360
    while not rospy.is_shutdown():
        if(pc.laserScan_data.ranges[direction] >= 2.0):
            pc.move_straight(1, 1, toggler)
        else:
            if toggler:
                direction = 360
            else:
                direction = 0
            toggler = not toggler
    # print(pc.slam_data)
