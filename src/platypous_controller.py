import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from enum import Enum

scan_range = 35
real_scan_range = scan_range*2
distance_to_keep = 1


class Direction(Enum):
    Left = 180,
    ForwardLeft = 90,
    Forward = 0,
    ForwardRight = -90,
    Right = 180,


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

    def move_straight(self, speed_m_per_s, forward=True):
        vel_msg = Twist()
        if forward:
            vel_msg.linear.x = speed_m_per_s
        else:
            vel_msg.linear.x = -speed_m_per_s
        self.twist_pub.publish(vel_msg)

        rate = rospy.Rate(100)
        while not (rospy.is_shutdown()) and not self.detect_collision(Direction.Forward):
            self.twist_pub.publish(vel_msg)
            rate.sleep()
        vel_msg.linear.x = 0
        self.twist_pub.publish(vel_msg)

    def rotate(self, degrees_per_sec, forward=True):
        vel_msg = Twist()
        if forward:
            vel_msg.angular.z = math.radians(degrees_per_sec)

        else:
            vel_msg.angular.z = math.radians(-degrees_per_sec)

        self.twist_pub.publish(vel_msg)

        rate = rospy.Rate(100)
        while not (rospy.is_shutdown()) and self.detect_collision(Direction.Forward):
            self.twist_pub.publish(vel_msg)
            rate.sleep()
        vel_msg.angular.z = 0
        self.twist_pub.publish(vel_msg)

    def detect_collision(self, Direction):
        for i in range(real_scan_range):
            if self.laserScan_data.ranges[Direction.value[0] - scan_range + i] <= distance_to_keep:
                return True
        return False


if __name__ == '__main__':
    # Init
    pc = platypous_controller()
    while True:
        if not pc.detect_collision(Direction.Forward):
            print("moving forward")
            pc.move_straight(1)
        elif pc.detect_collision(Direction.ForwardRight):
            print("moving left")
            pc.rotate(30)
        elif pc.detect_collision(Direction.ForwardLeft):
            print("moving right")
            pc.rotate(30,  False)
        else:
            print("doing nothing, so moving right")
            pc.rotate(30, False)
