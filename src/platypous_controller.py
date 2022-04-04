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
turn_angle = 15

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

    def move(self, speed_m_per_s):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            vel_msg = Twist()
            if not self.detect_collision(Direction.Forward):
                vel_msg.linear.x += speed_m_per_s
                print("moving forward")
                if self.detect_collision(Direction.ForwardLeft):
                    vel_msg.angular.z += math.radians(-turn_angle)
                    print("turning right")
                elif self.detect_collision(Direction.ForwardRight):
                    vel_msg.angular.z += math.radians(turn_angle)
                    print("turning left")

                if self.detect_collision(Direction.Left):
                    vel_msg.angular.z += math.radians(-turn_angle)
                    print("turning left")
                else:
                    vel_msg.angular.z += math.radians(turn_angle)
                    print("turning right")

            else:
                print("detected wall")
                if self.detect_collision(Direction.ForwardLeft) or self.detect_collision(Direction.ForwardRight):
                    if self.detect_collision(Direction.ForwardLeft):
                        vel_msg.angular.z += math.radians(-turn_angle)
                        print("turning right")
                    elif self.detect_collision(Direction.ForwardRight):
                        vel_msg.angular.z += math.radians(turn_angle)
                        print("turning left")
                    elif self.get_closeset(Direction.ForwardLeft) < self.get_closeset(Direction.ForwardRight):
                        vel_msg.angular.z += math.radians(-turn_angle)
                        print("turning right")
                    elif self.get_closeset(Direction.ForwardLeft) > self.get_closeset(Direction.ForwardRight):
                        vel_msg.angular.z += math.radians(turn_angle)
                        print("turning left")
                    else:
                        vel_msg.angular.z += math.radians(-turn_angle)
                        print("turning right")
                else:
                    vel_msg.angular.z += math.radians(-turn_angle)
                    print("turning right")

            rate.sleep()
            self.twist_pub.publish(vel_msg)

    def get_closeset(self, Direction):
        minrange = 999
        for i in range(real_scan_range):
            range_measured = self.laserScan_data.ranges[Direction.value[0] + i]
            if range_measured < minrange:
                minrange = range_measured
        return minrange

    def detect_collision(self, Direction):
        for i in range(real_scan_range):
            if self.laserScan_data.ranges[Direction.value[0] - scan_range + i] <= distance_to_keep:
                return True
        return False


if __name__ == '__main__':
    # Init
    pc = platypous_controller()
    pc.move(1)
