import rospy
import math
import numpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from enum import Enum

scan_range = 35
real_scan_range = scan_range*2
distance_to_keep = 1.5
turn_angle = 10


class Direction(Enum):
    BackLeft = 270,
    Left = 180,
    ForwardLeft = 90,

    Forward = 0,

    ForwardRight = -90,
    Right = -180,
    BackRight = -270,
    Back = 360,


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

    def getWalAngle(self, starting_angle):
        angle = 10
        front_wall_dist = self.get_distance(starting_angle-int(angle/2),False)
        center_wall_dist = self.get_distance(starting_angle,False)
        back_wall_dist = self.get_distance(starting_angle+int(angle/2),False)

        third_wall_length_calc = (
            front_wall_dist**2)+(back_wall_dist**2)-(2*back_wall_dist*front_wall_dist)*math.cos(math.radians(angle))
        third_wall_length = math.sqrt(third_wall_length_calc)

        third_wall_length_angle_calc = ((third_wall_length**2)+(back_wall_dist**2)-(front_wall_dist**2))/(2*third_wall_length*back_wall_dist)
        front_wall_angle = math.degrees(math.acos(third_wall_length_angle_calc))

        return -(90-int(angle/2)-front_wall_angle)
        
    def test(self):
        rate = rospy.Rate(100)
        
        while not rospy.is_shutdown():
            vel_msg = Twist()
            angle_to_turn = self.getWalAngle(Direction.Right.value[0])
            vel_msg.angular.z = math.radians(angle_to_turn)
            print(angle_to_turn)
            self.twist_pub.publish(vel_msg)
            rate.sleep()

    def move(self, speed_m_per_s):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            vel_msg = Twist()
            if not self.detect_collision(Direction.Forward):
                angle_to_turn = self.getWalAngle(Direction.Right.value[0])
                print(angle_to_turn)

                safe_angle = 1.5
                if angle_to_turn <= safe_angle and angle_to_turn >= -safe_angle:
                    vel_msg.linear.x += speed_m_per_s
                if angle_to_turn != math.nan and angle_to_turn < 40:
                    vel_msg.angular.z = math.radians(angle_to_turn)
                print("moving forward")

            else:
                print("wall detected")

            rate.sleep()
            self.twist_pub.publish(vel_msg)

    def get_closeset(self, Direction):
        minrange = 999
        for i in range(real_scan_range):
            range_measured = self.laserScan_data.ranges[Direction.value[0] + i]
            if range_measured < minrange:
                minrange = range_measured
        return minrange

    def get_distance(self, degree, real_degrees=True):
        if real_degrees:
            degree *= 2
        return self.laserScan_data.ranges[degree]

    def detect_collision(self, Direction):
        for i in range(real_scan_range):
            if self.laserScan_data.ranges[Direction.value[0] - scan_range + i] <= distance_to_keep:
                return True
        return False


if __name__ == '__main__':
    # Init
    pc = platypous_controller()
    pc.move(1)
