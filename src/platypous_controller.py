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
distance_to_keep = 0.57
max_distance = 1
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
    NotSet = -1,


class Turning(Enum):
    Left = 0,
    Right = 1,
    NotSet = 2,


class Orientation:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

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

    def wheelTwistOdometry(self, msg):
        self.wheelTwistOdometry_data = msg

    def laserScan(self, msg):
        self.laserScan_data = msg

    def get_wall_angle(self, starting_angle):
        angle = 10
        front_wall_dist = self.get_distance(starting_angle-int(angle/2), False)
        center_wall_dist = self.get_distance(starting_angle, False)
        back_wall_dist = self.get_distance(starting_angle+int(angle/2), False)

        third_wall_length_calc = (
            front_wall_dist**2)+(back_wall_dist**2)-(2*back_wall_dist*front_wall_dist)*math.cos(math.radians(angle))
        third_wall_length = math.sqrt(third_wall_length_calc)

        third_wall_length_angle_calc = ((third_wall_length**2)+(back_wall_dist**2)-(
            front_wall_dist**2))/(2*third_wall_length*back_wall_dist)
        front_wall_angle = math.degrees(
            math.acos(third_wall_length_angle_calc))

        return -(90-int(angle/2)-front_wall_angle)

    def move(self, speed_m_per_s):
        rate = rospy.Rate(100)
        turning_dir = Turning.NotSet
        while not rospy.is_shutdown():
            vel_msg = Twist()
            if not self.detect_collision(Direction.Forward):
                turning_dir = Turning.NotSet
                vel_msg.linear.x += speed_m_per_s

                if self.detect_collision(Direction.ForwardLeft):
                    vel_msg.angular.z += math.radians(-10)

                elif self.detect_collision(Direction.ForwardRight):
                    vel_msg.angular.z += math.radians(10)

                stapilization_needed = False

                if self.get_distance(Direction.Right.value[0], False) < self.get_distance(Direction.Left.value[0], False):
                    if self.get_distance(Direction.Right.value[0], False) < distance_to_keep:
                        vel_msg.angular.z += math.radians(5)
                    elif self.get_distance(Direction.Right.value[0], False) > max_distance:
                        vel_msg.angular.z += math.radians(-5)
                    else:
                        stapilization_needed = True
                else:
                    if self.get_distance(Direction.Left.value[0], False) < distance_to_keep:
                        vel_msg.angular.z += math.radians(-5)
                    elif self.get_distance(Direction.Left.value[0], False) > max_distance:
                        vel_msg.angular.z += math.radians(5)
                    else:
                        stapilization_needed = True

                if stapilization_needed:
                    if self.detect_collision(Direction.Left) or self.detect_collision(Direction.Left) and self.detect_collision(Direction.Right):
                        turn = self.get_wall_angle(Direction.Right.value[0])
                        if not math.isnan(turn):
                            print(str(turn))
                            vel_msg.angular.z -= math.radians(turn)
                    else:
                        turn = self.get_wall_angle(Direction.Left.value[0])
                        if not math.isnan(turn):
                            print(str(turn))
                            vel_msg.angular.z += math.radians(turn)
            else:
                if turning_dir == Turning.NotSet:
                    if self.detect_collision(Direction.ForwardLeft) or self.detect_collision(Direction.ForwardRight):
                        if self.get_closeset(Direction.ForwardLeft) < self.get_closeset(Direction.ForwardRight):
                            turning_dir = Turning.Right
                            print("turning right")
                        elif self.get_closeset(Direction.ForwardLeft) > self.get_closeset(Direction.ForwardRight):
                            turning_dir = Turning.Left
                            print("turning left")
                        else:
                            turning_dir = Turning.Right
                            print("turning right")
                    else:
                        turning_dir = Turning.Right
                        print("turning right")
                elif turning_dir == Turning.Left:
                    vel_msg.angular.z += math.radians(10)
                else:
                    vel_msg.angular.z -= math.radians(10)

            rate.sleep()
            self.twist_pub.publish(vel_msg)

    def get_closeset(self, Direction):
        minrange = 999
        for i in range(real_scan_range):
            range_measured = self.laserScan_data.ranges[Direction.value[0] + i]
            if (range_measured < minrange) and range_measured != 0:
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
    pc = platypous_controller()
    pc.move(0.5)
