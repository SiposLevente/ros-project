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

class Orientation:
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z

class Position:
    def __init__(self,x,y,z):
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

    def getWalAngle(self, starting_angle):
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
        while not rospy.is_shutdown():
            vel_msg = Twist()
            print(self.wheelTwistOdometry_data.pose.pose.position.x)

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

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        orientation = Orientation(math.degrees(roll_x), math.degrees(pitch_y), math.degrees(yaw_z))

        return orientation

    def get_current_orientation(self):
        orientation = self.wheelTwistOdometry_data.pose.pose.orientation
        return self.euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)

    def get_current_position(self):
        position = self.wheelTwistOdometry_data.pose.pose.position
        return Position(position.x, position.y, position.z)

    def get_distance_between_positions(self, pos_base, pos_comp):
        return math.sqrt(math.pow((pos_comp.x-pos_base.x), 2)+math.pow((pos_comp.y-pos_base.y), 2)+math.pow((pos_comp.z-pos_base.z), 2))

if __name__ == '__main__':
    pc = platypous_controller()
    pc.move(1)
