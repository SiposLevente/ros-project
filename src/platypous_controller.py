import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid


class platypous_controller:
    def __init__(self):
        rospy.init_node('platypous_controller', anonymous=True)
        self.twist_pub = rospy.Publisher(
            '/cmd_vel/nav', Twist, queue_size=10)
        self.slam_listener = rospy.Subscriber(name, OccupancyGrid)

    def test(self, forward):
        vel_msg = Twist()
        if forward:
            vel_msg.linear.x = 2
            vel_msg.angular.z = 2
        else:
            vel_msg.linear.x = -2
            vel_msg.angular.z = -2

        rate = rospy.Rate(100)
        self.twist_pub.publish(vel_msg)

        t0 = rospy.Time().now().to_sec()

        while(rospy.Time().now().to_sec() - t0 <= 2):
            print(forward)
            self.twist_pub.publish(vel_msg)
            rate.sleep()

        vel_msg.linear.x = 0
        self.twist_pub.publish(vel_msg)


if __name__ == '__main__':
    # Init
    pc = platypous_controller()
    toggler = False
    while(True):
        pc.test(toggler)
        toggler = not toggler

    # Send turtle on a straight line
