import rospy
import math
from geometry_msgs.msg import Twist

class platypous_controller:
    def __init__(self):
        rospy.init_node('platypous_controller', anonymous=True)
        self.twist_pub = rospy.Publisher(
            '/cmd_vel/nav', Twist, queue_size=10)

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
            vel_msg.angular.y = degrees_per_sec
        else:
            vel_msg.angular.y = -degrees_per_sec
        self.twist_pub.publish(vel_msg)

        rate = rospy.Rate(100)
        t0 = rospy.Time().now().to_sec()
        while (rospy.Time().now().to_sec() - t0 <= time_sec) and not (rospy.is_shutdown()):
            self.twist_pub.publish(vel_msg)
            rate.sleep()
        vel_msg.linear.x = 0
        self.twist_pub.publish(vel_msg)



if __name__ == '__main__':
    # Init
    pc = platypous_controller()
    pc.rotate(1, 10)
