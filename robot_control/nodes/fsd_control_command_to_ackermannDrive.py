#!/usr/bin/python

import rospy
import math
from ackermann_msgs.msg import AckermannDrive
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from fsd_common_msgs.msg import ControlCommand

class Convert:
    def __init__(self):
        self.publisher = rospy.Publisher('/robot_control/command', AckermannDriveStamped, queue_size=10)
        self.max_steering = 1
        self.min_steering = -1
        self.epsilon_steering = math.radians(0.001)

    def callback(self, data):
        ack_cmd = AckermannDriveStamped()
        ack_cmd.header.stamp = rospy.Time.now()

        drive = AckermannDrive()
        rospy.loginfo(data)
        drive.speed = data.throttle.data
        drive.steering_angle = data.steering_angle.data

        # impose limits on commanded angle
        if drive.steering_angle > self.max_steering:
            drive.steering_angle = self.max_steering
        if drive.steering_angle < self.min_steering:
            drive.steering_angle = self.min_steering

        # clean up angle if it is very close to zero
        if math.fabs(drive.steering_angle) < self.epsilon_steering:
            drive.steering_angle = 0.0


        ack_cmd.drive = drive
        self.publisher.publish(ack_cmd)

    def listener(self):
        rospy.Subscriber("/control/pure_pursuit/control_command", ControlCommand, self.callback)
        rospy.spin()

if __name__ == '__main__':
    try:
        # ControllCommand comes from FSD_Skeleton
        rospy.init_node("controlCommandToAckermannDriveNode", anonymous=True)
        cnv = Convert()
        cnv.listener()
    except rospy.ROSInterruptException: pass
