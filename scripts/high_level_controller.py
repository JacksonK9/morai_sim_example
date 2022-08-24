#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math


class HighLevelController(object):
    def __init__(self):
        rospy.init_node('high_level_controller')
        rospy.loginfo("Initilized")
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        self.motor_pub = rospy.Publisher("commands/motor/speed", Float64, queue_size=5)
        self.servo_pub = rospy.Publisher("commands/servo/position", Float64, queue_size=5)
        while not rospy.is_shutdown():
            rospy.spin()
    
    def cmd_vel_callback(self, _data):
        motor = _data.linear.x * 1000
        servo = _data.angular.z * 0.5 / 0.34 + 0.5
        if servo < 0.0: servo = 0.0
        elif servo > 1.0: servo = 1.0
        self.motor_pub.publish(motor)
        self.servo_pub.publish(servo)
        rospy.loginfo("Input {:.4f} m/s, {:.4f} rad, {:.4f} deg ======= Output motor speed {} rpm, servo position {:.4f}".format(_data.linear.x, _data.angular.z, _data.angular.z * 180 / math.pi, motor, servo))
    
if __name__ == '__main__':
    try:
        new_class = HighLevelController()
    except rospy.ROSInterruptException:
        pass

