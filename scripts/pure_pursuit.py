#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import rospkg
from morai_msgs.msg  import EgoVehicleStatus # Message for get Vehicle Position
from math import pi, cos, sin, pi, sqrt, pow, atan2
from nav_msgs.msg import Path # Message for make Path
import tf
from geometry_msgs.msg import PoseStamped, Twist, PolygonStamped, Point32
from visualization_msgs.msg import Marker

class PurePursuit(object):

    def __init__(self):
        rospy.init_node('pure_pursuit')
        rospy.Subscriber('/global_path', Path, self.path_callback)
        rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.ego_callback)
        self.LOOKAHEAD_DISTANCE = rospy.get_param("~lookahead_distance", 3.0)
        self.PATH_ERROR_DISTANCE = rospy.get_param("~path_error_distance", 5.0)
        self.WHEELBASE = rospy.get_param("~wheelbase", 0.26)
        self.SPEED = rospy.get_param("~speed", 0.5)
        self.drive_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
        self.lookahead_pub = rospy.Publisher("lookahead_marker", Marker, queue_size=5)
        self.circle_polygon_pub = rospy.Publisher("lookahead_circle", PolygonStamped, queue_size=5)
        self.lookahead_marker = Marker()
        self.global_path = Path()
        self.closest_idx = 0
        
        while not rospy.is_shutdown():
            rospy.spin()
    
    def path_callback(self, _data):
        rospy.loginfo("Get {} path points".format(len(_data.poses)))
        self.global_path = _data

    def ego_callback(self, _data):
        br = tf.TransformBroadcaster()
        br.sendTransform((_data.position.x, _data.position.y, _data.position.z),
                        tf.transformations.quaternion_from_euler(0, 0, (_data.heading)/180*pi),
                        rospy.Time.now(),
                        "gps",
                        "map")
        self.lookahead_circle = PolygonStamped()
        self.lookahead_circle.header.stamp = rospy.Time.now()
        self.lookahead_circle.header.frame_id = "map"
        point_num = 30
        for i in range(point_num):
            theta = i * 2 * pi / point_num
            tmp_point = Point32()
            tmp_point.x = _data.position.x + self.LOOKAHEAD_DISTANCE * cos (theta)
            tmp_point.y = _data.position.y + self.LOOKAHEAD_DISTANCE * sin (theta)
            self.lookahead_circle.polygon.points.append(tmp_point)
        self.circle_polygon_pub.publish(self.lookahead_circle)
        
        if len(self.global_path.poses) < 3:
            self.drive_pub.publish(Twist())
            return
        self.closest_idx = self.find_closest_waypoint(_data.position.x, _data.position.y)
        self.lookahead_idx = self.find_lookahead_waypoint(_data.position.x, _data.position.y)
        if self.lookahead_idx == None:
            rospy.loginfo("LookAhead Point Not Found")
            self.drive_pub.publish(Twist())
            return
        alpha = atan2(self.global_path.poses[self.lookahead_idx].pose.position.y - _data.position.y, self.global_path.poses[self.lookahead_idx].pose.position.x - _data.position.x) - _data.heading * pi /180
        # rospy.loginfo("alpha {} rad, {} deg".format(alpha, alpha * 180 / pi))
        steering_angle = atan2(2 * self.WHEELBASE * sin(alpha), self.min_dist_print)

        drive_msg = Twist()
        drive_msg.linear.x = self.SPEED
        drive_msg.angular.z = steering_angle
        if self.min_dist_print < 0.25:
            self.drive_pub.publish(Twist())
            rospy.loginfo("maybe final path point reached?")
        else:
            self.drive_pub.publish(drive_msg)
            rospy.loginfo("speed = {} m/s, steering_angle = {} rad, {} deg".format(self.SPEED, steering_angle, steering_angle * 180 / pi))


    def find_lookahead_waypoint(self, x, y):
        # rospy.loginfo("x, y = {}, {}".format(x, y))
        min_dist = 1e9
        self.min_dist_print = 1e9
        min_idx = 0
        # rospy.loginfo(self.closest_idx)
        for i in range(self.closest_idx, len(self.global_path.poses)):
            dist = sqrt(pow(x - self.global_path.poses[i].pose.position.x, 2) + pow(y - self.global_path.poses[i].pose.position.y, 2))
            # rospy.loginfo("{}, {}".format(i, dist))
            if dist < self.LOOKAHEAD_DISTANCE:
                calc_dist = self.LOOKAHEAD_DISTANCE - dist
            else:
                calc_dist = dist - self.LOOKAHEAD_DISTANCE
            if calc_dist < min_dist:
                self.min_dist_print = dist
                min_dist = calc_dist
                min_idx = i
        # rospy.loginfo("{}, {}".format(min_dist, min_idx))
        if (min_dist + self.LOOKAHEAD_DISTANCE) > self.PATH_ERROR_DISTANCE:
            self.lookahead_marker.header.stamp = rospy.Time.now()
            self.lookahead_marker.header.frame_id = "map"
            self.lookahead_marker.id = 0
            self.lookahead_marker.type = 2
            self.lookahead_marker.action = 3
            self.lookahead_pub.publish(self.lookahead_marker)
            return None
        else:
            # rospy.loginfo("LookAhead Point Found, Distance to LookAhead Point --> {}m {}th point".format(self.min_dist_print, min_idx))
            self.lookahead_marker.header.stamp = rospy.Time.now()
            self.lookahead_marker.header.frame_id = "map"
            self.lookahead_marker.id = 0
            self.lookahead_marker.type = 2
            self.lookahead_marker.action = 0
            self.lookahead_marker.pose.position = self.global_path.poses[min_idx].pose.position
            self.lookahead_marker.pose.orientation.w = 1.0
            self.lookahead_marker.scale.x = 0.5
            self.lookahead_marker.scale.y = 0.5
            self.lookahead_marker.scale.z = 0.5
            self.lookahead_marker.color.r = 1.0
            self.lookahead_marker.color.g = 0.0
            self.lookahead_marker.color.b = 0.0
            self.lookahead_marker.color.a = 1.0
            self.lookahead_pub.publish(self.lookahead_marker)
            return min_idx

    def find_closest_waypoint(self, x, y):
        min_dist = 1e9
        min_idx = 0
        for i in range(self.closest_idx, len(self.global_path.poses)):
            dist = sqrt(pow(x - self.global_path.poses[i].pose.position.x, 2) + pow(y - self.global_path.poses[i].pose.position.y, 2))
            if dist < min_dist:
                min_dist = dist
                min_idx = i

        # rospy.loginfo("closest point in path --> {} m, {}th point".format(min_dist, min_idx))
        return min_idx


        
if __name__ == '__main__':
    try:
        new_class = PurePursuit()
    except rospy.ROSInterruptException:
        pass

