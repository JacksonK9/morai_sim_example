#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import rospkg
from morai_msgs.msg  import EgoVehicleStatus # Message for get Vehicle Position
from math import pi, cos, sin, pi, sqrt, pow
from nav_msgs.msg import Path # Message for make Path
import tf
from geometry_msgs.msg import PoseStamped

class PathLoader :

    def __init__(self):
        rospy.init_node('path_loader')
        arg = rospy.myargv(argv=sys.argv)
        self.global_path_pub= rospy.Publisher('/global_path', Path, queue_size=1, latch=True)
        self.path = Path()
        rospack=rospkg.RosPack()
        pkg_path=rospack.get_path('wego')
        self.path_folder_name=arg[1]
        self.make_path_name=arg[2]
        full_path=pkg_path +'/'+ self.path_folder_name+'/'+self.make_path_name+'.txt'
        self.path_load(full_path)
        
        while not rospy.is_shutdown():
            rospy.spin()
        

    def path_load(self, path_name):
        self.path.header.stamp = rospy.Time.now()
        self.path.header.frame_id = "map"

        with open(path_name, "r") as f:
            tmp_line = f.readline()
            point_len = 0
            while tmp_line != '':
                point_len += 1
                x, y, z = tmp_line.strip().split()
                tmp_pose = PoseStamped()
                tmp_pose.header.seq = point_len - 1
                tmp_pose.header.stamp = rospy.Time.now()
                tmp_pose.header.frame_id = "map"
                tmp_pose.pose.position.x = float(x)
                tmp_pose.pose.position.y = float(y)
                tmp_pose.pose.position.z = float(z)
                tmp_pose.pose.orientation.w = 1.0
                self.path.poses.append(tmp_pose)
                tmp_line = f.readline()

        rospy.loginfo("Total {} points loaded".format(point_len))            
        self.global_path_pub.publish(self.path)
        
if __name__ == '__main__':
    try:
        new_class = PathLoader()
    except rospy.ROSInterruptException:
        pass

