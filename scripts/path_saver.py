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


class PathSaver(object) :

    def __init__(self):
        rospy.init_node('path_saver', anonymous=True)

        arg = rospy.myargv(argv=sys.argv)
        rospy.Subscriber("/Ego_topic",EgoVehicleStatus, self.status_callback)
        self.global_path_pub= rospy.Publisher('/global_path', Path, queue_size=1)

        self.is_status=False
        self.prev_x = 0
        self.prev_y = 0
        self.path = Path()

        rospack=rospkg.RosPack()
        pkg_path=rospack.get_path('wego')
        self.path_folder_name=arg[1]
        self.make_path_name=arg[2]
        full_path=pkg_path +'/'+ self.path_folder_name+'/'+self.make_path_name+'.txt'
        self.f=open(full_path, 'w')

        rate=rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.is_status==True :
                self.path_make(0.3)
            rate.sleep()    

        self.f.close()
        

    def path_make(self, distance_th):
        self.path.header.stamp = rospy.Time.now()
        self.path.header.frame_id = "map"
        
        x=self.status_msg.position.x
        y=self.status_msg.position.y
        z=self.status_msg.position.z

        distance=sqrt(pow(x-self.prev_x,2)+pow(y-self.prev_y,2))
        if distance > distance_th:
            data='{0}\t{1}\t{2}\n'.format(x,y,z)
            self.f.write(data)
            self.prev_x=x
            self.prev_y=y
            tmp_pose = PoseStamped()
            tmp_pose.header.stamp = rospy.Time.now()
            tmp_pose.header.frame_id = "map"
            tmp_pose.pose.position.x = x
            tmp_pose.pose.position.y = y
            tmp_pose.pose.position.z = z
            tmp_pose.pose.orientation.w = 1.0
            self.path.poses.append(tmp_pose)
            
            print(x,y)
        self.global_path_pub.publish(self.path)

    def status_callback(self,msg): ## Vehicl Status Subscriber 
        self.is_status=True
        self.status_msg=msg
        br = tf.TransformBroadcaster()
        br.sendTransform((self.status_msg.position.x, self.status_msg.position.y, self.status_msg.position.z),
                        tf.transformations.quaternion_from_euler(0, 0, (self.status_msg.heading+90)/180*pi),
                        rospy.Time.now(),
                        "gps",
                        "map")
        

if __name__ == '__main__':
    try:
        new_class = PathSaver()
    except rospy.ROSInterruptException:
        pass

