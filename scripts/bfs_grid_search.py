#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from math import pi, cos, sin, pi, sqrt, pow, atan2
from nav_msgs.msg import Path  # Message for make Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from nav_msgs.srv import GetMap
from collections import deque

class BFSGridSearch(object):

    def __init__(self):
        rospy.init_node('grid_search')
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initialpose_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.path_pub = rospy.Publisher("global_path", Path, queue_size=1, latch=True)
        rospy.wait_for_service("static_map")
        self.map_service = rospy.ServiceProxy("static_map", GetMap)
        response = self.map_service()
        self.map_data = response.map
        rospy.loginfo("map resolution = {}, width, height = {}, {}".format(self.map_data.info.resolution, self.map_data.info.width, self.map_data.info.height))
        rospy.loginfo("Grid Searching Initialized")
        # self.BFSSearch()

        self.start_position = None
        self.goal_position = None
        while not rospy.is_shutdown():
            rospy.spin()

    def initialpose_callback(self, _data):
        self.start_position = _data.pose.pose.position
        if self.start_position == None or self.goal_position == None:
            rospy.logwarn("Start Position or Goal Position Not Set")
            return
        else:
            rospy.loginfo("Start Searching Path")
            self.BFSSearch()

    def goal_callback(self, _data):
        self.goal_position = _data.pose.position
        if self.start_position == None or self.goal_position == None:
            rospy.logwarn("Start Position or Goal Position Not Set")
            return
        else:
            rospy.loginfo("Start Searching Path")
            self.BFSSearch()

    def BFSSearch(self):
        response = self.map_service()
        self.map_data = response.map
        self.map_resolution = self.map_data.info.resolution
        self.map_width = self.map_data.info.width
        self.map_height = self.map_data.info.height
        self.map_origin_x = self.map_data.info.origin.position.x
        self.map_origin_y = self.map_data.info.origin.position.y
        self.map_data.data = list(self.map_data.data)
        # print(self.map_origin_x, self.map_origin_y)
        ## 1d list to 2d list
        map2d = []
        for i in range(self.map_height):
            map2d.append(self.map_data.data[i * self.map_width : (i + 1) * self.map_width]) # 모르는 곳 -1, Free Space가 0, Occupied Space가 100으로 설정됨
            # 0번이 지도의 가장 아래쪽 줄, 숫자가 커질수록 윗줄을 의미
        # for i in range(self.map_height):
        #     print(i, map2d[i])
        ## find start grid
        start_grid = [int((self.start_position.y - self.map_origin_y) // self.map_resolution), int((self.start_position.x - self.map_origin_x) // self.map_resolution)]
        goal_grid = [int((self.goal_position.y - self.map_origin_y) / self.map_resolution), int((self.goal_position.x - self.map_origin_x) / self.map_resolution)]
        # print(start_grid)
        # print(goal_grid)
        # print(map2d[start_grid[0]])
        rospy.loginfo("[{}, {}] to [{}, {}] Grid Search Start".format(start_grid[0], start_grid[1], goal_grid[0], goal_grid[1]))

        # BFS Searching
        # visited = [[0] * self.map_width for _ in range(self.map_height)]
        move_vector_x = [+1, 0, -1, 0]
        move_vector_y = [0, +1, 0, -1]
        
        Grid_Queue = deque()
        Grid_Queue.append(start_grid)
        # print(type(map2d))
        map2d[start_grid[0]][start_grid[1]] = 1
        came_from = {}
        while Grid_Queue:
            now = Grid_Queue.popleft()
            # print(now)
            
            
            tmp_x = now[0] + move_vector_x[0]
            tmp_y = now[1] + move_vector_y[0]
            if self.check_inside_map(tmp_x, tmp_y) and map2d[tmp_x][tmp_y] == 0:
                Grid_Queue.append([tmp_x, tmp_y])
                came_from[tmp_x * self.map_width + tmp_y] = [now[0] * self.map_width + now[1]]
                map2d[tmp_x][tmp_y] = map2d[now[0]][now[1]] + 1
            
            if tmp_x == goal_grid[0] and tmp_y == goal_grid[1]:
                break

            tmp_x = now[0] + move_vector_x[1]
            tmp_y = now[1] + move_vector_y[1]
            if self.check_inside_map(tmp_x, tmp_y) and map2d[tmp_x][tmp_y] == 0:
                Grid_Queue.append([tmp_x, tmp_y])
                came_from[tmp_x * self.map_width + tmp_y] = [now[0] * self.map_width + now[1]]
                map2d[tmp_x][tmp_y] = map2d[now[0]][now[1]] + 1

            if tmp_x == goal_grid[0] and tmp_y == goal_grid[1]:
                break

            tmp_x = now[0] + move_vector_x[2]
            tmp_y = now[1] + move_vector_y[2]
            if self.check_inside_map(tmp_x, tmp_y) and map2d[tmp_x][tmp_y] == 0:
                Grid_Queue.append([tmp_x, tmp_y])
                came_from[tmp_x * self.map_width + tmp_y] = [now[0] * self.map_width + now[1]]
                map2d[tmp_x][tmp_y] = map2d[now[0]][now[1]] + 1

            if tmp_x == goal_grid[0] and tmp_y == goal_grid[1]:
                break

            tmp_x = now[0] + move_vector_x[3]
            tmp_y = now[1] + move_vector_y[3]
            if self.check_inside_map(tmp_x, tmp_y) and map2d[tmp_x][tmp_y] == 0:
                Grid_Queue.append([tmp_x, tmp_y])
                came_from[tmp_x * self.map_width + tmp_y] = [now[0] * self.map_width + now[1]]
                map2d[tmp_x][tmp_y] = map2d[now[0]][now[1]] + 1

            if tmp_x == goal_grid[0] and tmp_y == goal_grid[1]:
                break
            
            # for i in range(71, 35, -1):
            #     for j in range(35, 71):
            #         print map2d[i][j],
            #     print('')

        if map2d[goal_grid[0]][goal_grid[1]] < 1:
            rospy.loginfo("Cannot Find Path")
        else:
            rospy.loginfo("Total length from start to goal is {}".format(map2d[goal_grid[0]][goal_grid[1]]))
            for i in range(71, 35, -1):
                for j in range(35, 71):
                    print "{:3d}".format(map2d[i][j]),
                print('')

            current = goal_grid
            path = []
            while current != start_grid:
                path.append(current)
                tmp = came_from[current[0] * self.map_width + current[1]]
                # print(tmp)
                current = [int(tmp[0] // self.map_width), int(tmp[0] % self.map_width)]
            
            path.append(start_grid)
            path.reverse()
            print(path)
            self.pub_path(path)

    
    def pub_path(self, path):
        tmp = Path()
        tmp.header.stamp = rospy.Time.now()
        tmp.header.frame_id = 'map'

        length = 0
        for i in path:
            tmp_point = PoseStamped()
            tmp_point.header.stamp = rospy.Time.now()
            tmp_point.header.seq = length
            length += 1
            tmp_point.header.frame_id = "map"
            x, y = self.grid_to_meter(i)
            tmp_point.pose.position.x = x
            tmp_point.pose.position.y = y
            tmp_point.pose.orientation.w = 1.0
            tmp.poses.append(tmp_point)
        self.path_pub.publish(tmp)
        rospy.loginfo("Path Published")
        

    def grid_to_meter(self, grid):
        x_pos = self.map_origin_x + grid[1] * self.map_resolution + self.map_resolution * 0.5
        y_pos = self.map_origin_y + grid[0] * self.map_resolution + self.map_resolution * 0.5
        return x_pos, y_pos



    def check_inside_map(self, x, y):
        if 0 <= x < self.map_height and 0 <= y < self.map_width:
            return True
        else:
            return False



        
if __name__ == '__main__':
    try:
        new_class = BFSGridSearch()
    except rospy.ROSInterruptException:
        pass

