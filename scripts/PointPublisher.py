#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
import os
import math

def talker():
    rospy.init_node('PointPublisher', anonymous=True)
    pub = rospy.Publisher('point_database_markerarray', MarkerArray, queue_size=10)

    data = []
    file=open('/home/nvidia/catkin_ws/src/Corner-Detection/data/point_database2.txt', 'r')
    while (1):
        line=file.readline()

        try:escape=line.index('\n')
        except:escape=len(line)
        
        if line:
            line = line[0:escape]
            line_data = line.split(',')
            data.append(line_data)
        else:
            break
    file.close()

    markerArray = MarkerArray()
    for d in data:
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.POINTS
        marker.action = marker.ADD
        marker.scale.x = 0.1
    	marker.scale.y = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        p = Point()
        p.x = float(d[1])
        p.y = float(d[2])
        p.z = 0
        marker.points.append(p)
        markerArray.markers.append(marker)

    cos = math.cos(0.355001)
    sin = math.sin(0.355001)
    # cos = math.cos(0)
    # sin = math.sin(0)
    rx = 73.4874
    ry = -58.2916

    cx1 = 1.22876 + 0.23
    cy1 = -1.56395

    cx2 = -2.55152 + 0.23
    cy2 = 1.96797
    
    id = 0
    for m in markerArray.markers:
        m.id = id
        id += 1

    while not rospy.is_shutdown():
        # Publish the MarkerArray
        pub.publish(markerArray)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
