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
    rospy.init_node('LinePublisher', anonymous=True)
    pub = rospy.Publisher('line_database_markerarray', MarkerArray, queue_size=10)

    data = []
    file=open('/home/nvidia/catkin_ws/src/corner_detector/data/line_database.txt', 'r')
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
    for i in range(0, len(data), 2):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0

        p1 = Point()
        p1.x = float(data[i][1])
        p1.y = float(data[i][2])
        p1.z = 0

        p2 = Point()
        p2.x = float(data[i+1][1])
        p2.y = float(data[i+1][2])
        p2.z = 0

        # print(p1.x, p1.y, p1.z, "|", p2.x, p2.y, p2.z)

        marker.points.append(p1)
        marker.points.append(p2)

        markerArray.markers.append(marker)

    id = 0
    for m in markerArray.markers:
        m.id = id
        id += 1

    while not rospy.is_shutdown():
        # Publish the MarkerArray
        pub.publish(markerArray)
        # print(markerArray)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
