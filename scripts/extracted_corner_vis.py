#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

pub = rospy.Publisher('extracted_corner_vis', MarkerArray, queue_size=10)
markerArray = MarkerArray()

scale = Vector3()
scale.x = 0.1
scale.y = 0.1
scale.z = 0.1

color = ColorRGBA()
color.a = 1.0
color.r = 0.0
color.g = 0.0
color.b = 1.0

def callback(data):
    data_size = len(data.poses)
    # if data_size >= 3:
    # for i in range(2, data_size, 2):
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.type = marker.POINTS
    marker.action = marker.ADD
    marker.scale = scale
    marker.color = color
    marker.pose.orientation.w = 1.0

    p = Point()
    p.x = 42.0
    p.y = -59.3
    p.z = 0
    # p.x = data.poses[i].position.x
    # p.y = data.poses[i].position.y
    # p.z = 0
    print('-----------------')
    print(marker)
    marker.points.append(p)
    markerArray.markers.append(marker)
   # print(data.poses)
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s\n", data)

    
def listener():
    rospy.init_node('extracted_corner_vis', anonymous=True)
    pub = rospy.Publisher('extracted_corner_vis', MarkerArray, queue_size=10)
    rospy.Subscriber("posearray_corner", PoseArray, callback)

    while not rospy.is_shutdown():
        pub.publish(MarkerArray)
        
        rospy.spin()

if __name__ == '__main__':
    listener()