#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg


if __name__ == '__main__':
    rospy.init_node('tf2_broadcaster')
    
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    
    x = 45.0
    while not rospy.is_shutdown():
		t.header.stamp = rospy.Time.now()
		t.header.frame_id = "/map"
		t.child_frame_id = "/laser"
		t.transform.translation.x = x
		t.transform.translation.y = -58.0
		t.transform.translation.z = 0.0
		t.transform.rotation.x = 0.0
		t.transform.rotation.y = 0.0
		t.transform.rotation.z = 0.0
		t.transform.rotation.w = 1.0
		br.sendTransform(t)
    	
    rospy.spin()
