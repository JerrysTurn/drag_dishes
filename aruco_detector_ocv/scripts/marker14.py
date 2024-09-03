#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg

def broadcast_transform():
    rospy.init_node("tf2_broadcaster_node")
    
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    broadcaster = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    
    while not rospy.is_shutdown():
        try:
            trans = tf_buffer.lookup_transform("origin", "marker_id14", rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
            
        t = geometry_msgs.msg.TransformStamped()
        
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "origin"
        t.child_frame_id = "marker_id14"
        
        t.transform.translation.x = trans.transform.translation.x
        t.transform.translation.y = trans.transform.translation.y
        t.transform.translation.z = trans.transform.translation.z
        t.transform.rotation.x = trans.transform.rotation.x
        t.transform.rotation.y = trans.transform.rotation.y
        t.transform.rotation.z = trans.transform.rotation.z
        t.transform.rotation.w = trans.transform.rotation.w
        
        broadcaster.sendTransform(t)
        rate.sleep()
        
if __name__ == '__main__':
    try:
        broadcast_transform()
    except rospy.ROSInterruptException:
        pass    