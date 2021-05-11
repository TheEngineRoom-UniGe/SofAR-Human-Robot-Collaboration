#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
from human_baxter_collaboration.msg import HumanTf

def publish_human_frames(msg):

    br = tf2_ros.TransformBroadcaster()
    for transform in msg.frames:
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = transform.header.frame_id
        t.transform.translation.x = transform.pose.position.x
        t.transform.translation.y = transform.pose.position.y
        t.transform.translation.z = transform.pose.position.z

        t.transform.rotation.x = transform.pose.orientation.x
        t.transform.rotation.y = transform.pose.orientation.y
        t.transform.rotation.z = transform.pose.orientation.z
        t.transform.rotation.w = transform.pose.orientation.w

        br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('human_tf_publisher')
    rospy.Subscriber('human_tf', HumanTf, publish_human_frames)
    rospy.spin()
