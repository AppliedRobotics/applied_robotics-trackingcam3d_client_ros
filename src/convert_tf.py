#!/usr/bin/env python3  
import roslib

import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
import tf_conversions

import tf2_ros

if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    listener = tf.TransformListener()
    turtle_vel = rospy.Publisher('turtle/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/world', '/kinect', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        angular = 4 * math.atan2(trans[1], trans[0])
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        turtle_vel.publish(cmd)
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world_1"
        t.child_frame_id = "camera"
        t.transform.translation.x = trans[0]
        t.transform.translation.y =  trans[1]
        t.transform.translation.z =  trans[2]
       # q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = rot[0]
        t.transform.rotation.y =rot[1]
        t.transform.rotation.z = rot[2]
        t.transform.rotation.w = rot[3]

        br.sendTransform(t)

        rate.sleep()