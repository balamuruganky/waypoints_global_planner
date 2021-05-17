#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import path_planner as pp
import path_planning.PoseHelper as PoseHelper

trajectory_type = "dubins"
id = "map"

def prepare_path():
    desired_path = Path()
    samples, yaw_samples = pp.prepare_desired_path(trajectory_type)
    for t, point in enumerate(samples, start=0):
        pose = PoseHelper.prepare_posestamped(samples[t][0], samples[t][1], yaw_samples[t])
        pose.header.seq = t 
        pose.header.frame_id = id
        pose.header.stamp = rospy.get_rostime()
        desired_path.poses.append(pose)

    desired_path.header.frame_id = id
    desired_path.header.stamp = rospy.get_rostime()
    desired_path_pub.publish(desired_path) 
    desired_path = []

def path_publisher():
    global desired_path_pub
    desired_path_pub = rospy.Publisher('/desired_path', Path, queue_size=1)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        prepare_path()
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('desired_path_node')
    rospy.loginfo("desired_path node is started!!")
    trajectory_type =  rospy.get_param('~trajectory_type')
    try:
        path_publisher()
    except rospy.ROSInterruptException:
        pass
