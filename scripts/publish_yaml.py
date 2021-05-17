#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import rospkg

import sys
from yaml import load, dump

import path_planner as pp
import path_planning.PoseHelper as PoseHelper

trajectory_type = "dubins"
yaml_file_path = "waypoints_path/waypoints.yaml"

def prepare_path():
    samples, yaw_samples = pp.prepare_desired_path(trajectory_type)
    path_list = []
    waypoint_dict = {}
    for t, point in enumerate(samples, start=0):
        point_dict = {}
        path_dict = {}
        point_dict['x'] = str(samples[t][0])
        point_dict['y'] = str(samples[t][1])
        point_dict['yaw'] = str(yaw_samples[t])
        path_dict['point'] = point_dict
        path_list.append(path_dict)

    waypoint_dict['waypoints'] = path_list
    rospack = rospkg.RosPack()
    abs_path = rospack.get_path('waypoints_global_planner') + '/' + yaml_file_path
    with open(abs_path, 'w') as f:
        dump(waypoint_dict, f, allow_unicode=True)

    yaml_path_pub.publish(abs_path)

def path_publisher():
    global yaml_path_pub
    yaml_path_pub = rospy.Publisher('/yaml_filepath', String, queue_size=1)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        prepare_path()
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('yaml_filepath_node')
    rospy.loginfo("yaml_filepath node is started!!")
    trajectory_type =  rospy.get_param('~trajectory_type')
    yaml_file_path =  rospy.get_param('~yaml_file_path')
    try:
        path_publisher()
    except rospy.ROSInterruptException:
        pass
