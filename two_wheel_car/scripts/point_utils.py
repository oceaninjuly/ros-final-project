#! /usr/bin/env python
import yaml
import rospy
import move_base_msgs.msg as move_base_msgs
import geometry_msgs.msg as geometry_msgs
import visualization_msgs.msg as viz_msgs
import std_msgs.msg as std_msgs

id_count = 1

def get_waypoints(filename):
    # 目标点文件是yaml格式的：
    with open(filename, 'r') as f:
        data = yaml.load(f,Loader=yaml.FullLoader)

    return data['waypoints']

def create_geo_pose(p):
    pose = geometry_msgs.Pose()

    pose.position.x = p['pose']['position']['x']
    pose.position.y = p['pose']['position']['y']
    pose.position.z = p['pose']['position']['z']
    pose.orientation.x = p['pose']['orientation']['x']
    pose.orientation.y = p['pose']['orientation']['y']
    pose.orientation.z = p['pose']['orientation']['z']
    pose.orientation.w = p['pose']['orientation']['w']
    return pose

def create_move_base_goal(p):
    target = geometry_msgs.PoseStamped()
    target.header.frame_id = p['frame_id']
    target.header.stamp = rospy.Time.now()
    target.pose = create_geo_pose(p)
    goal = move_base_msgs.MoveBaseGoal(target)
    return goal
