#! /usr/bin/env python
import random
import rospy
import actionlib # 引用actionlib库

import move_base_msgs.msg as move_base_msgs
import visualization_msgs.msg as viz_msgs

import yaml
import geometry_msgs.msg as geometry_msgs
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



class TourMachine(object):

    def __init__(self, filename, random_visits=False, repeat=False):
        self._waypoints = get_waypoints(filename) # 获取一系列目标点的值

        action_name = 'move_base'
        self._ac_move_base = actionlib.SimpleActionClient(action_name, move_base_msgs.MoveBaseAction) # 创建一个SimpleActionClient
        rospy.loginfo('Wait for %s server' % action_name)
        self._ac_move_base.wait_for_server
        self._counter = 0
        self._repeat = repeat
        self._random_visits = random_visits

        if self._random_visits:
            random.shuffle(self._waypoints)

    def move_to_next(self):
        p = self._get_next_destination()

        if not p:
            rospy.loginfo("Finishing Tour")
            return True
        # 把文件读取的目标点信息转换成move_base的goal的格式：
        goal = create_move_base_goal(p)
        rospy.loginfo("Move to %s" % p['name'])
        # 这里也是一句很简单的send_goal:
        self._ac_move_base.send_goal(goal)
        self._ac_move_base.wait_for_result()
        result = self._ac_move_base.get_result()
        rospy.loginfo("Result : %s" % result)

        return False

    def _get_next_destination(self):
        """
        根据是否循环，是否随机访问等判断，决定下一个目标点是哪个
        """
        if self._counter == len(self._waypoints):
            if self._repeat:
                self._counter = 0
                if self._random_visits:
                    random.shuffle(self._waypoints)
            else:
                next_destination = None
        next_destination = self._waypoints[self._counter]
        self._counter = self._counter + 1
        return next_destination

    def spin(self):
        rospy.sleep(1.0)
        finished = False
        while not rospy.is_shutdown() and not finished:
            finished = self.move_to_next()
            rospy.sleep(2.0)

if __name__ == '__main__':
    rospy.init_node('tour')
    # 使用了ros的get_param读取文件名：
    filename = rospy.get_param('~filename')
    random_visits = rospy.get_param('~random', False)
    repeat = rospy.get_param('~repeat', False)

    m = TourMachine(filename, random_visits, repeat)
    rospy.loginfo('Initialized')
    m.spin()
    rospy.loginfo('Bye Bye')
