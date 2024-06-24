import rospy
import actionlib
import move_base_msgs.msg as move_base_msgs

def active_callback(state,res):
    print('goal active.')
    print(state)

def done_callback():
    print('goal done.')

def feedback_callback(fb):
    print(fb)

rospy.init_node('guider')
move_base = actionlib.SimpleActionClient('move_base',move_base_msgs.MoveBaseAction)
move_base.wait_for_server()
goal = move_base_msgs.MoveBaseGoal()
goal.target_pose.pose.position.x = 7.0
goal.target_pose.pose.position.y = 0.5
goal.target_pose.pose.orientation.z = 0  
goal.target_pose.pose.orientation.w = 1
move_base.send_goal(goal,active_callback,done_callback,feedback_callback)
move_base.wait_for_result()
