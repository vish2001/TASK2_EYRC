#!/usr/bin/env python
import rospy

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry


def movebase_client():
    global i
    i = 0
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    Waypoints = [[-9.1,-1.2],[10.7,10.5],[12.6,-1.9],[18.2,-1.4],[-2.0,4.0]]
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = -9.1
    goal.target_pose.pose.position.y =  -1.2
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.loginfo("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        data = Odometry()
        
        if result:
            rospy.loginfo("Goal execution done!")
            

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

