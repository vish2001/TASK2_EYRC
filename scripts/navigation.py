#!/usr/bin/env python
"""Import statements"""
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion

class MoveBaseSeq():

    def __init__(self):

        rospy.init_node('move_base_sequence')
        #List of x,y,z values of all the 5 waypoints
        points_seq = [-9.1, -1.2, 0.0, 10.7, 10.5, 0.0, 12.6, -1.6, 0.0, 18.2, -1.4, 0.0, -2.0, 4.0, 0.0]
        #List of Quaternions for each waypoint
        #w=1 ; For having no rotation of the mobile base frame w.r.t. map frame
        quat_seq = [Quaternion(0, 0, 0, 1), Quaternion(0, 0, 0, 1), Quaternion(0, 0, 0, 1), Quaternion(0, 0, 0, 1), Quaternion(0, 0, 0, 1)]
        #List of  goal pose sequence
        self.pose_seq = list()
        self.goal_cnt = 0
        n = 3
        # Returns a list points as [[point1], [point2],...[pointn]]
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
        for point in points:
            self.pose_seq.append(Pose(Point(*point), quat_seq[n-3]))
            n += 1
        #Create action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(10.0))
        if not wait:
            rospy.loginfo("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Task 2 initiating ... :)")
        self.movebase_client()

    def active_cb(self):
        '''Activation command for the navigation'''
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        '''Feedback message while achieving waypoints'''
        rospy.loginfo("Ebot is now moving towards waypoint "+str(self.goal_cnt+1))

    def done_cb(self, status, result):
        '''Command to be executed on completion of the previous waypoint'''
        self.goal_cnt += 1
        if status == 2:
            # The goal received a cancel request after it started executing
            # and has since completed its execution (Terminal State)
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            # The goal was achieved successfully by the action server (Terminal State)
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached")
            if self.goal_cnt < len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb)
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                return

        if status == 4:
            # The goal was aborted during execution by the action server due
            # to some failure (Terminal State)
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            return
    #Create the movebase client
    #This is used to send the sequence of goals to the navigation stack
    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal  "+str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        rospy.spin()
# Python Main
if __name__ == '__main__':
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")

