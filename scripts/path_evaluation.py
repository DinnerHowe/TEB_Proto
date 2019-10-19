#!/usr/bin/env python
#coding=utf-8

# Author: xuzhihao
# Email: xuzhihao5@jd.com

import rospy
from teb_local_planner.msg import FeedbackMsg
from scipy import integrate
from nav_msgs.msg import Path

class ClearParams():
    def __init__(self):
        rospy.delete_param('~teb_paths')

class TEB_Path_Evalutator():
    def __init__(self):
        self.define()
        rospy.Subscriber(self.teb_paths, FeedbackMsg, self.path_evaluate)
        rospy.spin()

    def define(self):
        if not rospy.has_param('~teb_paths'):
            rospy.set_param('~teb_paths', '/move_base/TebLocalPlannerROS/local_plan')
            self.teb_paths = rospy.get_param('~teb_paths')
        if not rospy.has_param('~teb_topic_name'):
            rospy.set_param('~teb_topic_name', '/move_base/TebLocalPlannerROS/choose_plan')
            teb_topic_name = rospy.get_param('~teb_topic_name')

        self.pub_ = rospy.Publisher(teb_topic_name, Path, queue_size=1)


    def path_evaluate(self, trajetorys):
        selected_trajectory_idx = 0
        for i in trajetorys.trajectories:
            pre_slote = i.trajectory.poses.position
            for poses_num in len(i.trajectory.poses)[1:len(i.trajectory.poses)-2]:
                slote = i.trajectory.poses[poses_num].position
                slote_next = i.trajectory.poses[poses_num+1].position
                if not self.smooth(pre, cur, nex):
                    selected_trajectory_idx += 1
                    continue
                else:
                    if selected_trajectory_idx <= len(trajetorys.trajectories):
                        self.pub_.publish(trajetorys.trajectories[selected_trajectory_idx])


    def smooth(self, pre, cur, nex):
    	if cur.y - pre.y != 0 and nex.y - cur.y != 0:
    		if self.sign((cur.x - pre.x)/(cur.y - pre.y)): == self.sign((nex.x - cur.x)/(nex.y - cur.y)):
    	elif cur.y - pre.y == 0 and nex.y - cur.y != 0:
    		return false
    	elif cur.y - pre.y != 0 and nex.y - cur.y == 0:
    		return false
    	else:
    		return ture
