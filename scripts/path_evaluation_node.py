#!/usr/bin/env python
#coding=utf-8

# Author: xuzhihao
# Email: xuzhihao5@jd.com

import rospy
from path_evaluation

if __name__ == '__main__':
    node_name = 'teb_path_evalutator'
    rospy.init_node(node_name)
    try:
        rospy.loginfo(node_name + 'initialization system')
        path_evaluation.TEB_Path_Evalutator()
        path_evaluation.ClearParams()
        rospy.loginfo(node_name + 'process done and quit')
    except:
        path_evaluation.ClearParams()
        rospy.loginfo(node_name + 'process done and quit')
