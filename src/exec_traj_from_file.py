#!/usr/bin/env python

import pdb
import sys
import os
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import geometry_msgs.msg
import std_msgs.msg
from moveit_utils import ArmMoveIt

def read_file(is_joint_pos, filename):
    traj_file = open(filename, 'r')
    lines = traj_file.readlines()
    poses = []

    for l in lines:
        l = l.replace(',', '')
        l = l.replace(';', '')
        pose_str = l.strip().split()
        target_pose = geometry_msgs.msg.Pose()
        if is_joint_pos:
            pose_pt = dict() 
            for p in range(len(pose_str)):
                pose_pt["joint_" + str(p+1)] = float(pose_str[p])
            target_pose = pose_pt
        elif len(pose_str) == 3:
            target_pose.position.x = float(pose_str[0])
            target_pose.position.y = float(pose_str[1])
            target_pose.position.z = float(pose_str[2])
            target_pose.orientation.w = 1.0
            target_pose.orientation.x = 0.0
            target_pose.orientation.y = 0.0
            target_pose.orientation.z = 0.0
        elif len(pose_str) == 6:
            ## TODO: Convert Euler angles to Quaternion
            target_pose.position.x = float(pose_str[0])
            target_pose.position.y = float(pose_str[1])
            target_pose.position.z = float(pose_str[2])
            target_pose.orientation.w = 1.0
            target_pose.orientation.x = 0.0
            target_pose.orientation.y = 0.0
            target_pose.orientation.z = 0.0
        elif len(pose_str) == 7:
            target_pose.position.x = float(pose_str[0])
            target_pose.position.y = float(pose_str[1])
            target_pose.position.z = float(pose_str[2])
            target_pose.orientation.x = float(pose_str[3])
            target_pose.orientation.y = float(pose_str[4])
            target_pose.orientation.z = float(pose_str[5])
            target_pose.orientation.w = float(pose_str[6])
        else:
            rospy.logerr("Formatting error in pose file: " % (pose_str))
            return []
        poses.append(target_pose)
    return poses

def main():
    arm = ArmMoveIt()
    filename = rospy.get_param("~traj_file")
    is_joint_pos = not rospy.get_param("~ee")
    target_poses = read_file(is_joint_pos, filename)

    for target_pose in target_poses:
        if not rospy.is_shutdown():
            print('\n The target coordinate is: %s \n' %target_pose)     
        
            ## IK for target position  
            #joint_target = arm.get_IK(target_pose)
            #print('IK calculation step:DONE') 
        
            ## planning with joint target from IK 
            #plan_traj = arm.plan_pose(joint_target, is_joint_pos=True)
            #print('Planning step with target joint angles:DONE') 
        
            ## planning with pose target
            plan_traj = arm.plan_pose(target_pose, is_joint_pos=is_joint_pos)
            plan_pts = plan_traj.joint_trajectory.points
            print("Last joint pose:")
            print(plan_pts[-1].positions)
        
            ## execution of the movement   
            arm.group.execute(plan_traj)
    
if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_utils', anonymous=True)
    
    main()

