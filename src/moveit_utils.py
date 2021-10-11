#!/usr/bin/env python

# ADAPTED FROM https://github.com/HLP-R/hlpr_manipulation

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
from math import pi, floor, ceil, fabs

class ArmMoveIt(object):
    def __init__(self, planner="RRTConnectkConfigDefault", base_frame="base_link", ee_frame="end_effector_link"):
        super(ArmMoveIt, self).__init__()

        try:
          self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
          if self.is_gripper_present:
            gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
            self.gripper_joint_name = gripper_joint_names[0]
          else:
            gripper_joint_name = ""
          self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

          # Create the MoveItInterface necessary objects
          arm_group_name = "arm"
          self.robot = moveit_commander.RobotCommander("robot_description")
          self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
          self.group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
          self.group.set_pose_reference_frame(base_frame)
          self.group.set_end_effector_link(ee_frame)
          self.group.set_goal_orientation_tolerance(0.01)

          self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=20)
          self.continuous_joints = ['joint_1','joint_3','joint_5','joint_7']
          self.planner = planner

          if self.is_gripper_present:
            gripper_group_name = "gripper"
            self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

          rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
        except Exception as e:
          print (e)
          self.is_init_success = False
        else:
          self.is_init_success = True

    def get_IK(self, new_pose, root = None, group_id=0):
        """ Find the corresponding joint angles for an end effector pose
        
        Uses MoveIt! to compute the inverse kinematics for a given end
        effector pose.

        Parameters
        ----------
        new_pose : geometry_msgs/Pose
            the end effector pose
        root : string, optional
            the root link (if none is provided, uses the planning frame)
        group_id : int, optional
            the index of the group for which to calculate the IK.  Used 
            for compatibility with future functionality; leave set to 
            default for now.

        Returns
        ----------
        list 
            The joint angles corresponding to the end effector pose
        """
        
        ## from a defined newPose (geometry_msgs.msg.Pose()), retunr its correspondent joint angle(list)
        rospy.wait_for_service('compute_ik')
        compute_ik = rospy.ServiceProxy('compute_ik', moveit_msgs.srv.GetPositionIK)

        wkPose = geometry_msgs.msg.PoseStamped()
        if root is None:
            wkPose.header.frame_id = self.group.get_planning_frame() # name:odom
        else:
            wkPose.header.frame_id = root
        
        wkPose.header.stamp=rospy.Time.now()
        wkPose.pose=new_pose

        msgs_request = moveit_msgs.msg.PositionIKRequest()
        msgs_request.group_name = self.group.get_name() # name: arm
        msgs_request.robot_state = self.robot.get_current_state()
        msgs_request.pose_stamped = wkPose
        msgs_request.timeout.secs = 2
        msgs_request.avoid_collisions = False
        try:
            joint_angle=compute_ik(msgs_request)
            ans = dict(zip(joint_angle.solution.joint_state.name, joint_angle.solution.joint_state.position))
            if joint_angle.error_code.val == -31:
                rospy.logerr('No IK solution')
                return None
            return ans
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))

    def get_FK(self, root = None, state = None):
        """ Find the end effector pose for the current joint configuration
        
        Uses MoveIt! to compute the forward kinematics for a given joint
        configuration.

        Parameters
        ----------
        root : string, optional
            the root link (if none is provided, uses the planning frame)
        state : RobotState, optional
            the state to calculate FK for (if none is provided, uses the 
                current state)
        
        Returns
        ----------
        geometry_msgs/PoseStamped []
            the pose of the end effector
        """
        if root is None:
            root = self.group.get_pose_reference_frame()
        
        rospy.wait_for_service('compute_fk')
        compute_fk = rospy.ServiceProxy('compute_fk', moveit_msgs.srv.GetPositionFK)

        header = std_msgs.msg.Header()
        header.frame_id = root
        header.stamp = rospy.Time.now()
        fk_link_names = ['j2s7s300_ee_link']
        if state is None:
            robot_state = self.robot.get_current_state()
        else:
            robot_state = state
        try:
            reply=compute_fk(header,fk_link_names,robot_state)
            return reply.pose_stamped

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))

    def plan_joint_pos(self, target, starting_config=None, group_id=0):
        """ Plan a trajectory to reach a given joint configuration
        
        Uses MoveIt! to plan a trajectory to reach a given joint
        configuration

        Parameters
        ----------
        target : list or dict
            if a list, a list of positions for all active joints 
            in the group; if a dictionary, a mapping of joint names 
            to positions for the joints that should be moved (all other 
            joints will be held at their current angles).
        starting_config : RobotState, optional
            the starting configuration to plan from.  If not set, will 
            be set to the current state of the robot.
        group_id : int, optional
            the index of the group for which to calculate the IK.  Used 
            for compatibility with future functionality; leave set to 
            default for now.

        Returns
        ----------
        RobotTrajectory
            the plan for reaching the target position
        """
        
        self.set_start_state(starting_config)
        self.set_joint_target(target,group_id)
        return self.get_plan()

    def plan_ee_pos(self, target, starting_config=None, group_id=0):
        """ Plan a trajectory to reach a given end effector position
        
        Uses MoveIt! to plan a trajectory to reach a given end effector
        position

        Parameters
        ----------
        target : geometry_msgs/Pose or geometry_msgs/PoseStamped
            the desired pose of the end effector
        starting_config : RobotState, optional
            the starting configuration to plan from.  If not set, will 
            be set to the current state of the robot.
        group_id : int, optional
            the index of the group for which to calculate the IK.  Used 
            for compatibility with future functionality; leave set to 
            default for now.

        Returns
        ----------
        RobotTrajectory
            the plan for reaching the target position
        """
        self.set_start_state(starting_config)
        self.set_ee_target(target, group_id)
        return self.get_plan()

    def set_start_state(self, joint_config=None, group_id=0):
        """ Set the starting state from which to plan
        
        Sets the MoveIt! starting position in preparation for planning

        Parameters
        ----------
        joint_config (RobotState) -- the starting configuration to plan
            from.  If not set, will be set to the current state of the robot.
        group_id : int, optional
            the index of the group for which to calculate the IK.  Used 
            for compatibility with future functionality; leave set to 
            default for now.
        """

        if joint_config is not None:
            start_state = joint_config
        else:
            start_state = self._copy_state()

        try:
            self.group.set_start_state(start_state)
        except moveit_commander.MoveItCommanderException as e:
            rospy.logerr('Unable to set start state: {}'.format(e))


    def set_joint_target(self, target, group_id=0):
        """ Set the joint configuration target
        
        Sets the MoveIt! target position in preparation for planning

        Parameters
        ----------
        target (list or dict) -- if a list, a list of positions for
            all active joints in the group; if a dictionary, a mapping
            of joint names to positions for the joints that should be
            moved (all other joints will be held at their current angles).
        group_id (int) -- the index of the group for which to calculate
            the IK.  Used for compatibility with future functionality; 
            leave set to default for now.
        """
        try:
            a = self._simplify_joints(target,group_id)
            self.group.set_joint_value_target(self._simplify_joints(target,group_id))
            self.group.set_planner_id(self.planner)
        except moveit_commander.MoveItCommanderException as e:
            rospy.logerr('Unable to set target and planner: {}'.format(e))

    def set_ee_target(self, target, group_id=0):
        """ Set the end effector position target
        
        Sets the MoveIt! target position in preparation for planning

        Parameters
        ----------
        target : geometry_msgs/Pose
            the desired pose of the end effector
        group_id : int, optional
            the index of the group for which to calculate the IK.  Used 
            for compatibility with future functionality; leave set to 
            default for now.
        """
        
        try:
            # sanitize possible numpy before sending off to moveit
            if type(target).__module__ == 'numpy':
                target = target.tolist()

            self.group.set_pose_target(target)
            self.group.set_planner_id(self.planner)
        except moveit_commander.MoveItCommanderException as e:
            rospy.logerr('Unable to set target and planner: {}'.format(e))

    def get_plan(self,group_id=0):
        '''Generates a plan for reaching the current goal
        
        Uses MoveIt! to generate a plan based on the previously-set starting
        position and target position.

        .. note:: You must set the target and starting position before calling
            this function.
        
        Parameters
        ----------
        group_id : int, optional
            the index of the group for which to calculate the IK.  Used 
            for compatibility with future functionality; leave set to 
            default for now.

        Returns
        ----------
        RobotTrajectory
            the plan for reaching the target position
        '''
        try:
            plan =  self.group.plan()
        except moveit_commander.MoveItCommanderException as e:
            rospy.logerr('No plan found: {}'.format(e))
            return None
        return plan

    def move_robot(self, plan, wait = True, group_id=0):
        '''Moves the robot according to a plan
        
        Uses MoveIt! to move the arm according to the provided plan.

        .. warning:: The plans can be stitched together, but this can 
            have unexpected issues since the plan boundaries can violate 
            acceleration limits.
        
        Parameters
        ----------
        plan : RobotTrajectory
            a plan generated by MoveIt!
        wait : bool, optional
            whether to return immediately or block until movement is complete. 
            Default value is True
        group_id : int, optional
            the index of the group for which to calculate the IK.  Used 
            for compatibility with future functionality; leave set to 
            default for now.

        Returns
        ----------
        bool
            True on success
        '''
        return self.group.execute(plan, wait)


    def plan_joint_waypoints(self, targets, group_id=0, starting_config=None):
        '''Generates a multi-segment plan to reach waypoints in joint space
        
        Uses MoveIt! to generate a plan from target to target. One plan is
        generated for each segment.

        .. warning:: The plans can be stitched together, but this can 
            have unexpected issues since the plan boundaries can violate 
            acceleration limits.
        
        Parameters
        ----------
        targets : list
            a list of joint targets (either list or dict; see documentation 
            for :py:meth:`plan_joint_pos` )
        group_id : int, optional
            the index of the group for which to calculate the IK.  Used 
            for compatibility with future functionality; leave set to 
            default for now.
        starting_config : RobotState
            the starting state. If not provided this will default to the 
            current state of the robot.

        Returns
        ----------
        list of RobotTrajectory
            the plan for reaching the target position
        '''
        all_plans = []
        current_config = starting_config
        for target in targets:
            plan = self.plan_joint_pos(target, current_config, group_id=group_id)
            if plan!=None:
                all_plans.append(plan)
                try:
                    current_config=self.state_from_trajectory(plan.joint_trajectory)
                except moveit_commander.MoveItCommanderException as e:
                    rospy.logerr("Couldn't set configuration. Error:{}".format(e))
        return all_plans

    def plan_ee_waypoints(self, targets, group_id=0, starting_config=None):
        '''Generates a multi-segment plan to reach end effector waypoints
        
        Uses MoveIt! to generate a plan from target to target. One plan is
        generated for each segment.

        .. warning:: The plans can be stitched together, but this can have 
            unexpected issues since the plan boundaries can violate 
            acceleration limits.
        
        Parameters
        ----------
        targets : list
            a list of geometry_msgs/Pose for the end effector
        group_id : int, optional
            the index of the group for which to calculate the IK.  Used 
            for compatibility with future functionality; leave set to 
            default for now.
        starting_config : RobotState
            the starting state. If not provided this will default to the 
            current state of the robot.

        Returns
        ----------
        list of RobotTrajectory
            the plan for reaching the target position
        '''
        all_plans = []
        plan_start_config = starting_config
        for target in targets:
            plan = self.plan_ee_pos(target, plan_start_config,
                                    group_id=group_id)
            if plan is not None:
                if len(plan.joint_trajectory.points) != 0:
                    all_plans.append(plan)
                    plan_start_config = self.state_from_trajectory(
                        plan.joint_trajectory)
            else:
                rospy.logerr('EE waypoints could not calculate plan')
        return all_plans

    def move_through_waypoints(self, targets, is_joint_pos=False, wait=True, group_id=0):
        '''Uses MoveIt! to generate a plan then move the robot.

        Generates a plan to move the robot through the specified end effector 
        or joint pose waypoints, then moves the robot. Returns True on success,
        otherwise returns False.

        Parameters
        ----------
        targets : list
            a list of either joint positions or end effector positions. 
            See the documentation for plan_joint_waypoints and
            plan_ee_waypoints.
        is_joint_pos : bool, optional
            True if the targets are given in joint space (defaults to False)
        wait : bool, optional
            whether to return immediately or block until movement is complete. 
            Default value is True
        group_id : int, optional
            the index of the group for which to calculate the IK.  Used 
            for compatibility with future functionality; leave set to 
            default for now.

        Returns
        ----------
        bool
            True on success
        '''
        plans = self.plan_waypoints(targets, is_joint_pos, 
                                   group_id=group_id)
        if plans == None or len(plans)==0:
            rospy.logwarn('no plans generated')
            return False
        success = True
        for idx, plan in enumerate(plans):
            success = success and self.move_robot(plan, wait, group_id)
            # unfortunate hack. There appears to be a significant settling time
            # for the kinova arm.
            if not is_joint_pos and idx < (len(plans)-1):
                rospy.sleep(0.5)
        return success

    def move_through_joint_waypoints(self, targets, group_id=0):
        '''Uses MoveIt! to generate a plan then move the robot.

        Generates a plan to move the robot through the specified end effector 
        or joint pose waypoints, then moves the robot. Returns True on success,
        otherwise returns False.

        Parameters
        ----------
        targets : list
            a list of joint positions (either list or dict)
            See the documentation for plan_joint_waypoints.
        group_id : int, optional
            the index of the group for which to calculate the IK.  Used 
            for compatibility with future functionality; leave set to 
            default for now.

        Returns
        ----------
        bool
            True on success
        '''
        return self.move_through_waypoints(targets, is_joint_pos=True, 
                                    group_id=group_id)


    def move_to_joint_pose(self, target, group_id=0):
        '''Uses MoveIt! to generate a plan then move the robot.

        Generates a plan to move the robot to the specified joint pose, 
        then moves the robot. Returns True on success, otherwise returns False.

        Parameters
        ----------
        target : list or dict
            a joint position (list or dict). See the documentation for 
            plan_joint_waypoints.
        group_id : int, optional
            the index of the group for which to calculate the IK.  Used 
            for compatibility with future functionality; leave set to 
            default for now.

        Returns
        ----------
        bool
            True on success
        '''
        return self.move_to_pose(target, is_joint_pos=True, 
                                 group_id=group_id)

    def move_to_ee_pose(self, target, group_id=0):
        '''Uses MoveIt! to generate a plan then move the robot.

        Generates a plan to move the robot to the specified joint pose, 
        then moves the robot. Returns True on success, otherwise returns False.

        Parameters
        ----------
        target : geometry_msgs/Pose
            an end effector position. See the documentation for 
            plan_ee_waypoints.
        group_id : int, optional
            the index of the group for which to calculate the IK.  Used 
            for compatibility with future functionality; leave set to 
            default for now.

        Returns
        ----------
        bool
            True on success
        '''
        return self.move_to_pose(target, is_joint_pos=False, 
                                 group_id=group_id)
    
    def move_through_ee_waypoints(self, targets, group_id=0):
        '''Uses MoveIt! to generate a plan then move the robot.

        Generates a plan to move the robot through the specified end effector 
        or joint pose waypoints, then moves the robot. Returns True on success,
        otherwise returns False.

        Parameters
        ----------
        targets : list
            a list of end effector positions (geometry_msgs/Pose)
            See the documentation for plan_ee_waypoints.
        group_id : int, optional
            the index of the group for which to calculate the IK.  Used 
            for compatibility with future functionality; leave set to 
            default for now.

        Returns
        ----------
        bool
            True on success
        '''
        return self.move_through_waypoints(targets, is_joint_pos=False, 
                                    group_id=group_id)    

    def move_to_pose(self, target, is_joint_pos=False, wait=True, group_id=0):
        '''Uses MoveIt! to generate a plan then move the robot.

        Generates a plan to move the robot to the specified end effector 
        or joint pose, then moves the robot. Returns True on success,
        otherwise returns False.

        Parameters
        ----------
        target : list, dict, or geometry_msgs/Pose
            either a joint position (list or dict) or end effector position. 
            See the documentation for plan_joint_waypoints and
            plan_ee_waypoints.
        is_joint_pos : bool, optional
            True if the targets are given in joint space (defaults to False)
        wait : bool, optional
            whether to return immediately or block until movement is complete. 
            Default value is True
        group_id : int, optional
            the index of the group for which to calculate the IK.  Used 
            for compatibility with future functionality; leave set to 
            default for now.

        Returns
        ----------
        bool
            True on success
        '''
        plan = self.plan_pose(target, is_joint_pos, group_id=group_id)
        if plan != None:
            return self.move_robot(plan, wait)
        else:
            return False
    
    def plan_pose(self, target, is_joint_pos=False, starting_config=None,group_id=0):
        '''Plan a trajectory to reach a given end effector or joint pose

        Uses MoveIt! to plan a trajectory to reach a given end effector
        position or joint configuration.

        Parameters
        ----------
        target : list, dict, or geometry_msgs/Pose
            either a joint position (list or dict) or end effector position. 
            See the documentation for plan_joint_waypoints and
            plan_ee_waypoints.
        is_joint_pos : bool, optional
            True if the targets are given in joint space (defaults to False)
        group_id : int, optional
            the index of the group for which to calculate the IK.  Used 
            for compatibility with future functionality; leave set to 
            default for now.

        Returns
        ----------
        RobotTrajectory
            the plan for reaching the target position
        '''
        if is_joint_pos:
            return self.plan_joint_pos(target, starting_config, group_id)
        else:
            return self.plan_ee_pos(target, starting_config, group_id)

    def plan_waypoints(self, targets, is_joint_pos=False, 
                       merge_plans=False, starting_config=None, group_id=0):
        '''Generates a multi-segment plan to reach waypoints in target space
        
        Uses MoveIt! to generate a plan from target to target. One plan is
        generated for each segment.

        .. warning:: The plans can be stitched together, but this can have 
            unexpected issues since the plan boundaries can violate 
            acceleration limits.
        
        Parameters
        ----------
        targets : list
            a list of either joint positions or end effector positions. 
            See the documentation for plan_joint_waypoints and
            plan_ee_waypoints.
        is_joint_pos : bool, optional
            True if the targets are given in joint space (defaults to False)
        merge_plans : bool
            if True, return a single merged plan (see note above about 
            potential issues)
        group_id : int, optional
            the index of the group for which to calculate the IK.  Used 
            for compatibility with future functionality; leave set to 
            default for now.
        starting_config : RobotState
            the starting state. If not provided this will default to the 
            current state of the robot.

        Returns
        ----------
        list of RobotTrajectory
            the plan for reaching the target position
        '''
        if is_joint_pos:
            print('joint pos in plan_waypoints')
            plans = self.plan_joint_waypoints(targets, group_id, starting_config)
        else:
            plans = self.plan_ee_waypoints(targets, group_id, starting_config)
        
        if merge_plans:
            return self._merge_plans(plans)
        else:
            return plans

    def get_current_pose(self, simplify=True,group_id=0):
        '''Returns the current pose of the planning group
        
        Parameters
        ----------
        simplify : bool, optional
            whether or not to simplify continuous joints into +/- pi
        group_id : int, optional
            the index of the group for which to calculate the IK.  Used 
            for compatibility with future functionality; leave set to 
            default for now.

        Returns
        ----------
        dict
            the joint positions, mapped into (-pi,pi) if simplify
        '''
        if simplify:
            return dict(list(zip(self.group.get_active_joints(),self._simplify_joints(self.group.get_current_joint_values(), group_id))))
        else:
            return dict(list(zip(self.group.get_active_joints(),self.group.get_current_joint_values()))) 


    def state_from_joints(self, joints):
        ''' Returns a RobotState object with given joint positions

        Returns a RobotState object with the given joints
        set to the given positions.  The joints may be given
        as a dict or list object.  All other joints are taken from
        the current state.

        Parameters
        ----------
        joints : list or dict
            if a list, a list of positions for all active joints in 
            the group; if a dictionary, a mapping of joint names to 
            positions for the joints that should be moved (all other 
            joints will be held at their current angles).
        group_id : int, optional
            the index of the group for which to calculate the IK.  Used 
            for compatibility with future functionality; leave set to 
            default for now.

        Returns
        ----------
        RobotState
            A RobotState object with only the given joints changed
        '''
        state = self._copy_state()
        #print('joints: ' + str(type(joints)))
        simple_joints = self._simplify_joints(joints)
        if isinstance(joints, dict):
            joint_names = list(joints.keys())
            new_joints = [x for x in state.joint_state.position]
            
            for jname in joint_names:
                new_joints[state.joint_state.name.index(jname)]=simple_joints[jname]
            state.joint_state.position = new_joints
        elif isinstance(joints, list):
            state.joint_state.position = copy.copy(joints)
        else:
            rospy.logerr("Joints must be provided as a list or dictionary")
            raise TypeError("Joints must be provided as a list or dictionary")
        return state

    def state_from_trajectory(self, trajectory, point_idx=-1):
        ''' Returns a RobotState object with joint positions from trajectory

        Returns a RobotState object with joint positions taken from the 
        given trajectory. By default, sets the position to the last point
        in the trajectory.

        Parameters
        ----------
        trajectory : JointTrajectory
            the trajectory from which to take the joint positions.
        point_idx : int
            which point in the trajectory to take the state from. Defaults 
            to -1, taking the last point.

        Returns
        ----------
        RobotState 
            A RobotState object with state corresponding to the
            given point in the trajectory.
        '''

        state = self._copy_state()
        target = trajectory.points[point_idx]
        joints = [x for x in state.joint_state.position]
        for i in range(len(trajectory.joint_names)):
            joint_name = trajectory.joint_names[i]
            idx = state.joint_state.name.index(joint_name)
            joints[idx]=target.positions[i]
        state.joint_state.position = joints
        return state

    def _simplify_angle(self, angle):
        # Very simple function that makes sure the angles are between -pi and pi
        if angle > pi:
            while angle > pi:
                angle -= 2*pi
        elif angle < -pi:
            while angle < -pi:
                angle += 2*pi

        return angle

    def _simplify_joints(self, joints, group_id=0):
        # Helper function to convert a dictionary of joint values
        if isinstance(joints, dict):
            simplified_joints = dict()
            for joint in joints:
                # Pull out the name of the joint
                joint_name = '_'.join(joint.split('_')[1::])
                if joint_name in self.continuous_joints:
                    simplified_joints[joint] = self._simplify_angle(joints[joint])
                else:
                    simplified_joints[joint] = joints[joint]
        elif isinstance(joints, list):
            simplified_joints = []
            #separate the joint name from the group name
            joint_order = ["_".join(s.split("_")[1::]) for s in self.group.get_active_joints()]
            continuous_joints = ["_".join(s.split("_")[1::]) for s in self.group.get_active_joints()]
            
            continuous_joint_indices = [joint_order.index(j) for j in continuous_joints]

            for i in range(len(joints)):
                a = joints[i]
                if i in continuous_joint_indices:
                    simplified_joints.append(self._simplify_angle(a))
                else:
                    simplified_joints.append(a)
        else:
            rospy.logerr("Joints must be provided as a list or dictionary")
            raise TypeError("Joints must be provided as a list or dictionary")
        return simplified_joints

    def _copy_state(self):
        ''' Copies the robot state (so it can be modified to plan from
            non-current joint configurations).'''
        ## output: a copy of the robot's current state
        return copy.deepcopy(self.robot.get_current_state())

    def _merge_plans(self, plan_list, time_between=0.1):
        #check if the list is empty
        if plan_list==None or len(plan_list)==0:
            rospy.logwarn("Cannot merge plans: no plans provided")
            return plan_list
        
        all_points = []
        start_time = rospy.Duration(0)
        for plan in plan_list:
            for point in plan.joint_trajectory.points:
                new_point = copy.deepcopy(point)
                new_point.time_from_start = new_point.time_from_start+start_time
                all_points.append(new_point)
            start_time=all_points[-1].time_from_start+rospy.Duration(time_between)
                
        full_plan = copy.deepcopy(plan_list[0])
        full_plan.joint_trajectory.points=all_points
        return full_plan

def ask_scene_integration(arm):
    # Ask the user if want to integrate a box scene
    answer= input("""\n Integrate a box as a table from code ? (1 or 0)  
  (if 1: box can't be displaced nor resized by user, if 0: no scene (can always do add from rviz interface) ) \n""")
    
    if answer == 1:
        arm.box_table_scene()
        print("\n Box inserted; to see it ==> rviz interface ==> add button==> planning scene  ")
        return
    else:
        print("\n No scene added")
        return  
      
def ask_position(arm,target_pose):
    #Ask the user the values of the target position
    while True:
        try:   
            inputPosition=input(""" \n Target position coord. (format: x,y,z or write -1 to take the robot current position ): """)
            
            if inputPosition == -1:
                inputPosition=target_pose.position  
                return inputPosition
          
        except (ValueError,IOError,NameError):
            print("\n Please, enter the coordinate in the following format: x,y,z ")
            continue
        else:          
            if len(list(inputPosition)) == 3:
                poseTmp= geometry_msgs.msg.Pose()
                poseTmp.position.x=inputPosition[0]
                poseTmp.position.y=inputPosition[1]
                poseTmp.position.z=inputPosition[2]
                return poseTmp.position
            else:
                print("\n Please, enter the coordinate in the following format: x,y,z ")
                continue
            
        
def ask_orientation(arm,target_pose):
    # Ask the user the values of the target quaternion
    while True:
        try:   
            inputQuat=input(""" \n Target quaternion coordi. (format: qx,qy,qz,qw or write -1 to take the robot current quaternion ):""")
            
            if inputQuat == -1:
                inputQuat=arm.group.get_current_pose().pose.orientation                   
            return  inputQuat
          
        except (ValueError,IOError,NameError):
            print("\n Please, enter the coordinate in the following format: qx,qy,qz,qw ")
            continue
        else:
            if len(list(inputQuat)) == 4:
                poseTmp= geometry_msgs.msg.Pose()
                poseTmp.orientation.x=inputQuat[0]
                poseTmp.orientation.y=inputQuat[1]
                poseTmp.orientation.z=inputQuat[2]
                poseTmp.orientation.w=inputQuat[3]
                return poseTmp.orientation    
            else:
                print("\n Please, enter the coordinate in the following format: qx,qy,qz,qw ")
            
def main():
    arm = ArmMoveIt()

    target_pose = geometry_msgs.msg.Pose()

    ## ask if integrate object scene from code or not
    ask_scene_integration(arm)
    
    while not rospy.is_shutdown():
      
        ##   Assigned tarPose the current Pose of the robot 
        target_pose = arm.group.get_current_pose().pose

        ## ask input from user (COMMENT IF NOT USE AND WANT TO ASSIGN MANUAL VALUE IN CODE)    
        target_pose.position = ask_position(arm,target_pose)   
        target_pose.orientation = ask_orientation(arm,target_pose)  

        ##  Example of Assigned values for new targetPose to robot
        #    tarPose.position.x = 0.89
        #    tarPose.position.y = 0.00
        #    tarPose.position.z = 0.32   
        #    tarPose.orientation.x = 0.0     
   
        print('\n The target coordinate is: %s \n' %target_pose)     
    
        ## IK for target position  
        joint_target = arm.get_IK(target_pose)
        print('IK calculation step:DONE') 
    
        ## planning with joint target from IK 
        plan_traj = arm.plan_pose(joint_target, is_joint_pos=True)
        print('Planning step with target joint angles:DONE') 
    
        ## planning with pose target
        plan_traj = arm.plan_pose(target_pose, is_joint_pos=False)
    
        ## execution of the movement   
        arm.group.execute(plan_traj)
    
if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_utils', anonymous=True)
    
    main()

