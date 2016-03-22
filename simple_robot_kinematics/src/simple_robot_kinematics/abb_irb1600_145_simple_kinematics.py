#!/usr/bin/python

##################################################
#                                                #
#   Calder Phillips-Grafflin - WPI/ARC Lab       #
#                                                #
# Simple class interface to the onboard services #
# for inverse and forward kinematics on the ABB. #
#                                                #
##################################################

import rospy
import math
import random
import time

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import *
from moveit_msgs.msg import *
from moveit_msgs.srv import *


def pretty_print_moveit_error_code(error_code, label=""):
    if error_code == MoveItErrorCodes.SUCCESS:
        error_str = ""
    elif error_code == MoveItErrorCodes.FAILURE:
        error_str = "Failure"
    elif error_code == MoveItErrorCodes.PLANNING_FAILED:
        error_str = "Planning Failed"
    elif error_code == MoveItErrorCodes.INVALID_MOTION_PLAN:
        error_str = "Invalid Motion Plan"
    elif error_code == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
        error_str = "Motion Plan Invalidated By Environment Change"
    elif error_code == MoveItErrorCodes.CONTROL_FAILED:
        error_str = "Control Failed"
    elif error_code == MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA:
        error_str = "Unable To Acquire Sensor Data"
    elif error_code == MoveItErrorCodes.TIMED_OUT:
        error_str = "Timed Out"
    elif error_code == MoveItErrorCodes.PREEMPTED:
        error_str = "Preempted"
    elif error_code == MoveItErrorCodes.START_STATE_IN_COLLISION:
        error_str = "Start State In Collision"
    elif error_code == MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS:
        error_str = "Start State Violated Path Constraints"
    elif error_code == MoveItErrorCodes.GOAL_IN_COLLISION:
        error_str = "Goal In Collision"
    elif error_code == MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS:
        error_str = "Goal Violates Path Constraints"
    elif error_code == MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED:
        error_str = "Goal Constraints Violated"
    elif error_code == MoveItErrorCodes.INVALID_GROUP_NAME:
        error_str = "Invalid Group Name"
    elif error_code == MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS:
        error_str = "Invalid Goal Constraints"
    elif error_code == MoveItErrorCodes.INVALID_ROBOT_STATE:
        error_str = "Invalid Robot State"
    elif error_code == MoveItErrorCodes.INVALID_LINK_NAME:
        error_str = "Invalid Link Name"
    elif error_code == MoveItErrorCodes.INVALID_OBJECT_NAME:
        error_str = "Invalid Object Name"
    elif error_code == MoveItErrorCodes.FRAME_TRANSFORM_FAILURE:
        error_str = "Frame Transform Failure"
    elif error_code == MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE:
        error_str = "Collision Checking Unavailable"
    elif error_code == MoveItErrorCodes.ROBOT_STATE_STALE:
        error_str = "Robot State Stale"
    elif error_code == MoveItErrorCodes.SENSOR_INFO_STALE:
        error_str = "Sensor Info Stale"
    elif error_code == MoveItErrorCodes.NO_IK_SOLUTION:
        error_str = "No IK Solution"
    else:
        error_str = "UNKNOWN MOVEIT ERROR " + str(error_code)
    if error_str == "":
        rospy.loginfo("<MoveIt! Status> Success [" + label + "]")
    else:
        rospy.logerr("<MoveIt! Status> " + error_str + " [" + label + "]")


def make_arm_joint_state(left_arm_joint_positions):
    joint_state = JointState()
    joint_state.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
    joint_state.position = left_arm_joint_positions
    assert(len(joint_state.name) == len(joint_state.position))
    return joint_state


def extract_arm_from_joint_state(joint_state):
    joint_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    joint_values[0] = joint_state.position[joint_state.name.index("joint_1")]
    joint_values[1] = joint_state.position[joint_state.name.index("joint_2")]
    joint_values[2] = joint_state.position[joint_state.name.index("joint_3")]
    joint_values[3] = joint_state.position[joint_state.name.index("joint_4")]
    joint_values[4] = joint_state.position[joint_state.name.index("joint_5")]
    joint_values[5] = joint_state.position[joint_state.name.index("joint_6")]
    return joint_values


class SimpleInverseKinematics:

    def __init__(self):
        rospy.loginfo("Connecting to MoveIt! IK service...")
        rospy.wait_for_service("compute_ik")
        self.ikClient = rospy.ServiceProxy("compute_ik", GetPositionIK)
        rospy.loginfo("...IK service loaded")

    def compute_arm_ik(self, desired_wrist_pose_stamped, seed_state=None, collision_aware=False, raw_output=False):
        request = GetPositionIKRequest()
        request.ik_request.group_name = "arm"
        request.ik_request.pose_stamped = desired_wrist_pose_stamped
        request.ik_request.avoid_collisions = collision_aware
        if seed_state is not None:
            if type(seed_state) == RobotState:
                request.ik_request.robot_state = seed_state
            elif type(seed_state) == JointState:
                request.ik_request.robot_state.joint_state = seed_state
            elif type(seed_state) == list:
                request.ik_request.robot_state.joint_state = make_arm_joint_state(seed_state)
            else:
                raise AttributeError("Provided seed_state is an invalid type")
        try:
            response = self.ikClient.call(request)
        except:
            rospy.logerr("IK service call failed to connect to IK server")
            response = None
        if response is not None:
            # Check the error state
            if response.error_code.val == MoveItErrorCodes.SUCCESS:
                if raw_output:
                    return response.solution
                else:
                    return extract_arm_from_joint_state(response.solution.joint_state)
            else:
                pretty_print_moveit_error_code(response.error_code.val, "ABB IRB 1600-120 arm IK")
                return None
        else:
            return None


class SimpleForwardKinematics:

    def __init__(self):
        rospy.loginfo("Connecting to MoveIt! FK service...")
        rospy.wait_for_service("compute_fk")
        self.fkClient = rospy.ServiceProxy("compute_fk", GetPositionFK)
        rospy.loginfo("...FK service loaded")

    def compute_fk(self, state, fk_frame_id, fk_link_names):
        request = GetPositionFKRequest()
        request.header.frame_id = fk_frame_id
        # First, figure out the type of the provided link names and process accordingly
        if type(fk_link_names) == list:
            request.fk_link_names = fk_link_names
        elif type(fk_link_names) == str:
            request.fk_link_names = [fk_link_names]
        else:
            raise AttributeError("Provided fk_link_names is an invalid type")
        # Second, figure out the type of the provided state and process accordingly
        if type(state) == RobotState:
            request.robot_state = state
        elif type(state) == JointState:
            request.robot_state.joint_state = state
        else:
            raise AttributeError("Provided state is an invalid type")
        # Call the FK server
        try:
            response = self.fkClient.call(request)
        except:
            rospy.logerr("FK service call failed to connect to FK server")
            response = None
        if response is not None:
            # Check the error state
            if response.error_code.val == MoveItErrorCodes.SUCCESS:
                if len(response.fk_link_names) == 1:
                    return response.pose_stamped[0]
                else:
                    return response.pose_stamped
            else:
                pretty_print_moveit_error_code(response.error_code.val, "ABB IRB 1600-120 FK")
                return None
        else:
            return None

    def compute_arm_fk(self, state, fk_frame_id, fk_link_names):
        # First, figure out the type of the provided state and process accordingly
        if type(state) == RobotState:
            return self.compute_fk(state, fk_frame_id, fk_link_names)
        elif type(state) == JointState:
            return self.compute_fk(state, fk_frame_id, fk_link_names)
        elif type(state) == list:
            return self.compute_fk(make_arm_joint_state(state), fk_frame_id, fk_link_names)
        else:
            raise AttributeError("Provided state is an invalid type")
