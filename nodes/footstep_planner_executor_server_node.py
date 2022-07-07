#!/usr/bin/env python3
"""
Footstep Planner Executor Server Node
Emily Sheetz, Summer 2022

NOTE: this node handles planning and executing for both waypoints and stances
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

# ROS imports
import rospy
import actionlib
from valkyrie_navigation.msg import WaypointToFootstepsAction, WaypointToFootstepsGoal, WaypointToFootstepsResult, WaypointToFootstepsFeedback
from valkyrie_navigation.srv import TransformProvider
from controller_msgs.msg import FootstepDataListMessage
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from std_msgs.msg import Bool, Header
from val_footstep_planner_executor.srv import PlanToWaypoint, PlanToWaypointRequest, PlanToWaypointResponse
from val_footstep_planner_executor.srv import ExecuteToWaypoint, ExecuteToWaypointRequest, ExecuteToWaypointResponse
from val_footstep_planner_executor.srv import PlanToStance, PlanToStanceRequest, PlanToStanceResponse
from val_footstep_planner_executor.srv import ExecuteToStance, ExecuteToStanceRequest, ExecuteToStanceResponse

class FootstepPlannerExecutorServerNode:

    def __init__(self):
        # set action server and service names
        self.footstep_planner_action_server_name = 'ValNavFootstepPlannerInterfaceServer'
        self.footstep_executor_topic_name = '/ihmc/valkyrie/humanoid_control/input/footstep_data_list'
        # self.transform_provider_service_name = '/nasa/transform_provider'
        self.moveit_forward_kinematics_service_name = 'get_fk'
        self.stance_gen_robot_state_subscriber_name = '/nasa/stance_generation/result/robot_state'
        self.stance_gen_goal_response_subscriber_name = '/nasa/stance_generation/response/vr'
        self.stance_gen_show_stance_response_subscriber_name = '/nasa/stance_generation/show_stance/response/vr'
        self.stance_gen_adjust_stance_response_subscriber_name = '/nasa/stance_generation/adjust_stance/response/vr'

        # create simple action client for WaypointToFootstepsAction
        self.footstep_planner_client = actionlib.SimpleActionClient(self.footstep_planner_action_server_name, WaypointToFootstepsAction)

        # wait until the action server has started up and started listening for goals
        rospy.loginfo("[Footstep Planner Executor Server Node] Waiting for action server %s..." % self.footstep_planner_action_server_name)
        self.footstep_planner_client.wait_for_server()
        rospy.loginfo("[Footstep Planner Executor Server Node] Action server %s is ready" % self.footstep_planner_action_server_name)

        # initialize publisher for footstep execution
        self.footstep_executor_pub = rospy.Publisher(self.footstep_executor_topic_name, FootstepDataListMessage, queue_size=1)

        # # wait until transform provider service has started up and started listening for requests
        # rospy.loginfo("[Footstep Planner Executor Server Node] Waiting for ROS service %s..." % self.transform_provider_service_name)
        # rospy.wait_for_service(self.transform_provider_service_name)
        # rospy.loginfo("[Footstep Planner Executor Server Node] ROS service %s is ready" % self.transform_provider_service_name)

        # # server for transforms
        # self.transform_client = rospy.ServiceProxy(self.transform_provider_service_name, TransformProvider)

        # wait until FK service has started up and started listening for requests
        rospy.loginfo("[Footstep Planner Executor Server Node] Waiting for ROS service %s..." % self.moveit_forward_kinematics_service_name)
        rospy.wait_for_service(self.moveit_forward_kinematics_service_name)
        rospy.loginfo("[Footstep Planner Executor Server Node] ROS service %s is ready" % self.moveit_forward_kinematics_service_name)

        # server for FK
        self.forward_kinematics_client = rospy.ServiceProxy(self.moveit_forward_kinematics_service_name, GetPositionFK)

        # initialize stance gen flags
        self.stance_gen_robot_state = None
        self.stance_gen_result = False
        self.stance_gen_adjust = False

        # create stance gen response subscribers
        rospy.Subscriber(self.stance_gen_robot_state_subscriber_name, RobotState, self.stance_gen_robot_state_callback)
        rospy.Subscriber(self.stance_gen_goal_response_subscriber_name, Bool, self.stance_gen_goal_response_callback)
        rospy.Subscriber(self.stance_gen_show_stance_response_subscriber_name, Bool, self.stance_gen_show_response_callback)
        rospy.Subscriber(self.stance_gen_adjust_stance_response_subscriber_name, Bool, self.stance_gen_adjust_response_callback)

        rospy.loginfo("[Footstep Planner Executor Server Node] Ready to go!")

    ###################################
    ### STANCE GENERATION CALLBACKS ###
    ###################################

    def stance_gen_robot_state_callback(self, msg):
        # got robot state, store it internally
        self.stance_gen_robot_state = msg
        return

    def stance_gen_goal_response_callback(self, msg):
        # stance gen response sent
        self.reset_stance_gen_flags()
        self.stance_gen_result = msg.data
        return

    def stance_gen_show_response_callback(self, msg):
        # stance gen show stance response sent
        # stance gen goal response should have been sent previously
        self.stance_gen_result = (self.stance_gen_result and msg.data)
        return

    def stance_gen_adjust_response_callback(self, msg):
        # stance gen adjust stance response sent
        self.stance_gen_adjust = msg.data
        return

    def reset_stance_gen_flags(self):
        self.stance_gen_result = False
        self.stance_gen_adjust = False
        return

    ##########################
    ### NODE FUNCTIONALITY ###
    ##########################

    # def get_transform(self, parent_frame, child_frame):
    #   try:
    #       # request transform of child frame w.r.t. parent frame
    #       result = self.transform_client(rospy.Time(0), parent_frame, child_frame)
    #       # get transform from result
    #       t = result.transform.transform
    #       # get position and rotation
    #       pos = t.translation
    #       rot = t.rotation
    #       # return success and position and rotation as lists
    #       return (True, [[pos.x, pos.y, pos.z], [rot.x, rot.y, rot.z, rot.w]])
    #   except rospy.ServiceException as e:
    #       return (False, [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]])

    def get_plan(self, goal):
        # send the goal to the action server
        self.footstep_planner_client.send_goal(goal)

        # wait for the server to perform the action
        self.footstep_planner_client.wait_for_result()

        # get the result
        result = self.footstep_planner_client.get_result()

        # check that footsteps were received
        if len(result.footstep_toolbox_output.footstep_data_list.footstep_data_list) > 0:
            rospy.loginfo("[Footstep Planner Executor Server Node] Footsteps planned successfully! Found %d footsteps to goal." % len(result.footstep_toolbox_output.footstep_data_list.footstep_data_list))
            footstep_plan = result.footstep_toolbox_output.footstep_data_list
            return (True, footstep_plan)
        else:
            rospy.logwarn("[Footstep Planner Executor Server Node] Footsteps not planned successfully.")
            return (False, FootstepDataListMessage()) # return empty footstep data list message

    def execute_plan(self, footstep_plan):
        # verify that we have a plan
        if len(footstep_plan.footstep_data_list) > 0:
            # publish message
            self.footstep_executor_pub.publish(footstep_plan)
            rospy.loginfo("[Footstep Planner Executor Server Node] Sent exeuction message!")
            return True
        else:
            rospy.logwarn("[Footstep Planner Executor Server Node] Plan does not exist; cannot execute.")
            return False

    ##########################################
    ### PLANNING AND EXECUTING TO WAYPOINT ###
    ##########################################

    def create_footstep_goal_from_waypoint(self, waypoint_pos, waypoint_quat):
        # create action server goal
        goal = WaypointToFootstepsGoal()

        # set waypoint goal
        goal.set_waypoint_goal.position.x = waypoint_pos[0]
        goal.set_waypoint_goal.position.y = waypoint_pos[1]
        goal.set_waypoint_goal.position.z = waypoint_pos[2]
        goal.set_waypoint_goal.orientation.x = waypoint_quat[0]
        goal.set_waypoint_goal.orientation.y = waypoint_quat[1]
        goal.set_waypoint_goal.orientation.z = waypoint_quat[2]
        goal.set_waypoint_goal.orientation.w = waypoint_quat[3]

        # set initial stance foot
        goal.initial_stance_robot_side = WaypointToFootstepsGoal.ROBOT_SIDE_LEFT

        # set planner type
        # goal.requested_footstep_planner_type = WaypointToFootstepsGoal.FOOTSTEP_PLANNER_TYPE_A_STAR

        # set timeout
        goal.timeout = 10.0 # 10 seconds to wait for plan; same as VR

        # set assume flat ground
        goal.assume_flat_ground = True

        return goal

    def request_plan_to_waypoint(self, waypoint_pose):
        # convert waypoint pose to lists for position and orientation
        waypoint_pos = [waypoint_pose.position.x,
                        waypoint_pose.position.y,
                        waypoint_pose.position.z]
        waypoint_quat = [waypoint_pose.orientation.x,
                         waypoint_pose.orientation.y,
                         waypoint_pose.orientation.z,
                         waypoint_pose.orientation.w]

        # create action server goal
        goal = self.create_footstep_goal_from_waypoint(waypoint_pos, waypoint_quat)

        # get plan from goal pose
        return self.get_plan(goal)

    def execute_plan_to_waypoint(self, footstep_plan):
        # execute planned footsteps
        return self.execute_plan(footstep_plan)

    ########################################
    ### PLANNING AND EXECUTING TO STANCE ###
    ########################################

    def get_forward_kinematics(self, parent_frame, child_frame_list):
        # need adjusted stance gen robot state, check if state has been received
        if not (self.stance_gen_result and self.stance_gen_adjust):
            rospy.logwarn("[Footstep Planner Executor Server Node] Stance gen robot state is not ready")
            return (False, None)

        # adjusted stance gen robot state exists, try getting forward kinematics
        try:
            # set header
            header = Header()
            header.frame_id = parent_frame
            # request transform of child frames w.r.t. parent frame
            result = self.forward_kinematics_client(header, child_frame_list, self.stance_gen_robot_state)
        except rospy.ServiceException as e:
            rospy.logwarn("[Footstep Planner Executor Server Node] Forward kinematics service call failed: %s" % e)
            return (False, None)

        # got result of FK, format as list of positions and rotations
        fk_result_list = []
        for pose_msg in result.pose_stamped:
            pos = pose_msg.pose.position
            rot = pose_msg.pose.orientation
            fk_result_list.append([[pos.x, pos.y, pos.z], [rot.x, rot.y, rot.z, rot.w]])

        return (True, fk_result_list)

    def create_waypoint_from_stance(self):
        # get pose of target left and right feet
        (success, feet_poses) = self.get_forward_kinematics("world", ["leftFoot", "rightFoot"])

        # check if transform lookup was successful
        if not success:
            rospy.logwarn("[Footstep Planner Executor Server Node] Cannot compute poses of feet")
            return (None, None)

        # successfully computed feet poses, unpack message
        [pos_left, rot_left] = feet_poses[0]
        [pos_right, rot_right] = feet_poses[1]

        # compute midfeet position
        pos_left = np.array(pos_left)
        pos_right = np.array(pos_right)
        midfeet_pos = 0.5 * (pos_left + pos_right)

        # compute midfeet orientation
        slerp_interpolator = Slerp([0, 1], R.from_quat([rot_left, rot_right]))
        midfeet_rot = slerp_interpolator(0.5)
        midfeet_euler = midfeet_rot.as_euler('xyz')
        # zero out roll and pitch, we only care about yaw
        midfeet_euler[0] = 0.0
        midfeet_euler[1] = 0.0
        # convert to quaternion
        midfeet_rot = R.from_euler('xyz', [midfeet_euler[0], midfeet_euler[1], midfeet_euler[2]])
        midfeet_quat = midfeet_rot.as_quat()

        # return waypoint as midfeet frame
        return (midfeet_pos, midfeet_quat)

    def create_waypoint_from_stance_to_footstep_goal(self):
        # get the waypoint based on the midfeet frame of the IK adjusted robot model
        waypoint_pos, waypoint_quat = self.create_waypoint_from_stance()

        # check that creating waypoint worked
        if (waypoint_pos is None) or (waypoint_quat is None):
            rospy.logwarn("[Footstep Planner Executor Server Node] Could not create waypoint")
            return None

        # create action server goal
        goal = self.create_footstep_goal_from_waypoint(waypoint_pos, waypoint_quat)

        return goal

    def request_plan_to_stance(self):
        # get action server goal
        goal = self.create_waypoint_from_stance_to_footstep_goal()

        # check if goal created
        if goal is None:
            rospy.logwarn("[Footstep Planner Executor Server Node] Could not create action server goal; not constructing plan")
            return (False, FootstepDataListMessage())

        # get plan from goal pose
        return self.get_plan(goal)

    def execute_plan_to_stance(self, footstep_plan):
        # execute planned footsteps
        return self.execute_plan(footstep_plan)

    #########################
    ### SERVICE CALLBACKS ###
    #########################

    def plan_to_waypoint_request_callback(self, req):
        # initialize response
        res = PlanToWaypointResponse()

        # plan based on given waypoint
        succ, footstep_plan = self.request_plan_to_waypoint(req.waypoint_pose)

        # set response fields
        res.success = succ
        res.planned_footsteps = footstep_plan

        # return result
        return res

    def execute_to_waypoint_request_callback(self, req):
        # initialize response
        res = ExecuteToWaypointResponse()

        # execute planned footsteps
        succ = self.execute_plan_to_waypoint(req.planned_footsteps)

        # set response fields
        res.success = succ

        # return result
        return res

    def plan_to_stance_request_callback(self, req):
        # initialize response
        res = PlanToStanceResponse()

        # verify that request wants service to plan
        if req.request:
            # plan based on received stances
            succ, footstep_plan = self.request_plan_to_stance()
            # set response fields
            res.success = succ
            res.planned_footsteps = footstep_plan
        else:
            # otherwise, send response indicating no success
            res.success = False

        # return result
        return res

    def execute_to_stance_request_callback(self, req):
        # initialize response
        res = ExecuteToStanceResponse()

        # execute planned footsteps
        succ = self.execute_plan_to_stance(req.planned_footsteps)

        # set response fields
        res.success = succ

        # return result
        return res

#####################
### MAIN FUNCTION ###
#####################

if __name__ == '__main__':
    # initialize server node
    rospy.init_node("ValkyrieFootstepPlannerExecutorServerNode")

    # create server node
    server_node = FootstepPlannerExecutorServerNode()

    # advertise services
    plan_to_waypoint_service = rospy.Service("plan_to_waypoint", PlanToWaypoint,
                                             server_node.plan_to_waypoint_request_callback)
    execute_to_waypoint_service = rospy.Service("execute_to_waypoint", ExecuteToWaypoint,
                                                server_node.execute_to_waypoint_request_callback)
    plan_to_stance_service = rospy.Service("plan_to_stance", PlanToStance,
                                           server_node.plan_to_stance_request_callback)
    execute_to_stance_service = rospy.Service("execute_to_stance", ExecuteToStance,
                                              server_node.execute_to_stance_request_callback)

    rospy.loginfo("[Footstep Planner Executor Server Node] Providing services for planning and executing footsteps!")

    # run node, wait for requests
    while not rospy.is_shutdown():
        rospy.spin()

    rospy.loginfo("[Footstep Planner Executor Server Node] Node stopped, all done!")
