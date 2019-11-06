#!/usr/bin/env python

import sys
import copy
import rospy
import rospkg
import os
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from time import sleep
from std_msgs.msg import String, Int8MultiArray
from moveit_commander.conversions import pose_to_list
from gripper_to_position import gripper_to_pos, reset_gripper, activate_gripper
from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
    SpawnModelRequest,
    SpawnModelResponse
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from copy import deepcopy
from tf.transformations import quaternion_from_euler

import intera_interface
from robotiq_2f_gripper_control.srv import move_robot, move_robotResponse
import numpy
from sawyer_irl_project.msg import onions_blocks_poses
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

# END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal: A list of floats, a Pose or a PoseStamped
    @param: actual: list of floats, a Pose or a PoseStamped
    @param: tolerance: A float
    """

    all_equal = True
    if type(goal) is list:
        for index in [0, 1, 2]:
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        # print "pose_to_list(goal):"+str(pose_to_list(goal))
        # print "pose_to_list(actual):"+str(pose_to_list(actual))
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


joint_state_topic = ['joint_states:=/robot/joint_states']

# a global class object
limb = 'right'
# intermediate variable needed for avoiding collisions
step_distance = 0.1  # meters
tip_name = "right_gripper_tip"
target_location_x = -1
target_location_y = -1
# at HOME position, orientation of gripper frame w.r.t world x=0.7, y=0.7, z=0.0, w=0.0 or [ rollx: -3.1415927, pitchy: 0, yawz: -1.5707963 ]
# (roll about an X-axis w.r.t home) / (subsequent pitch about the Y-axis) / (subsequent yaw about the Z-axis)
rollx = 3.30
pitchy = 0.0
yawz = -1.57
# use q with moveit because giving quaternion.x doesn't work.
q = quaternion_from_euler(rollx, pitchy, yawz)
overhead_orientation_moveit = Quaternion(
    x=q[0],
    y=q[1],
    z=q[2],
    w=q[3])
MOTION_SAMPLE_TIME = 0.025


class PickAndPlace(object):
    def __init__(self, limb, step_distance, tip_name):
        super(PickAndPlace, self).__init__()

        moveit_commander.roscpp_initialize(joint_state_topic)

        rospy.init_node('simple_pnp_gazebo',
                        anonymous=True, disable_signals=False)

        moveit_commander.roscpp_initialize(sys.argv)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "right_arm"
        group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        planning_frame = group.get_planning_frame()

        eef_link = group.get_end_effector_link()

        group_names = robot.get_group_names()

        print robot.get_current_state()

        # now for intera
        print("Getting robot state... ")
        self._rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self._limb_name = limb  # string
        self._limb = intera_interface.Limb(limb)
        self._step_distance = step_distance
        self._tip_name = tip_name

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles, timeout=2.0)
        return True

    def _guarded_move_to_joint_position(self, joint_angles, timeout=5.0):
        if rospy.is_shutdown():
            return
        if joint_angles:
            #move_to_joint_positions(self, positions, timeout=15.0,threshold=settings.JOINT_ANGLE_TOLERANCE,test=None)
            self._limb.move_to_joint_positions(joint_angles, timeout=timeout)
        else:
            rospy.logerr(
                "No Joint Angles provided for move_to_joint_positions. Staying put.")

    def _approach_with_leeway(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._step_distance
        print "finding ik for "+str((approach.position.x, approach.position.y, approach.position.z))
        joint_angles = self._limb.ik_request(approach, self._tip_name)
        # print str(joint_angles)+" is output from limb.ik_request "
        self._limb.set_joint_position_speed(0.001)
        self._guarded_move_to_joint_position(joint_angles, timeout=1.0)
        self._limb.set_joint_position_speed(0.1)
        if joint_angles:
            return True
        else:
            return False

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        joint_angles = self._limb.ik_request(approach, self._tip_name)
        self._limb.set_joint_position_speed(0.001)
        self._guarded_move_to_joint_position(joint_angles, timeout=5.0)
        self._limb.set_joint_position_speed(0.1)
        if joint_angles:
            return True
        else:
            return False

    def _servo_to_pose(self, current_pose, pose, time=4.0, steps=400.0):
        ''' An *incredibly simple* linearly-interpolated Cartesian move '''
        r = rospy.Rate(1/(time/steps))  # Defaults to 100Hz command rate
        # current_pose = self._limb.endpoint_pose()
        print "current_pose: "+str((current_pose['position'].x, current_pose['position'].y, current_pose['position'].z))
        ik_delta = Pose()
        ik_delta.position.x = (
            current_pose['position'].x - pose.position.x) / steps
        ik_delta.position.y = (
            current_pose['position'].y - pose.position.y) / steps
        ik_delta.position.z = (
            current_pose['position'].z - pose.position.z) / steps
        ik_delta.orientation.x = (
            current_pose['orientation'].x - pose.orientation.x) / steps
        ik_delta.orientation.y = (
            current_pose['orientation'].y - pose.orientation.y) / steps
        ik_delta.orientation.z = (
            current_pose['orientation'].z - pose.orientation.z) / steps
        ik_delta.orientation.w = (
            current_pose['orientation'].w - pose.orientation.w) / steps
        for d in range(int(steps), -1, -1):
            if rospy.is_shutdown():
                return
            ik_step = Pose()
            ik_step.position.x = d*ik_delta.position.x + pose.position.x
            ik_step.position.y = d*ik_delta.position.y + pose.position.y
            ik_step.position.z = d*ik_delta.position.z + pose.position.z
            ik_step.orientation.x = d*ik_delta.orientation.x + pose.orientation.x
            ik_step.orientation.y = d*ik_delta.orientation.y + pose.orientation.y
            ik_step.orientation.z = d*ik_delta.orientation.z + pose.orientation.z
            ik_step.orientation.w = d*ik_delta.orientation.w + pose.orientation.w
            print "finding angles for "+str((ik_step.position.x, ik_step.position.y, ik_step.position.z))
            joint_angles = self._limb.ik_request(ik_step, self._tip_name)
            while joint_angles == False:
                r.sleep()
                r.sleep()
                joint_angles = self._limb.ik_request(ik_step, self._tip_name)
            self._limb.set_joint_positions(joint_angles)
            r.sleep()
            # print("These are the joint angles I got: ",joint_angles)
            # if joint_angles:
            #     self._limb.set_joint_positions(joint_angles)
            # else:
            #     rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
            r.sleep()
        rospy.sleep(1.0)
        return True

    #Why do we have the exact same code as _servo to pose under this????
    def moveit_servo_to_pose(self, current_pose, pose, time=4.0, steps=400.0):
        ''' An *incredibly simple* linearly-interpolated Cartesian move '''
        r = rospy.Rate(1/(time/steps))  # Defaults to 100Hz command rate
        # current_pose = self.group.get_current_pose().pose
        ik_delta = Pose()
        ik_delta.position.x = (
            current_pose.position.x - pose.position.x) / steps
        ik_delta.position.y = (
            current_pose.position.y - pose.position.y) / steps
        ik_delta.position.z = (
            current_pose.position.x - pose.position.z) / steps
        ik_delta.orientation.x = (
            current_pose.orientation.x - pose.orientation.x) / steps
        ik_delta.orientation.y = (
            current_pose.orientation.y - pose.orientation.y) / steps
        ik_delta.orientation.z = (
            current_pose.orientation.z - pose.orientation.z) / steps
        ik_delta.orientation.w = (
            current_pose.orientation.w - pose.orientation.w) / steps
        for d in range(int(steps), -1, -1):
            if rospy.is_shutdown():
                return
            ik_step = Pose()
            ik_step.position.x = d*ik_delta.position.x + pose.position.x
            ik_step.position.y = d*ik_delta.position.y + pose.position.y
            ik_step.position.z = d*ik_delta.position.z + pose.position.z
            ik_step.orientation.x = d*ik_delta.orientation.x + pose.orientation.x
            ik_step.orientation.y = d*ik_delta.orientation.y + pose.orientation.y
            ik_step.orientation.z = d*ik_delta.orientation.z + pose.orientation.z
            ik_step.orientation.w = d*ik_delta.orientation.w + pose.orientation.w
            flag = False
            while not flag:
                self.go_to_pose_goal(q[0], q[1], q[2], q[3],
                                     ik_step.position.x, ik_step.position.y, ik_step.position.z)
                # self.go_to_pose_goal(ik_step.orientation.x,ik_step.orientation.y,
                #   ik_step.orientation.z,ik_step.orientation.w,
                # ik_step.position.x,ik_step.position.y,ik_step.position.z)
                # sleep(1.0)
                current_pose_step = self.group.get_current_pose().pose
                flag = (not all_close(ik_step, current_pose_step, 0.01))
                r.sleep()
            # print("These are the joint angles I got: ",joint_angles)
            # if joint_angles:
            #     self._limb.set_joint_positions(joint_angles)
            # else:
            #     rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
            # r.sleep()
        rospy.sleep(1.0)
        return True

    def go_to_joint_goal(self, angles, allow_replanning=True, planning_time=5.0,
                         goal_tol=0.02, orientation_tol=0.02):

        group = self.group
        # Allow some leeway in position(meters) and orientation (radians)
        group.set_goal_position_tolerance(goal_tol)
        group.set_goal_orientation_tolerance(goal_tol)
        group.allow_replanning(allow_replanning)
        group.set_planning_time(planning_time)
        group.go(angles, wait=True)
        group.stop()
        return True

    def go_to_pose_goal(self, ox, oy, oz, ow, px, py, pz, allow_replanning=True, planning_time=5.0):
        """
        Movement method to go to desired end effector pose
        @param: ox: Pose orientation for the x-axis (part of Quaternion)
        @param: oy: Pose orientation for the y-axis (part of Quaternion)
        @param: oz: Pose orientation for the z-axis (part of Quaternion)
        @param: ow: Pose orientation for the w (part of Quaternion)
        @param: px: Coordinate on the x-axis 
        @param: py: Coordinate on the y-axis
        @param: pz: Coordinate on the z-axis
        """
        group = self.group
        # Allow some leeway in position(meters) and orientation (radians)
        group.set_goal_position_tolerance(0.01)
        group.set_goal_orientation_tolerance(0.02)

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = ox
        pose_goal.orientation.y = oy
        pose_goal.orientation.z = oz
        pose_goal.orientation.w = ow
        pose_goal.position.x = px
        pose_goal.position.y = py
        pose_goal.position.z = pz
        group.set_pose_target(pose_goal)
        group.allow_replanning(allow_replanning)
        group.set_planning_time(planning_time)
        # group.set_planning_time(0.5)
        plan = group.go(wait=True)
        # rospy.sleep(1)
        group.stop()

        group.clear_pose_targets()

        current_pose = self.group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.02)

    def display_trajectory(self, plan):
        """
        Display a movement plan / trajectory
        @param: plan: Plan to be displayed
        """
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)

        display_trajectory_publisher.publish(display_trajectory)


############################## Need a global class object ###########################################
pnp = PickAndPlace(limb, step_distance, tip_name)

def handle_move_sawyer(req_move_robot):
    # "home","bin","hover","lowergripper","lookNrotate","roll"
    # location - (x,y)
    target_location_x = req_move_robot.location.x
    target_location_y = req_move_robot.location.y
    argument = req_move_robot.primitive
    switcher = {
        "home": goto_home,
        "bin": goto_bin,
        "hover": hover,
        "lowergripper": lowergripper,
        "lookNrotate": lookNrotate,
        "roll": roll
    }
    # Get the function from switcher dictionary
    func = switcher.get(
        argument, lambda: "Invalid input to move_sawyer service")
    if argument not in switcher.keys():
        return move_robotResponse(False)
    # Execute the function
    result = func()
    if result == True:
        return move_robotResponse(True)
    return move_robotResponse(False)


def goto_home(tolerance=0.1, goal_tol=0.02,
              orientation_tol=0.02):
    global pnp

    home_joint_angles = [-0.041662954890248294, -1.0258291091425074, 0.0293680414401436,
                         2.17518162913313, -0.06703022873354225, 0.3968371433926965, 1.7659649178699421]
    # pnp.go_to_joint_goal(home_joint_angles,True,5.0)
    # sleep(10.0)

    joint_angles = {'right_j0': -0.041662954890248294,
                    'right_j1': -1.0258291091425074,
                    'right_j2': 0.0293680414401436,
                    'right_j3': 2.17518162913313,
                    'right_j4': -0.06703022873354225,
                    'right_j5': 0.3968371433926965,
                    'right_j6': 1.7659649178699421}
    current_joints = pnp.group.get_current_joint_values()
    tol = tolerance
    diff = abs(joint_angles['right_j0']-current_joints[0]) > tol or \
        abs(joint_angles['right_j1']-current_joints[1]) > tol or \
        abs(joint_angles['right_j2']-current_joints[2]) > tol or \
        abs(joint_angles['right_j3']-current_joints[3]) > tol or \
        abs(joint_angles['right_j4']-current_joints[4]) > tol or \
        abs(joint_angles['right_j5']-current_joints[5]) > tol or \
        abs(joint_angles['right_j6']-current_joints[6]) > tol

    while diff:
        pnp.go_to_joint_goal(home_joint_angles, True, 5.0, goal_tol=goal_tol,
                             orientation_tol=orientation_tol)
        sleep(5.0)
        # measure after movement
        current_joints = pnp.group.get_current_joint_values()

        diff = abs(joint_angles['right_j0']-current_joints[0]) > tol or \
            abs(joint_angles['right_j1']-current_joints[1]) > tol or \
            abs(joint_angles['right_j2']-current_joints[2]) > tol or \
            abs(joint_angles['right_j3']-current_joints[3]) > tol or \
            abs(joint_angles['right_j4']-current_joints[4]) > tol or \
            abs(joint_angles['right_j5']-current_joints[5]) > tol or \
            abs(joint_angles['right_j6']-current_joints[6]) > tol
        if diff:
            pnp.move_to_start(joint_angles)
            sleep(5.0)

        print "diff:"+str(diff)

    print("reached home")
    return True
    # if diff:
    #   return pnp.move_to_start(joint_angles)
    # else:
    #   return True


def hover(hovertime=3, displacement_for_picking=0.03):
    global q, pnp, target_location_x, target_location_y
    while target_location_x == -100:
        sleep(0.05)

    # pnp._limb.set_joint_position_speed(40)
    allow_replanning = False
    planning_time = MOTION_SAMPLE_TIME
    reached = pnp.go_to_pose_goal(q[0], q[1], q[2], q[3], target_location_x,
                                  target_location_y+displacement_for_picking+0.01, 0.14,
                                  allow_replanning, planning_time)
    rospy.sleep(0.2)

    allow_replanning = False
    planning_time = MOTION_SAMPLE_TIME/2
    t = 0
    count = int(hovertime / MOTION_SAMPLE_TIME)
    for t in range(0, count):
        # joint_angles = pnp._limb.ik_request(Pose(Point(x=target_location_x,y=target_location_y,z=0.12),\
        #   Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])), pnp._tip_name)
        # pnp._guarded_move_to_joint_position(joint_angles,timeout=MOTION_SAMPLE_TIME)
        reached = pnp.go_to_pose_goal(q[0], q[1], q[2], q[3], target_location_x,
                                      target_location_y+displacement_for_picking, 0.14,
                                      allow_replanning, planning_time)
        rospy.sleep(MOTION_SAMPLE_TIME)

    return True


def lowergripper(displacement_for_picking=0.01):
    # approx centers of onions at 0.82, width of onion is 0.038 m. table is at 0.78
    # length of gripper is 0.163 m The gripper should not go lower than
    # (height_z of table w.r.t base+gripper-height/2+tolerance) = 0.78-0.93+0.08+0.01=-0.24
    # pnp._limb.endpoint_pose returns {'position': (x, y, z), 'orientation': (x, y, z, w)}
    # moving from z=-.02 to z=-0.1
    global overhead_orientation, pnp, target_location_x, target_location_y
    z_array = [0.1, 0.07, 0.05]

    allow_replanning = False
    planning_time = MOTION_SAMPLE_TIME
    for z in z_array:
        # calculating how much is base frame off
        reached = pnp.go_to_pose_goal(q[0], q[1], q[2], q[3], target_location_x-0.02,
                                      target_location_y+displacement_for_picking, z,
                                      allow_replanning, planning_time)
        print "at z="+str(z)
    return True


def liftgripper(displacement_for_picking=0.02):
    # approx centers of onions at 0.82, width of onion is 0.038 m. table is at 0.78
    # length of gripper is 0.163 m The gripper should not go lower than
    # (height_z of table w.r.t base+gripper-height/2+tolerance) = 0.78-0.93+0.08+0.01=-0.24
    # pnp._limb.endpoint_pose returns {'position': (x, y, z), 'orientation': (x, y, z, w)}
    # moving from z=-.02 to z=-0.1

    global overhead_orientation, pnp, target_location_x, target_location_y
    z_array = [0.07, 0.1, 0.14]

    allow_replanning = False
    planning_time = MOTION_SAMPLE_TIME
    for z in z_array:
        reached = pnp.go_to_pose_goal(q[0], q[1], q[2], q[3], target_location_x-0.02,
                                      target_location_y+displacement_for_picking, z,
                                      allow_replanning, planning_time)
        print "at z="+str(z)
    return True


def lookNrotate():
    # go to inspection position and rotate

    goto_home(0.4, goal_tol=0.1,
              orientation_tol=0.1)
    sleep(5.0)

    joint_angles = {}
    joint_goal = [-0.04, -1.015, 0.019, 2.15, -0.06, -2, 1.76]
    joint_angles['right_j0'] = joint_goal[0]
    joint_angles['right_j1'] = joint_goal[1]
    joint_angles['right_j2'] = joint_goal[2]
    joint_angles['right_j3'] = joint_goal[3]
    joint_angles['right_j4'] = joint_goal[4]
    joint_angles['right_j5'] = joint_goal[5]
    joint_angles['right_j6'] = joint_goal[6]
    pnp._guarded_move_to_joint_position(joint_angles, timeout=3.0)
    # pnp.go_to_joint_goal(joint_goal,True,5.0,goal_tol=0.03,orientation_tol=0.03)
    sleep(5.0)
    print("object up")

    joint_goal = pnp.group.get_current_joint_values()
    joint_angles['right_j0'] = joint_goal[0]
    joint_angles['right_j1'] = joint_goal[1]
    joint_angles['right_j2'] = joint_goal[2]
    joint_angles['right_j3'] = joint_goal[3]
    joint_angles['right_j4'] = joint_goal[4]
    joint_angles['right_j5'] = joint_goal[5]
    joint_angles['right_j6'] = joint_goal[6]
    current_j6 = joint_angles['right_j6']

    if abs(current_j6) == current_j6:
        sign = -1
    else:
        sign = +1

    # 31 x 0.1 = 3.1; rouhgly rotating 180 degrees
    for i in range(0, 31):
        # joint_angles['right_j6']=joint_angles['right_j6']+sign*0.1
        # pnp._guarded_move_to_joint_position(joint_angles, timeout=MOTION_SAMPLE_TIME)
        joint_goal[6] = joint_goal[6]+sign*0.1
        pnp.go_to_joint_goal(
            joint_goal, False, MOTION_SAMPLE_TIME/2, goal_tol=0.02, orientation_tol=0.02)
        sleep(MOTION_SAMPLE_TIME)

    # intera is giving shaky and oscillatory movement
    # joint_angles['right_j6']=current_j6
    # pnp._guarded_move_to_joint_position(joint_angles, timeout=2)

    # joint_angles = {'right_j0': -0.041662954890248294,
    # 'right_j1': -1.0258291091425074,
    # 'right_j2': 0.0293680414401436,
    # 'right_j3': 2.17518162913313,
    # 'right_j4': -0.06703022873354225,
    # 'right_j5': 0.3968371433926965,
    # 'right_j6': 1.7659649178699421}
    # pnp._guarded_move_to_joint_position(joint_angles, timeout=2)

    joint_goal[6] = current_j6
    pnp.go_to_joint_goal(joint_goal, True, 2.0)
    sleep(4.0)

    # home_joint_angles = [-0.041662954890248294,-1.0258291091425074, 0.0293680414401436,
    # 2.17518162913313,-0.06703022873354225,0.3968371433926965,1.7659649178699421]
    # pnp.go_to_joint_goal(home_joint_angles,True,5.0)
    # sleep(4.0)
    goto_home(0.3)

    return True


def goto_bin():
    global overhead_orientation, pnp
    allow_replanning = True
    planning_time = MOTION_SAMPLE_TIME
    reached = pnp.go_to_pose_goal(q[0], q[1], q[2], q[3], 0.1-0.02, 0.6, 0.03,
                                  allow_replanning, planning_time)
    return reached
    # pose=Pose(
    #     position=Point(x=0.1, y=0.6, z=-0.15),
    #     orientation=overhead_orientation)
    # return pnp._approach(pose)


def roll():
    # Height of rolling
    # sawyer base is 0.11 away from approx centers of onions at 0.82
    # read from poses topic. max height of onion from its center is 0.076/2=0.038.
    # then tops of onions are approx at 0.11-0.038 = 0.072 m from sawyer base.
    # distance between edge and center of gripper is approx 0.075/2 = 0.037 m.
    # so we lower gripper to z = -0.072+0.037 = -0.035 for rolling.
    # what about x and y locations of trajectory? length of gripper = 0.163 m,
    # width of belt = 0.398 m. not enough space to make a circle.
    # may be a straight line from starting object location to a fixed center
    # on right side boundary x=0.75,y=-0.31 of active region.

    global pnp, target_location_x, target_location_y
    rollx = 3.30
    pitchy = -1.57
    yawz = 0.0
    q_local = quaternion_from_euler(rollx, pitchy, yawz)
    new_orientation = Quaternion(
        x=q_local[0],
        y=q_local[1],
        z=q_local[2],
        w=q_local[3])
    # target_location_z = -0.035

    # give enough space to make gripper change orientation without hitting
    # pose=Pose(
    # position=Point(x=target_location_x, y=target_location_y, z=target_location_z-0.2),
    # orientation=new_orientation)
    # result=pnp._approach(pose)

    allow_replanning = True
    planning_time = 5.0
    print "reaching hover position"
    reached = pnp.go_to_pose_goal(q[0], q[1], q[2], q[3], target_location_x-0.02, target_location_y,
                                  0.14, allow_replanning, planning_time)
    sleep(2.0)

    # now go to desired pose
    # pose=Pose(
    #     position=Point(x=target_location_x, y=target_location_y, z=target_location_z),
    #     orientation=new_orientation)
    # result=pnp._approach(pose)
    allow_replanning = True
    planning_time = 5.0
    height_object_tip = 0.1
    print "reaching position to start rolling"
    reached = pnp.go_to_pose_goal(q_local[0], q_local[1], q_local[2], q_local[3],
                                  target_location_x-0.02, target_location_y,
                                  height_object_tip, allow_replanning, planning_time)
    sleep(2.0)

    for x in range(1, 10):
        reached = pnp.go_to_pose_goal(q_local[0], q_local[1], q_local[2], q_local[3],
                                      target_location_x-0.02, target_location_y+x*0.05,
                                      height_object_tip, allow_replanning, planning_time)
        sleep(2.0)

        pass

    if result:
        pose = Pose(
            position=Point(x=0.75, y=-0.31, z=target_location_z),
            orientation=new_orientation)
        pnp._approach(pose)

        # give enough space to make gripper change orientation without hitting
        pose = Pose(
            position=Point(x=0.75, y=-0.31, z=target_location_z+0.2),
            orientation=new_orientation)
        pnp._approach(pose)
        pose = Pose(
            position=Point(x=0.75, y=-0.31, z=target_location_z+0.2),
            orientation=overhead_orientation)
        res = pnp._approach(pose)
        return res

    return False


def callback_poses(onions_poses_msg):
    current_onions_x = onions_poses_msg.x
    current_onions_y = onions_poses_msg.y
    current_onions_z = onions_poses_msg.z
    global target_location_x, target_location_y
    target_location_x = current_onions_x[onion_index]
    target_location_y = current_onions_y[onion_index]
    # print "target_location_x,target_location_y"+str((target_location_x,target_location_y))
    return


target_location_x = -100
target_location_y = -100
onion_index = 0
req = AttachRequest()


def callback_modelname(color_indices_msg):
    global req, onion_index
    if (color_indices_msg.data[onion_index] == 0):
        req.model_name_1 = "good_onion_" + str(onion_index)
    else:
        req.model_name_1 = "bad_onion_" + str(onion_index)
    return


def main():
    try:

        # spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)    # SPAWNING CUBE OBJECT IN GAZEBO
        # rospy.loginfo("Waiting for /gazebo/spawn_sdf_model service...")
        # spawn_srv.wait_for_service()
        # rospy.loginfo("Connected to service!")

        global target_location_x, target_location_y, pnp
        hovertime = 1.0
        rospy.Subscriber("onions_blocks_poses",
                         onions_blocks_poses, callback_poses)
        rospy.Subscriber("current_onions_blocks",
                         Int8MultiArray, callback_modelname)
        # attach and detach service
        attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        attach_srv.wait_for_service()
        detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        detach_srv.wait_for_service()

        req.link_name_1 = "base_link"
        req.model_name_2 = "sawyer"
        req.link_name_2 = "right_l6"

        detach_srv.call(req)
        # os.system('rosrun gazebo_ros_link_attacher detach.py')
        # print "liftgripper()"
        # liftgripper()

        print "goto_home()"
        goto_home(0.4, goal_tol=0.1,
                  orientation_tol=0.1)
        # sleep(10.0)
        # print "current_pose "
        # print pnp.group.get_current_pose().pose
        # sleep(2.0)

        ## PICKING ##
        print "target_location_x,target_location_y"+str((target_location_x, target_location_y))
        print "hover()"
        success = hover(hovertime)
        # sleep(60.0)
        # exit(0)
        print "lowergripper()"
        reset_gripper()
        activate_gripper()
        gripper_to_pos(0, 60, 200, False)    # OPEN GRIPPER
        lowergripper()
        # sleep(5.0)
        attach_srv.call(req)
        # os.system('rosrun gazebo_ros_link_attacher attach.py')
        sleep(2.0)

        print "liftgripper()"
        liftgripper()
        sleep(2.0)
        exit(0)
        ## INPECTION ##
        # print "lookNrotate()"
        # lookNrotate()
        # sleep(15.0)

        ## PLACING ##
        # print "goto_bin()"
        # goto_bin()
        # sleep(2.0)

        detach_srv.call(req)

        ## ROLLING ##

        # print "hover() on leftmost onion"
        # hover()
        # sleep(2.0)
        print "roll()"
        roll()
        sleep(50.0)
        exit(0)

        # rospy.spin()

        s = rospy.Service('move_sawyer', move_robot, handle_move_sawyer)
        print "Ready to move sawyer."

        # Starting Joint angles for right arm

        # spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)    # SPAWNING CUBE OBJECT IN GAZEBO
        # rospy.loginfo("Waiting for /gazebo/spawn_sdf_model service...")
        # spawn_srv.wait_for_service()
        # rospy.loginfo("Connected to service!")
        rollx = 3.14
        pitchy = 0.0
        yawz = -1.57
        q = quaternion_from_euler(rollx, pitchy, yawz)
        overhead_orientation_moveit = Quaternion(
            x=q[0],
            y=q[1],
            z=q[2],
            w=q[3])
        #q=[ 0.707388, 0.706825, -0.0005629, 0.0005633 ]
        # # Spawn object 1
        # rospy.loginfo("Spawning cube1")
        # req1 = create_cube_request("cube1",
        #                           0.705, 0.0, 0.80,  # position
        #                           0.0, 0.0, 0.0,  # rotation
        #                           0.0762, 0.0762, 0.0762)  # size

        # # Move to the desired starting angles
        # print("Running. Ctrl-c to quit")
        # pnp.move_to_start(starting_joint_angles)

        # rospy.sleep(1)

        # spawn_srv.call(req1)  #Spawn cube now

        reset_gripper()

        activate_gripper()

        # 255 = closed, 0 = open
        gripper_to_pos(0, 60, 200, False)    # OPEN GRIPPER

        pnp.go_to_pose_goal(q[0], q[1],
                            q[2], q[3],
                            0.75, 0.0, 0.0)
        sleep(10.0)
        pnp.go_to_pose_goal(q[0], q[1],
                            q[2], q[3],
                            0.0, 0.5, 0.0)
        sleep(10.0)
        pnp.go_to_pose_goal(q[0], q[1],
                            q[2], q[3],
                            0.0, 0.0, 0.3)
        sleep(10.0)
        pnp.go_to_pose_goal(q[0], q[1],
                            q[2], q[3],
                            0.75, 0.0, 0.0)
        sleep(10.0)

        print("WAYPOINT 1 ")
        pnp.go_to_pose_goal(q[0], q[1],
                            q[2], q[3],
                            0.75, 0.0, 0.15)
        # pnp.go_to_pose_goal(0.7071029, 0.7071057, 0.0012299, 0.0023561,    # GO TO WAYPOINT 1 (HOVER POS)
        #                          0.665, 0.0, 0.15)

        sleep(5.0)
        print("WAYPOINT 2 ")
        pnp.go_to_pose_goal(0.7071029, 0.7071057, 0.0012299, 0.0023561,    # GO TO WAYPOINT 2 (HOVER POS)
                            0.75, 0.0, 0.002)

        sleep(5.0)
        print("WAYPOINT 3 ")
        pnp.go_to_pose_goal(0.7071029, 0.7071057, 0.0012299, 0.0023561,    # GO TO WAYPOINT 3 (PLUNGE AND PICK)
                            0.75, 0.0, -0.001)
        sleep(5.0)
        gripper_to_pos(50, 60, 200, False)    # GRIPPER TO POSITION 50

        # ATTACH CUBE AND SAWYER EEF
        os.system('rosrun gazebo_ros_link_attacher attach.py')

        sleep(1.0)
        print("WAYPOINT 4 ")
        pnp.go_to_pose_goal(0.7071029, 0.7071057, 0.0012299, 0.0023561,    # GO TO WAYPOINT 4 (TRAVEL TO PLACE DESTINATION)
                            0.75, 0.04, 0.15)

        sleep(5.0)
        print("WAYPOINT 5 ")
        pnp.go_to_pose_goal(0.7071029, 0.7071057, 0.0012299, 0.0023561,    # GO TO WAYPOINT 5 (TRAVEL TO PLACE DESTINATION)
                            0.75, 0.5, 0.02)

        sleep(5.0)
        print("WAYPOINT 6")
        pnp.go_to_pose_goal(0.7071029, 0.7071057, 0.0012299, 0.0023561,    # GO TO WAYPOINT 6 (PLACE)
                            0.75, 0.5, -0.055)

        # DETACH CUBE AND SAWYER EEF
        os.system('rosrun gazebo_ros_link_attacher detach.py')

        gripper_to_pos(0, 60, 200, False)    # OPEN GRIPPER

        sleep(1.0)
        print("WAYPOINT 6")
        pnp.go_to_pose_goal(0.7071029, 0.7071057, 0.0012299, 0.0023561,    # GO TO WAYPOINT 7 (RETURN TO HOVER POS)
                            0.75, 0.5, 0.12)
        sleep(5.0)
        rospy.sleep(1.0)
        delete_gazebo_models()

    except rospy.ROSInterruptException:
        delete_gazebo_models()
        return
    except KeyboardInterrupt:
        delete_gazebo_models()
        return


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
