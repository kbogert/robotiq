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


joint_state_topic = ['joint_states:=/robot/joint_states']

# a global class object
limb = 'right'
# intermediate variable needed for avoiding collisions
step_distance = 0.05  # meters
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
        # See ompl_planning.yaml for a complete list
        group.set_planner_id("RRTConnectkConfigDefault")

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        planning_frame = group.get_planning_frame()

        eef_link = group.get_end_effector_link()

        group_names = robot.get_group_names()

        print robot.get_current_state()

        # now for intera
        #print("Getting robot state... ")
        #self._rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        #self._init_state = self._rs.state().enabled
        #print("Enabling robot... ")
        #self._rs.enable()

        # Misc variables
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

    #This method uses intera to move because we assume there are no
    #obstacles on the way to the assigned pose.
    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to pose...".format(self._limb_name))
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

    # def _approach_with_leeway(self, pose):
    #     approach = copy.deepcopy(pose)
    #     # approach with a pose the hover-distance above the requested pose
    #     approach.position.z = approach.position.z + self._step_distance
    #     print "finding ik for "+str((approach.position.x, approach.position.y, approach.position.z))
    #     joint_angles = self._limb.ik_request(approach, self._tip_name)
    #     # print str(joint_angles)+" is output from limb.ik_request "
    #     self._limb.set_joint_position_speed(0.001)
    #     self._guarded_move_to_joint_position(joint_angles, timeout=1.0)
    #     self._limb.set_joint_position_speed(0.1)
    #     if joint_angles:
    #         return True
    #     else:
    #         return False

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

    #This is an intera based IK method - Doesn't know about collision objects
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
        group.set_goal_position_tolerance(0.001)
        group.set_goal_orientation_tolerance(0.03)

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
    
    def wait_for_state_update(box_name, scene, box_is_known = False, box_is_attached = False, timeout = 4):
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False


    def dip(self):

        global q, target_location_x, target_location_y
        while target_location_x == -100:
            sleep(0.05)
        current_pose = self.group.get_current_pose().pose
        allow_replanning = True
        planning_time = MOTION_SAMPLE_TIME*2
        if current_pose.position.y >= target_location_y - 0.028 and current_pose.position.y <= target_location_y + 0.028:
            dip = self.go_to_pose_goal(q[0], q[1], q[2], q[3], target_location_x+0.01,
                                    target_location_y + 0.02, #accounting for tolerance error
                                    current_pose.position.z - 0.15, #This is where we dip
                                    allow_replanning, planning_time)
            rospy.sleep(0.1)
            
            print "Successfully dipped! z pos: ", current_pose.position.z
            
            box_pose = geometry_msgs.msg.PoseStamped()
            box_pose.header.frame_id = "right_l6"
            box_pose.pose.orientation.w = 1.0
            # box_pose.pose.position.x = target_location_x
            # box_pose.pose.position.y = target_location_y
            box_pose.pose.position.z = current_pose.position.z + 0.005
            box_name = "good_onion_0"
            self.scene.add_box(box_name, box_pose, size=(0.065, 0.065, 0.065))
            
            grasping_group = 'right_arm'
            group = self.group
            robot = self.robot
            touch_links = robot.get_link_names(group=grasping_group)
            self.scene.attach_box(self.eef_link, box_name, touch_links = touch_links)
            # rospy.loginfo(self.wait_for_state_update(box_name, self.scene, box_is_attached=True, box_is_known=False))
            return True
        else:
            rospy.sleep(0.05)
            print "Current position of gripper (y,z): ", current_pose.position.y, current_pose.position.z
            print "Current position of onion in (y): ", target_location_y
            dip = self.dip()
        return dip
        

    def waitToPick(self):

        global q, target_location_x, target_location_y
        while target_location_x == -100:
            sleep(0.05)
        current_pose = self.group.get_current_pose().pose
        allow_replanning = True
        planning_time = 5
        waiting = self.go_to_pose_goal(q[0], q[1], q[2], q[3], target_location_x+0.01,
                                    current_pose.position.y, current_pose.position.z,
                                    allow_replanning, planning_time)
        rospy.sleep(0.05)
        dip = self.dip()
        print "dip status: ", dip 
        # dip = True
        return dip

    def liftgripper(self):
        # approx centers of onions at 0.82, width of onion is 0.038 m. table is at 0.78
        # length of gripper is 0.163 m The gripper should not go lower than
        # (height_z of table w.r.t base+gripper-height/2+tolerance) = 0.78-0.93+0.08+0.01=-0.24
        # pnp._limb.endpoint_pose returns {'position': (x, y, z), 'orientation': (x, y, z, w)}
        # moving from z=-.02 to z=-0.1
        print "Attempting to lift gripper"
        global q, target_location_x, target_location_y
        while target_location_x == -100:
            sleep(0.05)
        current_pose = self.group.get_current_pose().pose
        allow_replanning = True
        planning_time = 5
        i = 0
        while i < 0.6:
            print "Current z pose: ",current_pose.position.z
            waiting = self.go_to_pose_goal(q[0], q[1], q[2], q[3], target_location_x,
                                    current_pose.position.y, current_pose.position.z + i,
                                    allow_replanning, planning_time)
            i = i + 0.2
            rospy.sleep(0.1)
            print "Updated gripper z value is: ",current_pose.position.z

        return True

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
        
############################################## END OF CLASS ##################################################

#Creating a global class object pnp to access all the methods and attributes
pnp = PickAndPlace(limb, step_distance, tip_name)


# def handle_move_sawyer(req_move_robot):
#     # "home","bin","hover","lowergripper","lookNrotate","roll"
#     # location - (x,y)
#     target_location_x = req_move_robot.location.x
#     target_location_y = req_move_robot.location.y
#     argument = req_move_robot.primitive
#     switcher = {
#         "home": goto_home,
#         "bin": goto_bin,
#         "hover": hover,
#         "lowergripper": lowergripper,
#         "lookNrotate": lookNrotate,
#         "roll": roll
#     }
#     # Get the function from switcher dictionary
#     func = switcher.get(
#         argument, lambda: "Invalid input to move_sawyer service")
#     if argument not in switcher.keys():
#         return move_robotResponse(False)
#     # Execute the function
#     result = func()
#     if result == True:
#         return move_robotResponse(True)
#     return move_robotResponse(False)


#This method uses intera to move because we assume there are no
#obstacles on the way to the start pose.
def goto_home(tolerance=0.01, goal_tol=0.02, orientation_tol=0.02):
    
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
    # 0.7716502133436203, -0.25253308083711357, -0.9156571119870254, 1.6775039734444164, 2.969104448028304, -2.2600790124759307, -2.608939978894689
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
        sleep(0.1)
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
            sleep(0.1)

        print "diff:"+str(diff)

    print("reached home")
    return True
    # if diff:
    #   return pnp.move_to_start(joint_angles)
    # else:
    #   return True



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

def view(tolerance=0.01, goal_tol=0.02, orientation_tol=0.02):
    
    global pnp

    home_joint_angles = [-0.041662954890248294, -1.0258291091425074, 0.0293680414401436,
                         2.17518162913313, -0.06703022873354225, 0.3968371433926965, 1.7659649178699421]
    # pnp.go_to_joint_goal(home_joint_angles,True,5.0)
    # sleep(10.0)

    joint_angles = {'right_j0': 0.7716502133436203,
                    'right_j1': -0.25253308083711357,
                    'right_j2': -0.9156571119870254,
                    'right_j3': 1.6775039734444164,
                    'right_j4': 2.969104448028304,
                    'right_j5': -2.2600790124759307,
                    'right_j6': -2.608939978894689}
    # 0.7716502133436203, -0.25253308083711357, -0.9156571119870254, 1.6775039734444164, 2.969104448028304, -2.2600790124759307, -2.608939978894689
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
            sleep(0.1)

        print "diff:"+str(diff)

    print("reached viewpoint")
    return True



def goto_bin(tolerance=0.1):
#0.0007738188961337045, 0.9942022319650565, -0.6642366352730953, 0.46938807849915687, 1.5498016537213086, -0.8777244285593966, 0.8579252090846943, 2.18012354574336

    global overhead_orientation_moveit, pnp, target_location_x, target_location_y
    allow_replanning = True
    planning_time = 10
    reached = False
    reached_waypoint = False
    current_pose = pnp.group.get_current_pose().pose
    height = current_pose.position.z
    print "Attempting to reach the bin"
    #while 0.45 < current_pose.position.x < 0.5 or 0.48 < current_pose.position.y < 0.52:
    reached_waypoint = pnp.go_to_pose_goal(q[0], q[1], q[2], q[3], 0.482810140925, 0.509723584575, 0,
                                    allow_replanning, planning_time)
    rospy.sleep(0.1)
    # while current_pose.position.x - 0.1 >= tolerance:
    reached = pnp.go_to_pose_goal(q[0], q[1], q[2], q[3], 0.1, 0.6, 0,
                                allow_replanning, planning_time)
    rospy.sleep(0.1)
    
    current_pose = pnp.group.get_current_pose().pose
    print "Current gripper pose: ", current_pose
    print "Reached bin: ", reached
    
    return reached


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

    # if result:
    #     pose = Pose(
    #         position=Point(x=0.75, y=-0.31, z=target_location_z),
    #         orientation=new_orientation)
    #     pnp._approach(pose)

    #     # give enough space to make gripper change orientation without hitting
    #     pose = Pose(
    #         position=Point(x=0.75, y=-0.31, z=target_location_z+0.2),
    #         orientation=new_orientation)
    #     pnp._approach(pose)
    #     pose = Pose(
    #         position=Point(x=0.75, y=-0.31, z=target_location_z+0.2),
    #         orientation=overhead_orientation_moveit)
    #     res = pnp._approach(pose)
    #     return res

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
        print "Onion name set in IF as: ", req.model_name_1 
    else:
        req.model_name_1 = "bad_onion_" + str(onion_index)
        print "Onion name set in ELSE as: ", req.model_name_1 
    return



def main():
    try:

        global target_location_x, target_location_y, pnp, req
        rospy.Subscriber("onions_blocks_poses",
                         onions_blocks_poses, callback_poses)
        callback_modelname(rospy.wait_for_message("current_onions_blocks", 
                            Int8MultiArray)) #This will only listen once until it hears something,
                                             # this may cause trouble later, watch out!
        # attach and detach service
        attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        attach_srv.wait_for_service()
        detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        detach_srv.wait_for_service()
        req.link_name_1 = "base_link"
        req.model_name_2 = "sawyer"
        req.link_name_2 = "right_l6"
        print "(model_1,link_1,model_2,link_2)", req.model_name_1,req.link_name_1,req.model_name_2,req.link_name_2

        reset_gripper()
        activate_gripper()
        gripper_to_pos(0, 60, 200, False)    # OPEN GRIPPER
        print "goto_home()"
        goto_home(0.3, goal_tol=0.01, orientation_tol=0.1)

        ##############################################
        status = pnp.waitToPick()
        print "status: ", status
        #status = True
        if(status):
            attach_srv.call(req)
            sleep(0.1)
            pnp.liftgripper()
            sleep(0.1)
            view(0.3)
            sleep(0.1)
            # goto_home(0.3)
            # sleep(0.1)
            goto_bin()
            sleep(0.1)
            # detach_srv.call(req)
            print "Worked like a charm!"
        ##############################################

        # print "lowergripper()"
        # lowergripper()
        # attach_srv.call(req)
        # sleep(2.0)
        # print "liftgripper()"
        # liftgripper()
        # sleep(2.0)
        # exit(0)

        ## INPECTION ##
        # print "lookNrotate()"
        # lookNrotate()
        # sleep(15.0)

        ## PLACING ##
        # print "goto_bin()"
        # goto_bin()
        # sleep(2.0)

        #detach_srv.call(req)

        ## ROLLING ##

        # print "hover() on leftmost onion"
        # hover()
        # sleep(2.0)
        # print "roll()"
        # roll()
        # sleep(50.0)
        # exit(0)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print "Main function not found! WTH dude!"
