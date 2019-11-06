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
from std_msgs.msg import String
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

## END_SUB_TUTORIAL

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal: A list of floats, a Pose or a PoseStamped
  @param: actual: list of floats, a Pose or a PoseStamped
  @param: tolerance: A float
  """
  
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

joint_state_topic = ['joint_states:=/robot/joint_states']

class PickAndPlace(object):
  def __init__(self,limb="right"):
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

    # Misc variables
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    self._limb_name = limb # string
    self._limb = intera_interface.Limb(limb)

  def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)

  def _guarded_move_to_joint_position(self, joint_angles, timeout=5.0):
        if rospy.is_shutdown():
            return
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles,timeout=timeout)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")



  def go_to_pose_goal(self, ox, oy, oz, ow, px, py, pz):
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
    group.set_goal_orientation_tolerance(0.1) 

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = ox
    pose_goal.orientation.y = oy
    pose_goal.orientation.z = oz
    pose_goal.orientation.w = ow
    pose_goal.position.x = px
    pose_goal.position.y = py
    pose_goal.position.z = pz
    group.set_pose_target(pose_goal)
    group.allow_replanning(True)
    group.set_planning_time(5) 
    plan = group.go(wait=True)
    rospy.sleep(15)
    group.stop()
   
    group.clear_pose_targets()

    
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

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
  
    display_trajectory_publisher.publish(display_trajectory);


def main():
  try:
    
    limb = 'right'
    hover_distance = 0.15 # meters
    # Starting Joint angles for right arm
    starting_joint_angles = {'right_j0': -0.041662954890248294,
                             'right_j1': -1.0258291091425074,
                             'right_j2': 0.0293680414401436,
                             'right_j3': 2.17518162913313,
                             'right_j4':  -0.06703022873354225,
                             'right_j5': 0.3968371433926965,
                             'right_j6': 1.7659649178699421}
    pnp = PickAndPlace(limb)
	# An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(
                             x=0.7071057,
                             y=-0.7071029,
                             z=0.0023561,
                             w=-0.0012299)

    # Move to the desired starting angles
    print("Running. Ctrl-c to quit")
    pnp.move_to_start(starting_joint_angles)
    
    rospy.sleep(1)

    reset_gripper()

    activate_gripper()

    # 255 = closed, 0 = open
    gripper_to_pos(0, 60, 200, False)    # OPEN GRIPPER

    sleep(1.0)


    pnp.go_to_pose_goal(0.7071029, 0.7071057, 0.0012299, 0.0023561,    # GO TO WAYPOINT 1 (HOVER POS)
                             0.665, 0.0, 0.15)

    sleep(1.0)

    pnp.go_to_pose_goal(0.7071029, 0.7071057, 0.0012299, 0.0023561,    # GO TO WAYPOINT 2 (HOVER POS)
                             0.665, 0.0, 0.002)

    sleep(1.0)

    pnp.go_to_pose_goal(0.7071029, 0.7071057, 0.0012299, 0.0023561,    # GO TO WAYPOINT 3 (PLUNGE AND PICK)
                             0.665, 0.0, -0.001)
    sleep(1.0)

    gripper_to_pos(50, 60, 200, False)    # GRIPPER TO POSITION 50
    
    os.system('rosrun gazebo_ros_link_attacher attach.py')    # ATTACH CUBE AND SAWYER EEF

    sleep(1.0)

    pnp.go_to_pose_goal(0.7071029, 0.7071057, 0.0012299, 0.0023561,    # GO TO WAYPOINT 4 (TRAVEL TO PLACE DESTINATION)
                             0.665, 0.04, 0.15)

    sleep(1.0)

    pnp.go_to_pose_goal(0.7071029, 0.7071057, 0.0012299, 0.0023561,    # GO TO WAYPOINT 5 (TRAVEL TO PLACE DESTINATION)
                             0.665, 0.5, 0.02)

    sleep(1.0)

    pnp.go_to_pose_goal(0.7071029, 0.7071057, 0.0012299, 0.0023561,    # GO TO WAYPOINT 6 (PLACE)
                             0.665, 0.5, -0.055)

    os.system('rosrun gazebo_ros_link_attacher detach.py')    # DETACH CUBE AND SAWYER EEF

    gripper_to_pos(0, 60, 200, False)    # OPEN GRIPPER

    sleep(1.0)

    pnp.go_to_pose_goal(0.7071029, 0.7071057, 0.0012299, 0.0023561,    # GO TO WAYPOINT 7 (RETURN TO HOVER POS)
                             0.665, 0.5, 0.12)
    rospy.sleep(1.0)

    delete_gazebo_models()
    

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
	try:
  		main()
	except rospy.ROSInterruptException:
	    pass
