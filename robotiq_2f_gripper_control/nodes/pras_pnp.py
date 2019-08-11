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
from gripper_to_position import gripper_to_pos
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

import intera_interface

from copy import deepcopy
from tf.transformations import quaternion_from_euler

## END_SUB_TUTORIAL

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal: A list of floats, a Pose or a PoseStamped
  @param: actual: list of floats, a Pose or a PoseStamped
  @param: tolerance: A float
  """
  
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class PickAndPlace(object):
  def __init__(self, limb="right", hover_distance = 0.15, tip_name="right_gripper_tip"):
    super(PickAndPlace, self).__init__()

    joint_state_topic = ['joint_states:=/robot/joint_states']

    moveit_commander.roscpp_initialize(joint_state_topic)

    rospy.init_node('simple_pnp_gazebo',
                    anonymous=True, disable_signals=False)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    move_group = moveit_commander.MoveGroupCommander("right_arm")

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = move_group.get_planning_frame()

    eef_link = move_group.get_end_effector_link()

    group_names = robot.get_group_names()
    
    move_group.set_goal_orientation_tolerance(0.5)

    move_group.set_goal_position_tolerance(0.05)

    # Allow 5 seconds per planning attempt
    move_group.set_planning_time(3)
        
    # Set a limit on the number of pick attempts before bailing
    max_pick_attempts = 5
       
    # Set a limit on the number of place attempts
    max_place_attempts = 5
                
    # Give the scene a chance to catch up
    rospy.sleep(2)

    #print robot.get_current_state()
    
    
    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    self._hover_distance = hover_distance # in meters
    self._limb_name = limb # string
    self._limb = intera_interface.Limb(limb)
    self._tip_name = tip_name # string

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
    
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = ox
    pose_goal.orientation.y = oy
    pose_goal.orientation.z = oz
    pose_goal.orientation.w = ow
    pose_goal.position.x = px
    pose_goal.position.y = py
    pose_goal.position.z = pz
    self.move_group.set_pose_target(pose_goal)
    plan = self.move_group.go(wait=True)
    self.move_group.allow_replanning(True)
    self.move_group.stop()
    self.move_group.clear_pose_targets()
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def display_trajectory(self, plan):
    """
    Display a movement plan / trajectory
    @param: plan: Plan to be displayed
    """
    display_trajectory_publisher = self.display_trajectory_publisher
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = self.robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory);
   


############################## YET TO CLEANUP ###########################################

def create_cube_request(modelname, px, py, pz, rr, rp, ry, sx, sy, sz):
    """
    Create a cube to be spawned in gazebo
    @param: px: Position coordinate on x-axis
    @param: py: Position coordinate on y-axis
    @param: pz: Position coordinate on z-axis
    @param: rr: Roll rotation
    @param: rp: Pitch rotation
    @param: ry: Yaw rotation
    @param: sx: Cube size on x-axis
    @param: sy: Cube size on y-axis
    @param: sz: Cube size on z-axis
    """

    # Get Models' Path
    model_path = rospkg.RosPack().get_path('robotiq_2f_gripper_control')+"/models/"

    with open(model_path + 'cube.sdf', 'r') as file:
      sdf_cube = file.read().replace('\n', '')

    cube = deepcopy(sdf_cube)
    # Replace size of model
    size_str = str(round(sx, 3)) + " " + \
        str(round(sy, 3)) + " " + str(round(sz, 3))
    cube = cube.replace('SIZEXYZ', size_str)
    # Replace modelname
    cube = cube.replace('MODELNAME', str(modelname))

    req = SpawnModelRequest()
    req.model_name = modelname
    req.model_xml = cube
    req.initial_pose.position.x = px
    req.initial_pose.position.y = py
    req.initial_pose.position.z = pz

    q = quaternion_from_euler(rr, rp, ry)
    req.initial_pose.orientation.x = q[0]
    req.initial_pose.orientation.y = q[1]
    req.initial_pose.orientation.z = q[2]
    req.initial_pose.orientation.w = q[3]

    return req

##################################################################################################


def main():
  try:
    
    spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)    # SPAWNING CUBE OBJECT IN GAZEBO
    rospy.loginfo("Waiting for /gazebo/spawn_sdf_model service...")
    spawn_srv.wait_for_service()
    rospy.loginfo("Connected to service!")

    # Spawn object 1
    rospy.loginfo("Spawning cube1")
    req1 = create_cube_request("cube1",
                              0.7, 0.0, 0.80,  # position
                              0.0, 0.0, 0.0,  # rotation
                              0.0762, 0.0762, 0.0762)  # size
    spawn_srv.call(req1)

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
    pnp = PickAndPlace(limb, hover_distance)
	# An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(
                             x=-0.00142460053167,
                             y=0.999994209902,
                             z=-0.00177030764765,
                             w=0.00253311793936)
    pose_list = []
    block_poses = list()
    # The Pose of the block in its initial location.
    # You may wish to replace these poses with estimates
    # from a perception node.
    block_poses.append(Pose(
        position=Point(x=0.7, y=0.02, z=-0.12),
        orientation=overhead_orientation))
    # Feel free to add additional desired poses for the object.
    # Each additional pose will get its own pick and place.
    block_poses.append(Pose(
        position=Point(x=0.7, y=-0.3, z=-0.10),
        orientation=overhead_orientation))
    # Move to the desired starting angles
    print("Running. Ctrl-c to quit")
    pnp.move_to_start(starting_joint_angles)

    gripper_to_pos(0, 60, 200, False)    # ACTIVATION STEP
    gripper_to_pos(0, 60, 200, False)    # OPEN GRIPPER

    sleep(1.0)


    pnp.go_to_pose_goal(-0.00142460053167, 0.999994209902, -0.00177030764765, 0.00253311793936,    # GO TO WAYPOINT 1 (HOVER POS)
                             0.64, 0.02, 0.15)

    sleep(1.0)

    pnp.go_to_pose_goal(-0.00142460053167, 0.999994209902, -0.00177030764765, 0.00253311793936,    # GO TO WAYPOINT 2 (PLUNGE AND PICK)
                             0.64, 0.02, -0.1)
    sleep(1.0)

    gripper_to_pos(50, 60, 200, False)    # GRIPPER TO POSITION 50
    
    os.system('rosrun gazebo_ros_link_attacher attach.py')    # ATTACH CUBE AND SAWYER EEF

    sleep(1.0)

    pnp.go_to_pose_goal(-0.00142460053167, 0.999994209902, -0.00177030764765, 0.00253311793936,    # GO TO WAYPOINT 3 (TRAVEL TO PLACE DESTINATION)
                             0.7, 0.04, 0.15)

    sleep(1.0)

    pnp.go_to_pose_goal(-0.00142460053167, 0.999994209902, -0.00177030764765, 0.00253311793936,    # GO TO WAYPOINT 4 (PLACE)
                             0.7, 0.3, -0.10)

    os.system('rosrun gazebo_ros_link_attacher detach.py')    # DETACH CUBE AND SAWYER EEF

    gripper_to_pos(0, 60, 200, False)    # OPEN GRIPPER

    sleep(1.0)

    pnp.go_to_pose_goal(-0.00142460053167, 0.999994209902, -0.00177030764765, 0.00253311793936,    # GO TO WAYPOINT 5 (RETURN TO HOVER POS)
                             0.7, 0.04, 0.15)
    

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
