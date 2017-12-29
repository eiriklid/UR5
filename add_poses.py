#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley

## BEGIN_SUB_TUTORIAL imports
##
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
import trajectory_msgs.msg
## END_SUB_TUTORIAL
from pprint import pprint

from std_msgs.msg import String

import quaternion

def move_group_python_add_world():
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in the left
  ## arm.  This interface can be used to plan and execute motions on the left
  ## arm.
  #group = moveit_commander.MoveGroupCommander("right_arm")
  #group = moveit_commander.MoveGroupCommander("manipulator")
  group = moveit_commander.MoveGroupCommander("endeffector")
  
  ## We create this DisplayTrajectory publisher which is used below to publish
  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory, queue_size=10)

    ## We can get the name of the reference frame for this robot
  print "============ Robot Reference frame: %s" % group.get_planning_frame()

  ## We can also print the name of the end-effector link for this group
  print "============ End_effector Reference frame: %s" % group.get_end_effector_link()

  ## We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()

  
  planning_scene_diff_publisher = rospy.Publisher("planning_scene",moveit_msgs.msg.PlanningScene, queue_size=1)
  
  while planning_scene_diff_publisher.get_num_connections() < 1:
    rospy.sleep(0.5)
    print "\n\n======Pub sleep==========\n"
  

  attached_object = moveit_msgs.msg.AttachedCollisionObject()
  attached_object.link_name = "world"
  attached_object.object.header.frame_id = "world"
  # The id of the object */
  attached_object.object.id = "table"

  pose = geometry_msgs.msg.Pose()
  pose.position.x =0
  pose.position.z =-0.32
  pose.position.y =0

  pose.orientation.w = 1 
  

  primitive = shape_msgs.msg.SolidPrimitive()
  primitive.type = primitive.CYLINDER
  #primitive.dimensions.resize(3)
  primitive.dimensions.append(0.01)
  primitive.dimensions.append(0.7)

  attached_object.object.primitives.append(primitive)
  attached_object.object.primitive_poses.append(pose)

  attached_object.object.operation = attached_object.object.ADD;

  
  rospy.loginfo("Adding table into the world.")
  planning_scene = moveit_msgs.msg.PlanningScene()
  planning_scene.world.collision_objects.append(attached_object.object)
  planning_scene.is_diff = True
  planning_scene_diff_publisher.publish(planning_scene)
  rospy.sleep(0.5)


  ###### ADD WALL ###########################################
  attached_object = moveit_msgs.msg.AttachedCollisionObject()
  attached_object.link_name = "world"
  attached_object.object.header.frame_id = "world"
  # The id of the object */
  attached_object.object.id = "wall"

  pose = geometry_msgs.msg.Pose()
  pose.position.x =0
  pose.position.y =-0.3
  pose.position.z =0

  pose.orientation.w = 1  
  pose.orientation.z = 0.71 
  

  primitive = shape_msgs.msg.SolidPrimitive()
  primitive.type = primitive.BOX
  #primitive.dimensions.resize(3)
  primitive.dimensions.append(0.01)
  primitive.dimensions.append(2.5)
  primitive.dimensions.append(1)

  attached_object.object.primitives.append(primitive)
  attached_object.object.primitive_poses.append(pose)

  attached_object.object.operation = attached_object.object.ADD;

  
  rospy.loginfo("Adding wall into the world.")
  planning_scene = moveit_msgs.msg.PlanningScene()
  planning_scene.world.collision_objects.append(attached_object.object)
  planning_scene.is_diff = True
  planning_scene_diff_publisher.publish(planning_scene)
  rospy.sleep(0.5)
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"
  ############# ADD WALL - END #######################################

def move_group_python_attach_phone():
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in the left
  ## arm.  This interface can be used to plan and execute motions on the left
  ## arm.
  #group = moveit_commander.MoveGroupCommander("right_arm")
  #group = moveit_commander.MoveGroupCommander("manipulator")
  group = moveit_commander.MoveGroupCommander("endeffector")
  
  ## We create this DisplayTrajectory publisher which is used below to publish
  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory, queue_size=10)

  ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
  print "============ Waiting for RVIZ..."
  #rospy.sleep(10)
  print "============ Starting tutorial "

  ## Getting Basic Information
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^
  ##
  ## We can get the name of the reference frame for this robot
  print "============ Reference frame: %s" % group.get_planning_frame()

  ## We can also print the name of the end-effector link for this group
  print "============ Reference frame: %s" % group.get_end_effector_link()

  ## We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()

  
  planning_scene_diff_publisher = rospy.Publisher("planning_scene",moveit_msgs.msg.PlanningScene, queue_size=1)
  
  while planning_scene_diff_publisher.get_num_connections() < 1:
    rospy.sleep(0.5)
    print "\n\n======Pub sleep==========\n"
  

  attached_object = moveit_msgs.msg.AttachedCollisionObject()
  attached_object.link_name = "ee_link"
  attached_object.object.header.frame_id = "ee_link"
  # The id of the object */
  attached_object.object.id = "phone"

  pose = geometry_msgs.msg.Pose()
  pose.position.x = 0.015
  pose.orientation.w = 1.0  
  pose.orientation.x = 0.36  


  primitive = shape_msgs.msg.SolidPrimitive()
  primitive.type = primitive.BOX
  #primitive.dimensions.resize(3)
  primitive.dimensions.append(0.03)
  primitive.dimensions.append(0.15)
  primitive.dimensions.append(0.08)

  attached_object.object.primitives.append(primitive)
  attached_object.object.primitive_poses.append(pose)

  attached_object.object.operation = attached_object.object.ADD;

  
  rospy.loginfo("Adding the object into the world at the location of the wrist.")
  planning_scene = moveit_msgs.msg.PlanningScene()
  planning_scene.world.collision_objects.append(attached_object.object)
  planning_scene.is_diff = True
  planning_scene_diff_publisher.publish(planning_scene)
  rospy.sleep(0.5)
  
  remove_object = moveit_msgs.msg.CollisionObject()
  remove_object.id = "phone"
  remove_object.header.frame_id = "ee_link"
  remove_object.operation = remove_object.REMOVE

  rospy.loginfo("Attaching the object to the right wrist and removing it from the world.")
  planning_scene.world.collision_objects = [] #clear list
  planning_scene.world.collision_objects.append(remove_object)
  planning_scene.robot_state.attached_collision_objects.append(attached_object)
  planning_scene_diff_publisher.publish(planning_scene)
  rospy.sleep(0.5)

def move_group_python_interface_tutorial():
  ## BEGIN_TUTORIAL
  ##
  ## Setup
  ## ^^^^^
  ## CALL_SUB_TUTORIAL imports
  ##
  ## First initialize moveit_commander and rospy.
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in the left
  ## arm.  This interface can be used to plan and execute motions on the left
  ## arm.
  #group = moveit_commander.MoveGroupCommander("right_arm")
  group = moveit_commander.MoveGroupCommander("manipulator")


  ## We create this DisplayTrajectory publisher which is used below to publish
  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)

  ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
  print "============ Waiting for RVIZ..."
  #rospy.sleep(10)
  print "============ Starting tutorial "

  ## Getting Basic Information
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^
  ##
  ## We can get the name of the reference frame for this robot
  print "============ Reference frame: %s" % group.get_planning_frame()

  ## We can also print the name of the end-effector link for this group
  print "============ Reference frame: %s" % group.get_end_effector_link()

  ## We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()

  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"

  ## Planning to a Pose goal

  ## Let's set a joint space goal and move towards it. 
  ## First, we will clear the pose target we had just set.

  group.clear_pose_targets()
  ## ^^^^^^^^^^^^^^^^^^^^^^^
  ## We can plan a motion for this group to a desired pose for the 
  ## end-effector
  print "============ Generating init pose plan "
  print group.get_current_pose().pose
  init_pose_target = geometry_msgs.msg.Pose()
  init_pose_target = group.get_current_pose().pose
  init_pose_target.position.x = -0.5
  init_pose_target.position.y = 0.2
  init_pose_target.position.z = 0.1
  init_pose_target.orientation = quaternion.INIT
  group.set_pose_target(init_pose_target)

  ## Now, we call the planner to compute the plan
  ## and visualize it if successful
  ## Note that we are just planning, not asking move_group 
  ## to actually move the robot
  plan1 = group.plan()

  print "============ Waiting while RVIZ displays init pose plan..."
  rospy.sleep(5)

 
  ## You can ask RVIZ to visualize a plan (aka trajectory) for you.  But the
  ## group.plan() method does this automatically so this is not that useful
  ## here (it just displays the same trajectory again).
  print "============ Visualizing init"
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()

  display_trajectory.trajectory_start = robot.get_current_state()
  display_trajectory.trajectory.append(plan1)
  display_trajectory_publisher.publish(display_trajectory);

  print "============ Waiting while init pose plan is visualized (again)..."
  rospy.sleep(5)
  group.go(wait=True)
  rospy.sleep(5)

  #group_variable_values = group.get_current_joint_values()
  #print "============ Joint new init values: ", group_variable_values




  ## Planning to a Pose goal

  ## Let's set a joint space goal and move towards it. 
  ## First, we will clear the pose target we had just set.

  group.clear_pose_targets()
  ## ^^^^^^^^^^^^^^^^^^^^^^^
  ## We can plan a motion for this group to a desired pose for the 
  ## end-effector
  print "============ Generating pose plan "
  print group.get_current_pose().pose
  pose_target = geometry_msgs.msg.Pose()
  pose_target = group.get_current_pose().pose
  pose_target.position.x -= 0.2
  #pose_target.position.y += 0.2
  #pose_target.position.z -= 0.2
  pose_target.orientation = quaternion.ROLL
  group.set_pose_target(pose_target)

  ## Now, we call the planner to compute the plan
  ## and visualize it if successful
  ## Note that we are just planning, not asking move_group 
  ## to actually move the robot
  plan1 = group.plan()

  print "============ Waiting while RVIZ displays pose plan..."
  rospy.sleep(5)

 
  ## You can ask RVIZ to visualize a plan (aka trajectory) for you.  But the
  ## group.plan() method does this automatically so this is not that useful
  ## here (it just displays the same trajectory again).
  print "============ Visualizing plan1"
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()

  display_trajectory.trajectory_start = robot.get_current_state()
  display_trajectory.trajectory.append(plan1)
  display_trajectory_publisher.publish(display_trajectory);

  print "============ Waiting while pose plan is visualized (again)..."
  rospy.sleep(5)
  #group.go(wait=True)
  rospy.sleep(5)


  print group.get_current_pose().pose
  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()

  ## END_TUTORIAL

  print "============ STOPPING"
  

def move_group_python_pose_subscriber():
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)
  #group = moveit_commander.MoveGroupCommander("right_arm")
  group = moveit_commander.MoveGroupCommander("manipulator")
  print "============ Show pose"
  pprint( dir(group.get_current_pose()))
  print group.get_current_pose().pose
  print group.get_current_pose().header



if __name__=='__main__':
  try:
    move_group_python_pose_subscriber()
    move_group_python_add_world()
    move_group_python_attach_phone()
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass

