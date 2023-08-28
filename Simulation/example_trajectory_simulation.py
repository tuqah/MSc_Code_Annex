#!/usr/bin/env python
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2021 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

## import the necessary packages
import sys
import rospy
import time
import math
import copy
import actionlib
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from kortex_driver.srv import *
from kortex_driver.msg import *

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
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

class ExampleTrajectorySimulation(object):
    def __init__(self):
        # ensure that the parent class' constructor is executed before any additional
        # initialization specific to the ExampleTrajectorySimulation class is performed
        # Allows child class to leverage this behavior and attributes provided by the parent class
        super(ExampleTrajectorySimulation, self).__init__()

        ## Setup
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('example_trajectory_simulation', anonymous=True)

        try:
            self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
            if self.is_gripper_present:
                gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
                self.gripper_joint_name = gripper_joint_names[0]
            else:
                gripper_joint_name = ""
            self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

            ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
            ## the robot:
            self.robot = moveit_commander.RobotCommander("robot_description")

            ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
            ## to the world surrounding the robot:
            self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())

            ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
            ## to one group of joints.  In this case the group is the joints in the Kinova
            ## arm, so we set ``group_name = arm``.
            ## This interface can be used to plan and execute motions on Kinova:
            arm_group_name = "arm"
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())

            ## Create a `DisplayTrajectory`_ publisher which is used later to publish
            ## trajectories for RViz to visualize:
            self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)

            # Get the name of the reference frame for this robot:
            self.planning_frame = self.arm_group.get_planning_frame()
            print "============ Planning frame: %s" % self.planning_frame

            # Print the name of the end-effector link for this group, if present:
            self.eef_link = self.arm_group.get_end_effector_link()
            print "============ End effector link: %s" % self.eef_link

            # Get a list of all the groups in the robot:
            self.group_names = self.robot.get_group_names()
            print "============ Available Planning Groups:", self.robot.get_group_names()

            # Sometimes for debugging it is useful to print the entire state of the
            # robot, as done above and below:
            print "============ Printing robot state"
            print self.robot.get_current_state()
            print ""

        except Exception as e:
            print(e)
            self.is_init_success = False
        else:
            self.is_init_success = True

    ## while this function isn't used, it is kept to show that the robot is capable of
    ## physically executing the action immediately after it is planned, which helps in starting
    ## the movement immediately after the planned trajectory is approved by the user
    def reach_named_position(self, target):
        arm_group = self.arm_group

        # Going to one of those targets
        rospy.loginfo("Going to named target " + target)

        # Set the target
        arm_group.set_named_target(target)

        # Plan the trajectory
        #planned_path1 = arm_group.plan()
        arm_group.plan()

        user_input = raw_input("Type Y to Execute the Action")
        if user_input == 'Y':
            return arm_group.execute(arm_group.plan(), wait=True)
        else:
            return True

    def reach_cartesian_pose(self):
        arm_group = self.arm_group

        # Set the constants
        pi = 3.14159265358979323846

        ## dimensions of end-effector
        offset_vert = 0.297
        offset_horz = 0.015

        ## dimensions of the ring
        diameter = 0.010
        radius = diameter/2

        ## making sure that the end-effector abducts before it adducts
        scale = -1

        ## the same trajectories that were executed in the suturing waypoints are repeated here
        waypoints = []
        current_pose = arm_group.get_current_pose().pose
        current_pose.position.x += 0.01
        current_pose.position.z += 0.02
        waypoints.append(copy.deepcopy(current_pose))

        current_pose.position.x += 0.005 - offset_vert * (math.sin( 30 * pi / 180))
        current_pose.position.z += 0.000 - offset_vert * (1 - math.cos(30 * pi / 180))
        current_pose.orientation.w += scale*pi/12
        waypoints.append(copy.deepcopy(current_pose))

        current_pose.position.x += -0.010 + (-1*offset_vert)*(math.sin(15*pi/180) - math.sin(30*pi/180))
        current_pose.position.z += -0.020 + (-1*offset_vert)*((1-math.cos(15*pi/180)) - (1-math.cos(30*pi/180)))
        current_pose.orientation.w -= scale*pi/24
        waypoints.append(copy.deepcopy(current_pose))

        current_pose.position.x += -0.005 + (-1*offset_vert)*(math.sin(00*pi/180) - math.sin(15*pi/180))
        current_pose.position.z += -1 * radius + (-1*offset_vert)*((1-math.cos(00*pi/180)) - (1-math.cos(15*pi/180)))
        current_pose.orientation.w -= scale*pi/24
        waypoints.append(copy.deepcopy(current_pose))

        current_pose.position.x += -1 * offset_horz + offset_horz*(math.sin(30*pi/180) + math.sin(00*pi/180))
        current_pose.position.z += -1 * 0.030
        current_pose.orientation.w -= scale*pi/24
        waypoints.append(copy.deepcopy(current_pose))

        current_pose.position.x += 0.050
        waypoints.append(copy.deepcopy(current_pose))

        current_pose.orientation.w -= scale*pi/12
        waypoints.append(copy.deepcopy(current_pose))

        current_pose.position.x += 0.010
        waypoints.append(copy.deepcopy(current_pose))

        current_pose.position.x += 0.03
        current_pose.position.z -= 0.10
        current_pose.orientation.w -= scale*pi/8
        waypoints.append(copy.deepcopy(current_pose))

        current_pose.position.x += 0.05
        current_pose.position.z += 0.05
        waypoints.append(copy.deepcopy(current_pose))

        ## populate all the trajectories and put them in a planned cartesian path
        ## with the configurations being computed for every 0.001 step of the end-effector
        (plan, fraction) = arm_group.compute_cartesian_path(waypoints, 0.001, 0.0)
        return plan, fraction

    def display_trajectory(self, plan):
        ## connect to the robot
        robot = self.robot
        ## call the display trajectory node
        display_trajectory_publisher = self.display_trajectory_publisher

        ## send the message to display the planned path
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()

        ## get the state of the robot to check for collisions and if the robot can execute the plan
        display_trajectory.trajectory_start = robot.get_current_state()

        # add the plan from the calculated waypoints
        display_trajectory.trajectory.append(plan)

        # Publish and display the path
        display_trajectory_publisher.publish(display_trajectory);


    def main(self):

        try:
            print "----------------------------------------------------------"
            print "============ Press `Enter` to Setup the Moveit_commander and its associated Nodes"
            raw_input()
            example = ExampleTrajectorySimulation()

            success = example.is_init_success
            try:
                # If any exist, remove outdated configuration values from previous actions done with ROS
                # thus cleaning up and freeing up the used resoucres in ROS
                rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
            except:
                pass

            # print "============ Press `Enter` to Simulate and Possibly Execute an Action..."
            # raw_input()
            # if success:
            #     rospy.loginfo("Reaching Named Target Home...")
            #     success &= example.reach_named_position("home")
            #     print (success)

            print "============ Press `Enter` to Simulate the Action..."
            raw_input()
            if success:
                plan, fraction = example.reach_cartesian_pose()
                example.display_trajectory(plan)
                print (success)

            print "============ Simulation Done :D!"

        except rospy.ROSInterruptException:
            return

        except KeyboardInterrupt:
            return



if __name__ == "__main__":
    ex = ExampleTrajectorySimulation()
    ex.main()
