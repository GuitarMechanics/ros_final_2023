import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Twist
import threading
import cmd, sys, os
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
import tf2_geometry_msgs
import tf2_ros
import actionlib
from actionlib_msgs.msg import GoalStatus as goal_status
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import copy
from moveit_commander.conversions import pose_to_list

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class ControlSuiteShell(cmd.Cmd):
    intro = bcolors.OKBLUE + "Welcome to the control suite shell.\nType help or ? to list commands.\n" + bcolors.ENDC
    prompt = "(csuite) "

    def __init__(self):
        cmd.Cmd.__init__(self)
        rospy.init_node('simple_inspection_scenario')
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "mobile_group"
        self.group = moveit_commander.MoveGroupCommander(group_name)    
        self.movebase_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

    def do_quit(self, arg):
        return True
    
    def do_pose_by_input(self, arg):
        'moving joint to target pose: using input data'
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = float(input("x pos : "))
        pose_goal.position.y = float(input("y pos : "))
        pose_goal.position.z = float(input("z pos : "))
        pose_goal.orientation.x = 0.5
        pose_goal.orientation.y = 0.5
        pose_goal.orientation.z = 0.5
        pose_goal.orientation.w = 0.5
        self.group.set_pose_target(pose_goal)
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()


    def do_nav_and_inspect(self, arg):
        posOffset = 2
        hyd_positions = [[3.2, -6.5],[-3, 4],[-8,-2.5]] ## position of hydrants in Gazebo
        for posDatas in hyd_positions:
            print("nav to : ", posDatas) ## begin navigation
            scoutGoal = MoveBaseGoal()
            scoutGoal.target_pose.header.frame_id = 'map'
            scoutGoal.target_pose.pose.position.x = float(posDatas[0] + posOffset)
            scoutGoal.target_pose.pose.position.y = float(posDatas[1])
            scoutGoal.target_pose.pose.position.z = 0
            scoutGoal.target_pose.pose.orientation.x = 0
            scoutGoal.target_pose.pose.orientation.y = 0
            scoutGoal.target_pose.pose.orientation.z = -1
            scoutGoal.target_pose.pose.orientation.w = 0.0
            self.movebase_client.send_goal_and_wait(scoutGoal)
            print("nav to ", posDatas," completed")

            joint_goal = self.group.get_current_joint_values() ## begin task
            armGoal = self.group.get_current_joint_values()
            
            armGoal[0] = np.deg2rad(0)
            armGoal[1] = np.deg2rad(-60)
            armGoal[2] = np.deg2rad(0)
            armGoal[3] = np.deg2rad(0)
            armGoal[4] = np.deg2rad(40)
            armGoal[5] = np.deg2rad(0)

            plan = self.group.go(armGoal,wait=True)
            self.group.stop()
            print("inspection finished, retracting arm")

            joint_goal = self.group.get_current_joint_values() ## task finished, returning arm
            joint_goal[0] = 0
            joint_goal[1] = 0
            joint_goal[2] = 0
            joint_goal[3] = 0
            joint_goal[4] = 0
            joint_goal[5] = 0

            self.group.go(joint_goal, wait=True)
            self.group.stop()

if __name__ == '__main__':
    ControlSuiteShell().cmdloop()