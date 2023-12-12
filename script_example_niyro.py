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
        rospy.init_node('simple_docking_scenario')
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "niryo_arm"
        self.group = moveit_commander.MoveGroupCommander(group_name)


    def do_joint_move(self, arg):
        'moving joint to target joint'
        joint_goal = self.group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -np.pi/4
        joint_goal[2] = 0
        joint_goal[3] = -np.pi/2
        joint_goal[4] = 0
        joint_goal[5] = np.pi/3

        self.group.go(joint_goal, wait=True)
        self.group.stop()

    def do_pose_move(self, arg):
        'moving joint to target pose'
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = 0.5
        pose_goal.orientation.y = 0.5
        pose_goal.orientation.z = 0.5
        pose_goal.orientation.w = 0.5
        pose_goal.position.x = 0.245
        pose_goal.position.y = 0.0
        pose_goal.position.z = 0.5
        self.group.set_pose_target(pose_goal)
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

    def do_quit(self, arg):
        return True

    ## added scripts
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

if __name__ == '__main__':
    ControlSuiteShell().cmdloop()