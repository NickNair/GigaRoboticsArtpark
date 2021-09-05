#!/bin/env python3

from itertools import count
from logging import debug
from os import access, name
from typing import OrderedDict
from matplotlib.pyplot import table, twinx
from numpy import angle, mat
from numpy.core.fromnumeric import put
from py_trees import behaviour, common
from py_trees.behaviours import Success
from roslaunch.pmon import rl_signal
import rospy
import roslaunch
import rosnode
import actionlib
import moveit_commander
import moveit_msgs.msg
import tf
import py_trees
import argparse
import sys
import time
import math
import py_trees.console as console
import subprocess

from std_msgs.msg import String, Int64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import JointConstraint, Constraints
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import LaserScan
from vision_msgs.msg import Detection2DArray
from tf import transformations
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion, Twist, PoseStamped 
from py_trees_test.msg import battery_status
from datetime import datetime
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from countertop_spray import Countertop
from pick_pipeline import PickPipeline
from name import BoundingBoxes
from fusion_class import fusion
from table_top import TableTop
from sink_detection import SinkDetection
from wall_align import WallAlign

#----------------- Meta data start -----------------# 

def description():
    content = "Bathroom cleaning robot demo\n"
    content += "The robot finds a way to enter the bathroom\n"
    content += "through the door, pick and place the trash in\n"
    content += "the dustbin, spray sanitizing liquid on the\n"
    content += "basin and countertop, mop the floor, wipe\n"
    content += "the basin and countertop and finally return\n"
    content += "to the position where it started from.\n"
    content += "\n"
    content += "EVENTS\n"
    content += "\n"
    content += " -  1 : Enter bathroom\n"
    content += " -  2 : Detect sink\n"
    content += " -  3 : Pick and place trash\n"
    content += " -  4 : Spray sanitizing liquid\n"
    content += " -  5 : Mop the floor\n"
    content += " -  6 : Wipe the basin and countertop\n"
    content += " -  7 : Return to its start position\n"
    content += "\n"
    if py_trees.console.has_colours:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = "\n"
        s += banner_line
        s += console.bold_white + "Artpark Robotics Challenge".center(79) + "\n" + console.reset
        s += banner_line
        s += "\n"
        s += content
        s += "\n"
        s += banner_line
    else:
        s = content
    return s

def epilog():
    if py_trees.console.has_colours:
        return console.cyan + "And his noodly appendage reached forth to tickle the blessed...\n" + console.reset
    else:
        return None


def command_line_argument_parser():
    parser = argparse.ArgumentParser(description=description(),
                                     epilog=epilog(),
                                     formatter_class=argparse.RawDescriptionHelpFormatter,
                                     )
    group = parser.add_mutually_exclusive_group()
    group.add_argument('-r', '--render', action='store_true', help='render dot tree to file')
    group.add_argument(
        '--render-with-blackboard-variables',
        action='store_true',
        help='render dot tree to file with blackboard variables'
    )
    group.add_argument('-i', '--interactive', action='store_true', help='pause and wait for keypress at each tick')
    return parser

def pre_tick_handler(behaviour_tree):
    print("\n--------- Run %s ---------\n" % behaviour_tree.count)

#----------------- Meta data end -----------------# 

#----------------- Moveit class start -----------------# 

class MoveitSetup:
    
    # Constructor
    def __init__(self,x):

        # Instatiating related obejcts
        self._planning_group = x
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        # Current State of the Robot is needed to add box to planning scene
        # self._curr_state = self._robot.get_current_state()

        # rospy.loginfo(
        #     '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        # rospy.loginfo(
        #     '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        # rospy.loginfo(
        #     '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        # rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    # Function to go to specified position
    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        # if (flag_plan == True):
        #     rospy.loginfo(
        #         '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        # else:
        #     rospy.loginfo(
        #         '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    # Function to set joint angles
    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        # if (flag_plan == True):
        #     rospy.loginfo(
        #         '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        # else:
        #     rospy.logerr(
        #         '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    # Function to go to pre defined position
    def go_to_predefined_pose(self, arg_pose_name):
        # rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        try:
            rospy.sleep(1)
            self._group.set_named_target(arg_pose_name)
            plan = self._group.go()
        except:
            pass

    def cartesian_path(self, waypoints):
        (plan, fraction) = self._group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        # 0.00005,        # eef_step
                                        0.001,
                                        0.0)         # jump_threshold
        self._group.execute(plan, wait=True)

    def cartesian_path2(self, waypoints):
        (plan, fraction) = self._group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.0001,        # eef_step
                                        0.0)         # jump_threshold
        self._group.execute(plan, wait=True)

    def init_stay_up_constraints(self):
        self.up_constraints = Constraints()
        joint_constraint = JointConstraint()

        self.up_constraints.name = "stay_up"

        joint_constraint.position = 0.7
        joint_constraint.tolerance_above = .1
        joint_constraint.tolerance_below = .1
        joint_constraint.weight = 1

        joint_constraint.joint_name = "apbot_joint"
        self.up_constraints.joint_constraints.append(joint_constraint)
        self._group.set_path_constraints(self.up_constraints)

    def init_wipe_constraints(self):
        self.wipe_constraints = Constraints()
        rotation_joint_constraint = JointConstraint()
        height_joint_constraint = JointConstraint()

        self.wipe_constraints.name = "wipe"

        rotation_joint_constraint.position = 0
        rotation_joint_constraint.tolerance_above = 1.6
        rotation_joint_constraint.tolerance_below = 1.6
        rotation_joint_constraint.weight = 1

        rotation_joint_constraint.joint_name = "shoulder_pan_joint"

        height_joint_constraint.position = .7
        height_joint_constraint.tolerance_above = .1
        height_joint_constraint.tolerance_below = .1
        height_joint_constraint.weight = 1

        height_joint_constraint.joint_name = "apbot_joint"

        self.wipe_constraints.joint_constraints.append(rotation_joint_constraint)
        self.wipe_constraints.joint_constraints.append(height_joint_constraint)
        
        self._group.set_path_constraints(self.wipe_constraints)

    def init_spray_constraints(self):
        self.spray_constraints = Constraints()
        rotation_joint_constraint = JointConstraint()
        height_joint_constraint = JointConstraint()
        elbow_joint_constraint = JointConstraint()

        self.spray_constraints.name = "spray"

        rotation_joint_constraint.position = 0
        rotation_joint_constraint.tolerance_above = 1.6
        rotation_joint_constraint.tolerance_below = 1.6
        rotation_joint_constraint.weight = 1

        rotation_joint_constraint.joint_name = "shoulder_pan_joint"

        height_joint_constraint.position = .7
        height_joint_constraint.tolerance_above = .1
        height_joint_constraint.tolerance_below = .1
        height_joint_constraint.weight = 1

        height_joint_constraint.joint_name = "apbot_joint"

        elbow_joint_constraint.position = 0
        elbow_joint_constraint.tolerance_above = 0
        elbow_joint_constraint.tolerance_below = -3.14
        elbow_joint_constraint.weight = 1

        height_joint_constraint.joint_name = "elbow_joint"

        self.spray_constraints.joint_constraints.append(rotation_joint_constraint)
        self.spray_constraints.joint_constraints.append(height_joint_constraint)
        self.spray_constraints.joint_constraints.append(elbow_joint_constraint)
        
        self._group.set_path_constraints(self.spray_constraints)

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        # rospy.loginfo(
        #     '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

#----------------- Moveit class end -----------------# 

# Move base goal function

def movebase_client(goal_x, goal_y, quat):
    
   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
   # Set frame id    
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
   # Set goal position
    goal.target_pose.pose.position.x = goal_x
    goal.target_pose.pose.position.y = goal_y
   # Set goal orientation 
    goal.target_pose.pose.orientation.z = quat[2]
    goal.target_pose.pose.orientation.w = quat[3]

   # Sends the goal to the action server.
    client.send_goal(goal)
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result() 
    
# Move base goal function end

#----------------- Tree nodes start -----------------#

# Initialize reusable variables
class InitializeReusables(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name="InitializeReusables")
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("initializations", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("start_coordinates", access=py_trees.common.Access.WRITE)

    def get_pose(self, data):      
        self.blackboard.start_coordinates = [data.pose.pose.position.x, data.pose.pose.position.y, 0]
        self.pose_sub.unregister()

    def setup(self):
        global client, apbot_arm, apbot_gripper, ct, bb, duf, markings_bb, door_bb, cf, df, tp, mp, table_top, sink, wall_align, pose_x, pose_y
        global dustbin_bb

        self.pose_sub = rospy.Subscriber('/odom', Odometry, self.get_pose)

        # Move base client
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        # Moveit class object
        apbot_arm = MoveitSetup("arm")
        apbot_gripper = MoveitSetup("gripper")

        # Countertop object
        ct = Countertop(apbot_arm)

        # Bounding boxes class for yolo
        bb = BoundingBoxes(trash_marking = True)

        markings_bb = BoundingBoxes(trash_marking=True, item_specific="markings")

        dustbin_bb = BoundingBoxes(item="dustbin", trash_marking=False, isDustbin=True)

        # Fusion class for cans
        cf = fusion(debug="trash")

        # Fusion class for door
        df = fusion(debug="marking")

        # Fusion class for dustbin
        duf = fusion()

        # Pick pipeline class
        tp = PickPipeline()

        # Pick pipeline for markings
        mp = PickPipeline()

        # Table top detection object
        table_top = TableTop()

        # Sink detection object
        sink = SinkDetection()

        # Wall align object
        wall_align = WallAlign()
    def update(self):
        return py_trees.common.Status.SUCCESS

# Send move base goal
class GoToGoal(py_trees.behaviour.Behaviour):
    def __init__(self, name, goal=[0,0,0], isStart = False, isCounter=False, isDustbin=False, isCenter=False, isDoor=False):
        super().__init__(name=name)  
        self.goal = goal
        self.isCounter = isCounter
        self.isDustbin = isDustbin
        self.isCenter = isCenter
        self.isDoor = isDoor
        self.isStart = isStart
        self.blackboard = self.attach_blackboard_client(name=self.name)
    
    def initialise(self):
        self.blackboard.register_key("countertop_coordinates", access=py_trees.common.Access.READ)
        self.blackboard.register_key("door_coordinates", access=py_trees.common.Access.READ)
        self.blackboard.register_key("center_coordinates", access=py_trees.common.Access.READ)
        self.blackboard.register_key("start_coordinates", access=py_trees.common.Access.READ)

        if self.isCounter:
            self.goal = self.blackboard.countertop_coordinates

        if self.isCenter:
            self.goal = self.blackboard.center_coordinates

        if self.isDoor:
            self.goal = self.blackboard.door_coordinates

        if self.isStart:
            self.goal = self.blackboard.start_coordinates

        # Convert euler angles to quaternion
        quat_angle = quaternion_from_euler(0, 0, self.goal[2])

        # Creates a new goal with the MoveBaseGoal constructor
        self.mb_goal = MoveBaseGoal()

        # Set frame id    
        self.mb_goal.target_pose.header.frame_id = "map"
        self.mb_goal.target_pose.header.stamp = rospy.Time.now()

        # Set goal position
        self.mb_goal.target_pose.pose.position.x = self.goal[0]
        self.mb_goal.target_pose.pose.position.y = self.goal[1]

        # Set goal orientation 
        self.mb_goal.target_pose.pose.orientation.x = quat_angle[0]
        self.mb_goal.target_pose.pose.orientation.y = quat_angle[1]
        self.mb_goal.target_pose.pose.orientation.z = quat_angle[2]
        self.mb_goal.target_pose.pose.orientation.w = quat_angle[3]
        # Sends the goal to the action server
        client.send_goal(self.mb_goal)
    
    def update(self):
        print(console.green + "---------------------------")

        state = client.get_state()

        if state==3:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

# Navigate to the front of the bathroom door
class ReachBathroomDoor(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name="ReachBathroomDoor")
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.complete = False

    def initialise(self):
        cmd = ["rosrun","apbot_nav","enter.py"]
        self.proc = subprocess.Popen(cmd) 

    def enter_done_callback(self, data):
        if data.data == "Done":
            self.complete = True

    def update(self):
        print("\n"  +console.green + "---------------------------")

        enter_done_sub = rospy.Subscriber('/enter_done', String, self.enter_done_callback)       

        if self.complete:
            enter_done_sub.unregister()
            self.proc.terminate()
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

# Get basic poses
class GetBasicPoses(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name="GetBasicPoses")
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("center_coordinates", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("door_coordinates", access=py_trees.common.Access.WRITE)

    def update(self):
        self.center_pose = PoseStamped()
        self.center_pose.header.frame_id = "base_footprint"
        self.center_pose.pose.position.x = 1.75
        self.center_pose.pose.position.y = .9
        self.center_pose.pose.position.z = .6
        quat_center = quaternion_from_euler(0,0,math.pi/2)
        self.center_pose.pose.orientation.x = quat_center[0]
        self.center_pose.pose.orientation.y = quat_center[1]
        self.center_pose.pose.orientation.z = quat_center[2]
        self.center_pose.pose.orientation.w = quat_center[3]

        self.door_pose = PoseStamped()
        self.door_pose.header.frame_id = "base_footprint"
        self.door_pose.pose.position.x = .75
        self.door_pose.pose.position.y = 0
        self.door_pose.pose.position.z = .6
        quat_door = quaternion_from_euler(0,0,0)
        self.door_pose.pose.orientation.x = quat_door[0]
        self.door_pose.pose.orientation.y = quat_door[1]
        self.door_pose.pose.orientation.z = quat_door[2]
        self.door_pose.pose.orientation.w = quat_door[3]

        while True:
            try:
                self.center_in_map = t.transformPose("/map", self.center_pose)
                break
            except:
                continue
        
        while True:
            try:
                self.door_in_map = t.transformPose("/map", self.door_pose)
                break
            except:
                continue

        center_in_map_quat = (self.center_in_map.pose.orientation.x, self.center_in_map.pose.orientation.y, self.center_in_map.pose.orientation.z, self.center_in_map.pose.orientation.w)
        door_in_map_quat = (self.door_in_map.pose.orientation.x, self.door_in_map.pose.orientation.y, self.door_in_map.pose.orientation.z, self.door_in_map.pose.orientation.w)
        
        center_z = euler_from_quaternion(center_in_map_quat)
        door_z = euler_from_quaternion(door_in_map_quat)

        self.blackboard.center_coordinates = [self.center_in_map.pose.position.x, self.center_in_map.pose.position.y, center_z[2]]
        self.blackboard.door_coordinates = [self.door_in_map.pose.position.x, self.door_in_map.pose.position.y, door_z[2]]

        return py_trees.common.Status.SUCCESS

# Set arm to pre-defined pose
class SetPredefinedArmPose(py_trees.behaviour.Behaviour):
    def __init__(self, pose_name, name):
        super().__init__(name=name)
        self.pose_name = pose_name
        self.blackboard = self.attach_blackboard_client(name=self.name)

    def update(self):
        print(console.yellow + "Sending arm to " + str(self.pose_name) + " pose")

        apbot_arm.go_to_predefined_pose(self.pose_name)
        rospy.sleep(1)

        return py_trees.common.Status.SUCCESS

# Set gripper to pre-defined pose
class SetPredefinedGripperPose(py_trees.behaviour.Behaviour):
    def __init__(self, pose_name, name):
        super().__init__(name=name)
        self.pose_name = pose_name
        self.blackboard = self.attach_blackboard_client(name=self.name)

    def update(self):
        print(console.yellow + "Sending gripper to " + str(self.pose_name) + " pose")
        
        for i in range(3):
            try:
                apbot_gripper.go_to_predefined_pose(self.pose_name)
            except:
                pass

        rospy.sleep(1)

        return py_trees.common.Status.SUCCESS 

# Detect countertop
class DetectCountertop(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name="DetectCounter")
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="counter_details",access=py_trees.common.Access.WRITE)

    def update(self):
        counter_details =  table_top.start_detection() 
        self.blackboard.counter_details = counter_details
        if self.blackboard.counter_details["Table Dimension"][1] < .2:
            self.blackboard.counter_details["Table Dimension"][1] = .6
        if self.blackboard.counter_details["Table Dimension"][0] < 1:
            self.blackboard.counter_details["Table Dimension"][0] = 1.2
        return py_trees.common.Status.SUCCESS

# Get countertop co-ordinates
class GetCounterCooridnates(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name="GetCounterCooridnates")
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="countertop_coordinates",access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="sink_side",access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="counter_details",access=py_trees.common.Access.READ)

    def initialise(self):
        print("\n"  +console.green + "---------------------------")

        print(console.yellow + "Getting countertop co-ordinates...")

    def update(self):
        counter_details = self.blackboard.counter_details

        if counter_details["Sink Center"].pose.position.x > counter_details["Table Center"].pose.position.x:
            self.blackboard.sink_side = "Left"
        else:
            self.blackboard.sink_side = "Right"

        while True:
            try:
                counter_pose_in_map = t.transformPose("/map", counter_details["Table Center"])
                break
            except:
                continue
        
        counter_in_map_quat = (counter_pose_in_map.pose.orientation.x, counter_pose_in_map.pose.orientation.y, counter_pose_in_map.pose.orientation.z, counter_pose_in_map.pose.orientation.w)

        counter_z = euler_from_quaternion(counter_in_map_quat)

        self.blackboard.countertop_coordinates = (counter_pose_in_map.pose.position.x - .7*math.cos(counter_z[2]), counter_pose_in_map.pose.position.y - .7*math.sin(counter_z[2]), counter_z[2])

        return py_trees.common.Status.SUCCESS

# Detect Sink
class DetectSink(py_trees.behaviour.Behaviour):
    def __init__(self, side):
        super().__init__(name="DetectSink"+side)
        self.side = side
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="sink_side",access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="sink_details",access=py_trees.common.Access.WRITE)

    def update(self):
        print(self.blackboard.sink_side)
        if self.side == self.blackboard.sink_side:
            apbot_arm.go_to_predefined_pose("sink_detect")
            sink_details =  sink.start_sink_detection() 
            if sink_details:
                self.blackboard.sink_details = sink_details
            else:
                self.blackboard.sink_details = "NA"
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.SUCCESS

# Align robot at counter
class CounterAlign(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name=name)

    def initialise(self):
        self.aligned = False
        self.dist_fixed = False
        self.counter_aligned = False
        wall_align.t0 = rospy.get_time()

    def update(self):
        if not self.aligned:
            self.aligned = wall_align.align()
            return py_trees.common.Status.RUNNING
        if not self.dist_fixed:
            self.dist_fixed = wall_align.dist_fix()
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS

# Go to right side of counter
class GoRight(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="countertop_coordinates",access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="counter_details",access=py_trees.common.Access.READ)
    
    def initialise(self):
        print("\n"  +console.green + "---------------------------")

        print(console.yellow + "Going right...")

        self.counter_coordinates = self.blackboard.countertop_coordinates
        self.table_length = self.blackboard.counter_details["Table Dimension"][0]

    def update(self):
        counter_pose  =PoseStamped()
        counter_pose.header.frame_id = "/map"
        counter_pose.pose.position.x = self.counter_coordinates[0]
        counter_pose.pose.position.y = self.counter_coordinates[1]
        while True:
            try:
                counter_in_base = t.transformPose("/base_footprint", counter_pose)
                break
            except:
                continue
        
        y_counter = counter_in_base.pose.position.y

        vel = Twist()
        
        if (y_counter - self.table_length/4) < -0.01:
            vel.linear.y = (y_counter - self.table_length/4)*.2
            vel_pub.publish(vel)
            return py_trees.common.Status.RUNNING
        else:
            vel.linear.y = 0
            vel_pub.publish(vel)
            
            return py_trees.common.Status.SUCCESS

# Go to left side of counter
class GoLeft(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="countertop_coordinates",access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="counter_details",access=py_trees.common.Access.READ)
    
    def initialise(self):
        print("\n"  +console.green + "---------------------------")

        print(console.yellow + "Going left...")

        self.counter_coordinates = self.blackboard.countertop_coordinates
        self.table_length = self.blackboard.counter_details["Table Dimension"][0]

    def update(self):
        counter_pose  =PoseStamped()
        counter_pose.header.frame_id = "/map"
        counter_pose.pose.position.x = self.counter_coordinates[0]
        counter_pose.pose.position.y = self.counter_coordinates[1]
        while True:
            try:
                counter_in_base = t.transformPose("/base_footprint", counter_pose)
                break
            except:
                continue
        
        y_counter = counter_in_base.pose.position.y

        vel = Twist()
        
        if (y_counter + self.table_length/4) > 0.01:
            vel.linear.y = (y_counter + self.table_length/4)*.2
            vel_pub.publish(vel)
            return py_trees.common.Status.RUNNING
        else:
            vel.linear.y = 0
            vel_pub.publish(vel)
            
            return py_trees.common.Status.SUCCESS

# Spray countertop
class CountertopSpray(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, side):
        super().__init__(name=name)
        self.side = side
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="sink_side",access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="counter_details",access=py_trees.common.Access.READ)
    
    def initialise(self):
        print("\n"  +console.green + "---------------------------")

        print(console.yellow + "Spraying countertop")

    def update(self):
        counter_dimensions = self.blackboard.counter_details["Table Dimension"]
        
        ct.wipe_right(counter_dimensions)
        rospy.sleep(1)

        return py_trees.common.Status.SUCCESS

# Spray countertop
class CountertopWipe(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, side):
        super().__init__(name=name)
        self.side = side
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="sink_side",access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="counter_details",access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="sink_details",access=py_trees.common.Access.READ)
    
    def initialise(self):
        print("\n"  +console.green + "---------------------------")

        print(console.yellow + "Wiping countertop")

    def update(self):
        counter_dimensions = self.blackboard.counter_details["Table Dimension"]

        if self.blackboard.sink_side == self.side:
            if self.blackboard.sink_details == "NA":
                return py_trees.common.Status.SUCCESS
                
            sink_dimensions = self.blackboard.sink_details
            sink_xy = [sink_dimensions["Sink Center"][0], sink_dimensions["Sink Center"][1]]
            sink_height = sink_dimensions["Sink Center"][2]
                
            ct.sink_clean(sink_xy, sink_height, sink_dimensions["Sink Top Dimensions"], sink_dimensions["Sink Bottom Dimensions"])
            rospy.sleep(1)

            ct.sink_side_clean_right(sink_xy, counter_dimensions, sink_dimensions["Sink Top Dimensions"])
            rospy.sleep(1)

            return py_trees.common.Status.SUCCESS
        else:
            ct.spray_right(counter_dimensions)

            rospy.sleep(.1)

            ct.spray_left(counter_dimensions)

            return py_trees.common.Status.SUCCESS

# Attach/Detach trash tray
class JointAttach(py_trees.behaviour.Behaviour):
    def __init__(self, name, link1, link2, attach):
        super().__init__(name=name)
        self.link1 = link1
        self.link2 = link2
        self.attach = attach

    def initialise(self):
        self.req = AttachRequest()
        self.req.model_name_1 = "apbot"
        self.req.link_name_1 = self.link1
        self.req.model_name_2 = "apbot"
        self.req.link_name_2 = self.link2

        if self.attach:
            self.joint_service = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
            self.joint_service.wait_for_service()

        else:
            self.joint_service = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
            self.joint_service.wait_for_service()

    def update(self):
        self.joint_service.call(self.req)

        return py_trees.common.Status.SUCCESS

# Attach/Detach bot
class BotAttach(py_trees.behaviour.Behaviour):
    def __init__(self, name, attach):
        super().__init__(name=name)
        self.attach = attach

    def initialise(self):
        self.req = AttachRequest()
        self.req.model_name_1 = "artpark_world"
        self.req.link_name_1 = "wall_front_1"
        self.req.model_name_2 = "apbot"
        self.req.link_name_2 = "base_footprint"

        if self.attach:
            self.joint_service = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
            self.joint_service.wait_for_service()

        else:
            self.joint_service = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
            self.joint_service.wait_for_service()

    def update(self):
        self.joint_service.call(self.req)

        return py_trees.common.Status.SUCCESS

# Attach/detach trash with tray
class TrashAttach(py_trees.behaviour.Behaviour):
    def __init__(self, name, attach):
        super().__init__(name=name)
        self.attach = attach
    
    def update(self):
        tp.trash_tray_joints(attach=self.attach)

        rospy.sleep(1)

        return py_trees.common.Status.SUCCESS

# Reinitialize namy.py var
class ReName(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name=name)

    def update(self):
        bb.__init__(item="dustbin", trash_marking=False, isDustbin=True)
        return py_trees.common.Status.SUCCESS

# Start dustbin detection
class StartDustbinDetect(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name=name)

    def update(self):
        bb.listener(True)
        # rospy.sleep(1)
        return py_trees.common.Status.SUCCESS

# Stop dustbin detection
class StopDustbinDetect(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name=name)

    def update(self):
        bb.listener(False)
        # rospy.sleep(1)
        return py_trees.common.Status.SUCCESS

# Set dustbin detected to false
class DustbinFalse(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name="DustbinDetect")
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="has_dustbin_detected", access=py_trees.common.Access.WRITE)
    
    def update(self):
        bb.listener(False)
        self.blackboard.has_dustbin_detected = False
        objects_none_pub = rospy.Publisher('/objects', Detection2DArray, queue_size=10)
        arr = Detection2DArray()
        arr.detections = []
        objects_none_pub.publish(arr)
        return py_trees.common.Status.SUCCESS

# Start dustbin fusion
class StartDustbinFusion(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="has_dustbin_detected", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="dustbin_fusion_coordinates", access=py_trees.common.Access.WRITE)
        self.blackboard.has_dustbin_detected = False

    def update(self):
        duf.listener_dustbin(True)
        rospy.sleep(3)
        has_detected = duf.get_dustbin_pose()

        if has_detected:
            self.blackboard.has_dustbin_detected = True
            self.blackboard.dustbin_fusion_coordinates = has_detected

        return py_trees.common.Status.SUCCESS

# Start dustbin fusion
class UpdateDustbinFusion(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="has_dustbin_detected", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="dustbin_fusion_coordinates", access=py_trees.common.Access.WRITE)

    def update(self):
        rospy.sleep(3)
        has_detected = duf.get_dustbin_pose()

        if has_detected:
            self.blackboard.has_dustbin_detected = True
            self.blackboard.dustbin_fusion_coordinates = has_detected

        return py_trees.common.Status.SUCCESS

# Go to dustbin
class GoToDustbin(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="dustbin_fusion_coordinates", access=py_trees.common.Access.READ)
        self.got_laser = False

    def laser_callback(self, data):
        self.got_laser = True
        self.laser_data = data.ranges
        self.laser_sub.unregister()

    def update(self):
        while not self.got_laser:
            self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        dbc = self.blackboard.dustbin_fusion_coordinates
        vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        vel = Twist()
        vel.angular.z = -.2
        vel_pub.publish(vel)
        rospy.sleep(3)
        vel.angular.z = 0
        vel_pub.publish(vel)
        result = movebase_client(dbc[0], dbc[1], dbc[2])
        vel = Twist()
        vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        if result:
            vel.linear.x = .05
            vel_pub.publish(vel)
            # rospy.sleep(self.laser_data[180]//.05)
            if self.laser_data[180] > .1:                
                rospy.sleep(2)
            elif self.laser_data[180] > .15:
                rospy.sleep(3)
            elif self.laser_data[180] > .2:
                rospy.sleep(4)
            elif self.laser_data[180] > .25:
                rospy.sleep(5)
            elif self.laser_data[180] > .3:
                rospy.sleep(6)
            elif self.laser_data[180] > .35:
                rospy.sleep(7)
            elif self.laser_data[180] > .4:
                rospy.sleep(8)
            vel.linear.x = 0
            vel_pub.publish(vel)
        return py_trees.common.Status.SUCCESS

# Stop dustbin fusion
class StopDustbinFusion(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name=name)

    def update(self):
        duf.listener_dustbin(False)
        return py_trees.common.Status.SUCCESS

# Has dustbin been detected
class HasDustbinDetected(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="has_dustbin_detected", access=py_trees.common.Access.READ)

    def update(self):
        dustbin_detection_status = self.blackboard.has_dustbin_detected
        if dustbin_detection_status:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

# Turn bot to detect dustbin
class TurnBotDustbin(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="has_dustbin_detected", access=py_trees.common.Access.READ)
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)    

    def update(self):
        if not self.blackboard.has_dustbin_detected:
            vel = Twist()
            vel.angular.z = -.3

            self.pub_vel.publish(vel)
            rospy.sleep(4)   
            vel.angular.z = 0
            self.pub_vel.publish(vel)
        return py_trees.common.Status.SUCCESS

# Start trash detection
class StartDetect(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name=name)

    def update(self):
        bb.listener(True)
        return py_trees.common.Status.SUCCESS

# Stop trash detection
class StopDetect(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name=name)

    def update(self):
        bb.listener(False)
        return py_trees.common.Status.SUCCESS

# Start marking detection
class MarkingDetect(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name=name)

    def update(self):
        markings_bb.listener(True)
        return py_trees.common.Status.SUCCESS

# Stop marking detection
class MarkingDetectStop(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name=name)

    def update(self):
        markings_bb.listener(False)
        return py_trees.common.Status.SUCCESS

# Publish check
class PublishCheck(py_trees.behaviour.Behaviour):
    def __init__(self, name, on_or_off):
        super().__init__(name=name)
        self.on_or_off = on_or_off
        self.check_pub = rospy.Publisher("/check", String, queue_size=1, latch=True)
    
    def update(self):
        if self.on_or_off == "on":
            rospy.sleep(3)
            self.check_pub.publish("Detect")
        else:
            rospy.sleep(4)
            self.check_pub.publish("stop")
        return py_trees.common.Status.SUCCESS

# Start fusion
class StartFusion(py_trees.behaviour.Behaviour):
    def __init__(self, name, item="trash"):
        super().__init__(name=name)
        self.item = item

    def update(self):
        # if self.item=="trash":
        #     cf.listener(True)
        # else:
        #     df.listener(True, item="marking")
        if self.item=="marking":
            cf.__init__(debug="marking")
        cf.listener(True)
        return py_trees.common.Status.SUCCESS

# End fusion
class EndFusion(py_trees.behaviour.Behaviour):
    def __init__(self, name, item="trash"):
        super().__init__(name=name)
        self.item = item

    def update(self):
        cf.listener(False)
        return py_trees.common.Status.SUCCESS

# Clean Entrance
class CleanEntrance(py_trees.behaviour.Behaviour):
    def __init__(self, name, entrance_status):
        super().__init__(name=name)
        self.entrance_status = entrance_status

    def check_callback(self, data):
        if data.data=="stop":
            self.check_status = True

    def initialise(self):
        self.check_status = False
        tp.entrance = self.entrance_status

    def update(self):
        while not self.check_status:
            rospy.Subscriber("check", String, self.check_callback, queue_size=10)

        tp.go_to_object(apbot_arm, apbot_gripper)

        return py_trees.common.Status.SUCCESS

# Clean markings
class CleanMarking(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name=name)

    def check_callback(self, data):
        if data.data=="stop":
            self.check_status = True

    def initialise(self):
        self.check_status = False

    def update(self):
        # while not self.check_status:
        #     rospy.Subscriber("check", String, self.check_callback, queue_size=10)

        mp.go_to_markings(apbot_arm)

        return py_trees.common.Status.SUCCESS

# Drop Trash
class DropTrash(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name=name)

    def update(self):
        apbot_arm.go_to_predefined_pose("tray_up")
        rospy.sleep(.2)

        apbot_arm.go_to_predefined_pose("before_drop1")
        rospy.sleep(.2)

        apbot_arm.go_to_predefined_pose("before_drop2")
        rospy.sleep(.2)

        apbot_arm.go_to_predefined_pose("drop_trash3")
        rospy.sleep(2)

        apbot_arm.go_to_predefined_pose("before_drop2")
        rospy.sleep(.2)

        apbot_arm.go_to_predefined_pose("before_drop1")
        rospy.sleep(.2)   

        apbot_arm.go_to_predefined_pose("tray_up")
        rospy.sleep(.2)     

        return py_trees.common.Status.SUCCESS

# Exit Door
class ExitDoor(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name=name)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def update(self):
        vel = Twist()
        vel.linear.x = -.35
        self.vel_pub.publish(vel)
        rospy.sleep(3)
        vel.linear.x = 0
        self.vel_pub.publish(vel)
        return py_trees.common.Status.SUCCESS

#----------------- Tree nodes end -----------------#

# Create tree
def create_tree():

    # Initial use
    init_reusables = InitializeReusables()
    tray_base_attach1 = JointAttach(name="base_tray_attach1", link1="trash_tray_1", link2="base_footprint", attach=True)
    sponge_tray_attach1 = JointAttach(name="sponge_tray_attach1", link1="sponge", link2="base_footprint", attach=True)
    tray_gripper_detach = JointAttach(name="trash_gripper_detach", link1="trash_tray_1", link2="gripper_base_link", attach=False)

    reach_door_front = ReachBathroomDoor()
    get_basic_poses = GetBasicPoses()

    # Door sequence end

    # Countertop & sink detection
    detect_counter_pose1 = SetPredefinedArmPose("look_left", name="DetectCountertop1")
    detect_counter_pose2 = SetPredefinedArmPose("look_left", name="DetectCountertop2")
    detect_counter_pose_up = SetPredefinedArmPose("counter_door_detect", name="DetectCountertopUp")
    detect_counter = DetectCountertop() 
    get_counter_coordinates = GetCounterCooridnates() 

    parallel_counter_detect = py_trees.composites.Parallel(name="ParallelCounterDetect", policy=py_trees.common.ParallelPolicy.SuccessOnOne())

    parallel_counter_detect.add_children([detect_counter, get_counter_coordinates])

    # Dustbin detect sequence

    go_to_center_dustbin = GoToGoal(name="DustbinDetectCenter", isCenter=True)
    go_to_dustbin_detect_pose = SetPredefinedArmPose(name="DustbinDetectPose", pose_name="dustbin_detect")

    dustbin_false = DustbinFalse()
    
    has_dustbin_detected = HasDustbinDetected(name="HasDustbinDetected")
    start_dustbin_detection = StartDustbinDetect(name="StartDustbinDetect")
    start_dustbin_fusion = StartDustbinFusion(name="StartDustbinFusion")
    publish_check_dustbin = PublishCheck(name="PublishCheckDustbin", on_or_off="on")
    stop_dustbin_detection = StopDustbinDetect(name="StopDustbinDetect")
    publish_check_dustbin_off = PublishCheck(name="PublishCheckDustbinOff", on_or_off="off")
    update_dustbn_fusion = UpdateDustbinFusion(name="UpdateDustbinFusion")
    turn_bot_dustbin = TurnBotDustbin(name="TurnBotDustbin")
    
    dustbin_sequence = py_trees.composites.Sequence(name="DustbinSequence")

    dustbin_sequence.add_children(children=[ 
        start_dustbin_detection,
        start_dustbin_fusion,
        # publish_check_dustbin,
        stop_dustbin_detection,
        # publish_check_dustbin_off,
        update_dustbn_fusion,
        turn_bot_dustbin,])
    
    dustbin_sequence_failure = py_trees.decorators.SuccessIsFailure(child=dustbin_sequence)

    dustbin_selector = py_trees.composites.Selector(name="DustbinSelector")
    dustbin_selector.add_children(children=[has_dustbin_detected, dustbin_sequence_failure])

    go_to_dustbin = GoToDustbin(name="GoToDustbin")

    # Dustbin detect sequence end

    # Trash drop
    create_trash_joints = TrashAttach(attach=True, name="AttachTrash")
    go_to_tray_top1 = SetPredefinedArmPose("tray_top", name="TrayTop1")
    gripper_open1 = SetPredefinedGripperPose("gripper_open", name="GripperOpen1")
    go_to_tray_grab1 = SetPredefinedArmPose("tray_grab", name="TrayGrab1")
    gripper_tray_grab = SetPredefinedGripperPose("close_full", name="GripperTrayGrab")
    tray_base_detach = JointAttach(name="tray_base_detach", link1="trash_tray_1", link2="base_footprint", attach=False)
    tray_gripper_attach = JointAttach(name="tray_gripper_attach", link1="trash_tray_1", link2="gripper_base_link", attach=True)
    go_to_tray_up1 = SetPredefinedArmPose("tray_up", name="TrayUp1")
    before_drop1 = SetPredefinedArmPose("before_drop1", name="BeforeDrop1")
    drop_trash_pose = SetPredefinedArmPose("drop_trash5", name="DropTrashPose1")
    timer_wait = py_trees.behaviours.TickCounter(duration=2)
    drop_trash = DropTrash(name="DropTrash")
    break_trash_joints = TrashAttach(attach=False, name="DetachTrash")
    go_to_tray_up2 = SetPredefinedArmPose("tray_up", name="TrayUp2")
    go_to_tray_top2 = SetPredefinedArmPose("tray_top", name="TrayTop2")
    go_to_tray_grab2 = SetPredefinedArmPose("tray_grab", name="TrayGrab2")
    tray_gripper_detach = JointAttach(name="trash_gripper_detach", link1="trash_tray_1", link2="gripper_base_link", attach=False)
    gripper_open2 = SetPredefinedGripperPose("gripper_open", name="OpenGripper2")
    tray_base_attach2 = JointAttach(name="base_tray_attach2", link1="trash_tray_1", link2="base_footprint", attach=True)
    trash_drop_sequence = py_trees.composites.Sequence(name="TrashDropSequence")
    go_to_tray_top3 = SetPredefinedArmPose("tray_top", name="TrayTop3")
    go_to_start = SetPredefinedArmPose("start", name="trash_drop_start")

    trash_drop_sequence.add_children(
        [
        create_trash_joints,
        go_to_tray_top1, 
        gripper_open1,
        go_to_tray_grab1, 
        gripper_tray_grab, 
        tray_base_detach,
        tray_gripper_attach, 
        go_to_tray_up1,
        before_drop1,
        drop_trash_pose,
        timer_wait,
        break_trash_joints,
        go_to_tray_up2,
        go_to_tray_grab2,
        tray_gripper_detach,
        gripper_open2,
        tray_base_attach2,
        go_to_tray_top3]
        )

    trash_drop_oneshot = py_trees.idioms.oneshot(
        behaviour=trash_drop_sequence, 
        name="TrashDropOneshot",
        policy=common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION
        )

    # Initializations tree
    parllel_init = py_trees.composites.Parallel("ParallelInit", policy=py_trees.common.ParallelPolicy.SuccessOnAll())

    parllel_init.add_children([init_reusables, tray_base_attach1, sponge_tray_attach1])
    # Initializations tree end

    #----- Entrance -----#

    # Entrance trash parallel fusion & pick
    trash_detect_pose_entrance = SetPredefinedArmPose("trash_detect", name="EntranceDetect")

    publish_check_entrance = PublishCheck(name="PublishCheckEntrance", on_or_off="on")

    publish_check_entrance_off = PublishCheck(name="PublishCheckEntrance", on_or_off="off")

    start_detect_entrance = StartDetect(name="SDEntrance")

    start_entrance_fusion = StartFusion(name="EntranceFusion")

    clean_entrance = CleanEntrance(name="CleanEntrance", entrance_status=1)

    entrance_sequence = py_trees.composites.Sequence("EntranceSequence")

    stop_detection_entrance = StopDetect(name="StopDetectionEntrance")

    go_to_door = GoToGoal(name="Door", isDoor=True)

    entrance_sequence.add_children([start_detect_entrance, start_entrance_fusion, publish_check_entrance, stop_detection_entrance, publish_check_entrance_off, clean_entrance, go_to_door])

    # Entrance trash parallel fusion & pick end

    #----- Entrance End -----#

    #----- Trash detection after entering -----#

    bot_attach_trash_detection = BotAttach(attach=True, name="BotAttachTrashDetection")

    trash_detect_pose1 = SetPredefinedArmPose("trash_detect", name="TrashDetect1")
    trash_detect_far_pose1 = SetPredefinedArmPose("trash_detect_far", name="TrashDetectFar1")
    trash_detect_left_pose1 = SetPredefinedArmPose("trash_detect_left", name="TrashDetectLeft1")
    trash_detect_right_pose1 = SetPredefinedArmPose("trash_detect_right", name="TrashDetectRight1")
    trash_detect_lf_pose1 = SetPredefinedArmPose("trash_detect_lf", name="TrashDetectLf1")
    trash_detect_rf_pose1 = SetPredefinedArmPose("trash_detect_rf", name="TrashDetectRf1")
    trash_detect_left_far_pose1 = SetPredefinedArmPose("trash_detect_left_far", name="TrashDetectLeftFar1")
    trash_detect_right_far_pose1 = SetPredefinedArmPose("trash_detect_right_far", name="TrashDetectRightFar1")
    trash_detect_lf_far_pose1 = SetPredefinedArmPose("trash_detect_lf_far", name="TrashDetectLfFar1")
    trash_detect_rf_far_pose1 = SetPredefinedArmPose("trash_detect_rf_far", name="TrashDetectRfFar1")

    publish_check1 = PublishCheck(name="PublishCheck1", on_or_off="on")
    publish_check2 = PublishCheck(name="PublishCheck2", on_or_off="on")
    publish_check3 = PublishCheck(name="PublishCheck3", on_or_off="on")
    publish_check4 = PublishCheck(name="PublishCheck4", on_or_off="on")
    publish_check5 = PublishCheck(name="PublishCheck5", on_or_off="on")
    publish_check6 = PublishCheck(name="PublishCheck6", on_or_off="on")
    publish_check7 = PublishCheck(name="PublishCheck7", on_or_off="on")
    publish_check8 = PublishCheck(name="PublishCheck8", on_or_off="on")
    publish_check9 = PublishCheck(name="PublishCheck9", on_or_off="on")
    publish_check10 = PublishCheck(name="PublishCheck10", on_or_off="on")

    publish_check_off1 = PublishCheck(name="PublishCheck1", on_or_off="off")
    publish_check_off2 = PublishCheck(name="PublishCheck2", on_or_off="off")
    publish_check_off3 = PublishCheck(name="PublishCheck3", on_or_off="off")
    publish_check_off4 = PublishCheck(name="PublishCheck4", on_or_off="off")
    publish_check_off5 = PublishCheck(name="PublishCheck5", on_or_off="off")
    publish_check_off6 = PublishCheck(name="PublishCheck6", on_or_off="off")
    publish_check_off7 = PublishCheck(name="PublishCheck7", on_or_off="off")
    publish_check_off8 = PublishCheck(name="PublishCheck8", on_or_off="off")
    publish_check_off9 = PublishCheck(name="PublishCheck9", on_or_off="off")
    publish_check_off10 = PublishCheck(name="PublishCheck10", on_or_off="off")

    start_trash_fusion1 = StartFusion(name="TrashFusion1")
    start_trash_fusion2 = StartFusion(name="TrashFusion2")
    start_trash_fusion3 = StartFusion(name="TrashFusion3")
    start_trash_fusion4 = StartFusion(name="TrashFusion4")
    start_trash_fusion5 = StartFusion(name="TrashFusion5")
    start_trash_fusion6 = StartFusion(name="TrashFusion6")
    start_trash_fusion7 = StartFusion(name="TrashFusion7")
    start_trash_fusion8 = StartFusion(name="TrashFusion8")

    end_trash_fusion1 = EndFusion(name="EndTrashFusion1")
    end_trash_fusion2 = EndFusion(name="EndTrashFusion2")
    end_trash_fusion3 = EndFusion(name="EndTrashFusion3")
    end_trash_fusion4 = EndFusion(name="EndTrashFusion4")
    end_trash_fusion5 = EndFusion(name="EndTrashFusion5")
    end_trash_fusion6 = EndFusion(name="EndTrashFusion6")
    end_trash_fusion7 = EndFusion(name="EndTrashFusion7")
    end_trash_fusion8 = EndFusion(name="EndTrashFusion8")

    start_detect1 = StartDetect(name="SD1")
    start_detect2 = StartDetect(name="SD2")
    start_detect3 = StartDetect(name="SD3")
    start_detect4 = StartDetect(name="SD4")
    start_detect5 = StartDetect(name="SD5")
    start_detect6 = StartDetect(name="SD6")
    start_detect7 = StartDetect(name="SD7")
    start_detect8 = StartDetect(name="SD8")
    start_detect9 = StartDetect(name="SD9")
    start_detect10 = StartDetect(name="SD10")

    stop_detection1 = StopDetect(name="StopDetection1")
    stop_detection2 = StopDetect(name="StopDetection2")
    stop_detection3 = StopDetect(name="StopDetection3")
    stop_detection4 = StopDetect(name="StopDetection4")
    stop_detection5 = StopDetect(name="StopDetection5")
    stop_detection6 = StopDetect(name="StopDetection6")
    stop_detection7 = StopDetect(name="StopDetection7")
    stop_detection8 = StopDetect(name="StopDetection8")
    stop_detection9 = StopDetect(name="StopDetection9")
    stop_detection10 = StopDetect(name="StopDetection10")

    bot_detach_trash_detection = BotAttach(attach=False, name="BotDetachTrashDetection")

    trash_detect_sequence = py_trees.composites.Sequence("TrashDetectSequence")

    trash_detect_sequence.add_children(
        [bot_attach_trash_detection,
        trash_detect_left_pose1, 
        start_detect1, 
        start_trash_fusion1, 
        publish_check1, 
        stop_detection1,
        publish_check_off1,
        # end_trash_fusion1,
        trash_detect_left_far_pose1,
        start_detect2,
        # start_trash_fusion2,
        publish_check2, 
        stop_detection2,
        publish_check_off2,
        # end_trash_fusion2,
        trash_detect_lf_pose1,
        start_detect3,
        # start_trash_fusion3,
        publish_check3, 
        stop_detection3,
        publish_check_off3,
        # end_trash_fusion3,
        trash_detect_lf_far_pose1,
        start_detect4,
        # start_trash_fusion4,
        publish_check4, 
        stop_detection4,
        publish_check_off4,
        # end_trash_fusion4,
        trash_detect_pose1,
        start_detect5,
        # start_trash_fusion5,
        publish_check5, 
        stop_detection5,
        publish_check_off5,
        # end_trash_fusion5,
        trash_detect_far_pose1,
        start_detect6,
        # start_trash_fusion6,
        publish_check6, 
        stop_detection6,
        publish_check_off6,
        # end_trash_fusion6,
        # trash_detect_rf_pose1,
        # start_detect7,
        # publish_check7, 
        # stop_detection7,
        # publish_check_off7,
        trash_detect_rf_far_pose1,
        start_detect8,
        # start_trash_fusion8,
        publish_check8, 
        stop_detection8,
        publish_check_off8,
        # end_trash_fusion8,
        # start_trash_fusion7,
        # trash_detect_right_pose1,
        # start_detect9,
        # publish_check9, 
        # stop_detection9,
        # publish_check_off9,
        # trash_detect_right_far_pose1,
        # start_detect10,
        # publish_check10, 
        # stop_detection10,
        # publish_check_off10,
        bot_detach_trash_detection
        ]
    )

    end_entrance_fusion = EndFusion(name="EndEntranceFusion")

    clear_trash = CleanEntrance(name="ClearTrash1", entrance_status=0)

    #----- Trash detection after entering end -----#

    # Trash drop sub sequence

    # Trash drop sub sequence end

    # Countertop spray sequence

    go_to_center1 = GoToGoal(name="Center1", isCenter=True)
    go_to_countertop = GoToGoal(name='CountertopGoal', isCounter=True) 
    counter_align1 = CounterAlign(name="Align1")
    counter_align2 = CounterAlign(name="Align2")
    bot_attach_spray1 = BotAttach(attach=True, name="BotAttachSpray1")
    bot_attach_spray2 = BotAttach(attach=True, name="BotAttachSpray2")
    bot_detach_spray1 = BotAttach(attach=False, name="BotDetachSpray1")
    bot_detach_spray2 = BotAttach(attach=False, name="BotDetachSpray2")
    counter_align3 = CounterAlign(name="Align3")
    counter_align4 = CounterAlign(name="Align4")
    counter_right1 = GoRight(name="CounterRight1")
    counter_left1 = GoLeft(name="CounterLeft1")
    countertop_spray1 = CountertopSpray("Spray1", side="Right")
    countertop_spray2 = CountertopSpray("Spray2", side="Left")
    arm_up1 = SetPredefinedArmPose("up", name="ArmUp1")
    arm_start1 = SetPredefinedArmPose("start", name="ArmStart1")
    detect_sink_left = DetectSink(side="Left") 
    detect_sink_right = DetectSink(side="Right")

    countertop_spray_sequence = py_trees.composites.Sequence("CountertopSpraySequence", children=[
        go_to_center1,
        go_to_countertop,
        counter_align1,
        counter_left1,
        counter_align2,
        bot_attach_spray1,
        arm_start1,
        arm_up1,
        detect_sink_left,
        countertop_spray1,
        bot_detach_spray1,
        counter_align3,
        counter_right1,
        counter_align4,
        bot_attach_spray2,
        detect_sink_right,
        countertop_spray2,
        # bot_detach_spray2
    ])

    # Countertop spray sequence end

    # Grab sponge sequence

    grab_sponge_sequence = py_trees.composites.Sequence("GrabSpongeSequence")
    go_to_sponge_up1 = SetPredefinedArmPose("sponge_grip_up", name="SpongeUp1")
    open_gripper_sponge1 = SetPredefinedGripperPose("sponge_open", name="SpongeGripperOpen1")
    go_to_sponge1 = SetPredefinedArmPose("sponge_grip", name="SpongePose1")
    detach_sponge_base = JointAttach(name="SpongeBaseDetach", link1="sponge", link2="base_footprint", attach=False)
    grip_sponge1 = SetPredefinedGripperPose("sponge_grip", name="SpongeGrip1")
    attach_sponge_gripper = JointAttach(name="SpongeGripperAttach", link1="sponge", link2="gripper_base_link", attach=True)
    go_to_sponge_up2 = SetPredefinedArmPose("sponge_grip_up", name="SpongeUp2")
    arm_start2 = SetPredefinedArmPose("start", name="ArmStart2")

    grab_sponge_sequence.add_children([
        # go_to_center3,
        go_to_sponge_up1,
        open_gripper_sponge1,
        go_to_sponge1,
        detach_sponge_base,
        grip_sponge1,
        attach_sponge_gripper,
        go_to_sponge_up2,
        # arm_start2,
    ])

    # Grab sponge sequence end

    # Place sponge back sequence

    place_sponge_back_sequence = py_trees.composites.Sequence("PlaceSpongeBackSequence")
    go_to_center3 = GoToGoal(name="Center3", isCenter=True)
    go_to_sponge_up3 = SetPredefinedArmPose("sponge_grip_up", name="SpongeUp3")
    open_gripper_sponge2 = SetPredefinedGripperPose("sponge_open", name="SpongeGripperOpen2")
    go_to_sponge2 = SetPredefinedArmPose("sponge_grip", name="SpongePose2")
    attach_sponge_base = JointAttach(name="SpongeBaseAttach", link1="sponge", link2="base_footprint", attach=True)
    grip_sponge2 = SetPredefinedGripperPose("sponge_grip", name="SpongeGrip2")
    detach_sponge_gripper = JointAttach(name="SpongeGripperDetach", link1="sponge", link2="gripper_base_link", attach=False)
    go_to_sponge_up4 = SetPredefinedArmPose("sponge_grip_up", name="SpongeUp4")
    arm_start2 = SetPredefinedArmPose("start", name="ArmStart2")

    place_sponge_back_sequence.add_children([
        go_to_center3,
        go_to_sponge_up3,
        go_to_sponge2,
        open_gripper_sponge2,
        detach_sponge_gripper,
        attach_sponge_base,
        go_to_sponge_up4,
        arm_start2,
    ])

    # Place sponge back sequence end

    # Countertop wiping sequence

    go_to_center2 = GoToGoal(name="Center2", isCenter=True)
    go_to_countertop2 = GoToGoal(name='CountertopGoal2', isCounter=True) 
    counter_align5 = CounterAlign(name="Align5")
    counter_align6 = CounterAlign(name="Align6")
    bot_attach_wipe1 = BotAttach(attach=True, name="BotAttachWipe1")
    bot_attach_wipe2 = BotAttach(attach=True, name="BotAttachWipe2")
    bot_detach_wipe1 = BotAttach(attach=False, name="BotDetachWipe1")
    bot_detach_wipe2 = BotAttach(attach=False, name="BotDetachWipe2")
    counter_align7 = CounterAlign(name="Align7")
    counter_align8 = CounterAlign(name="Align8")
    counter_right2 = GoRight(name="CounterRight2")
    counter_left2 = GoLeft(name="CounterLeft2")
    countertop_wipe1 = CountertopWipe("Wipe1", side="Right")
    countertop_wipe2 = CountertopWipe("Wipe2", side="Left")
    arm_up_wipe1 = SetPredefinedArmPose("up", name="ArmUpWipe1")
    arm_start_wipe1 = SetPredefinedArmPose("start", name="ArmStartWipe1")

    countertop_wiping_sequence = py_trees.composites.Sequence("CountertopWipeSequence", children=[
        # go_to_center2,
        # go_to_countertop2,
        # counter_align5,
        # counter_right2,
        # bot_attach_wipe1,
        # arm_start_wipe1,
        arm_up_wipe1,
        countertop_wipe1,
        bot_detach_wipe1,
        counter_align7,
        counter_left2,
        counter_align8,
        bot_attach_wipe2,
        countertop_wipe2,
        bot_detach_wipe2
    ])

    # Countertop spray sequence end

    # Marking cleaning sequence

    go_to_door_marking = GoToGoal(name="MarkingDoor", isDoor=True)
    arm_start_after_marking = SetPredefinedArmPose("start", name="ArmStartAfterMarking")
    clean_marking = CleanMarking(name="CleanMarking1")

    # Marking cleaning sequence end

    # Go to start
    go_to_door2 = GoToGoal(name="GoToDoor2", isDoor=True)
    exit_door = ExitDoor(name="ExitDoor")
    go_to_start = GoToGoal(name="GoToStart", isStart=True)
    # Go to start end

    reIn = ReName(name="ReName")

    # Main sequence
    main_sequence = py_trees.idioms.pick_up_where_you_left_off(
        name="MainSequence",
        tasks=[
            parllel_init,
            reach_door_front,
            get_basic_poses,
            trash_detect_pose_entrance,
            entrance_sequence,
            end_entrance_fusion,
            # detect_counter_pose1,
            # detect_counter_pose_up,
            # parallel_counter_detect,
            # detect_counter_pose2,
            trash_detect_sequence,
            clear_trash,
            reIn,
            dustbin_false,
            go_to_center_dustbin,
            go_to_dustbin_detect_pose,
            dustbin_selector,
            go_to_dustbin,
            trash_drop_oneshot,
            # countertop_spray_sequence,
            # grab_sponge_sequence,
            # countertop_wiping_sequence,
            # place_sponge_back_sequence,
            go_to_door_marking,
            arm_start_after_marking,
            clean_marking,
            go_to_door2,
            exit_door,
            go_to_start,
            ]
    )

    root = main_sequence

    return root


if __name__ == '__main__':

    # Declare global variables
    global counter_detected, sink_detected
    global dustbin_bb, apbot_arm, apbot_gripper, client, ct, cf, df, duf, tp, mp, table_top, sink, wall_align, bb, markings_bb, door_bb
    global plume, pose_y, pose_x

    # Initialize global variables
    counter_detected = False
    sink_detected = False
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Initialize ros node
    rospy.init_node('artpark_node', anonymous=True)

    t = tf.TransformListener()

    # Start py trees
    args = command_line_argument_parser().parse_args()
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    tree = create_tree()
    print(description())

    py_trees.display.render_dot_tree(tree)

    py_trees.blackboard.Blackboard.enable_activity_stream(100)

    behaviour_tree = py_trees.trees.BehaviourTree(tree)
    behaviour_tree.add_pre_tick_handler(pre_tick_handler)
    behaviour_tree.visitors.append(py_trees.visitors.DebugVisitor())
    behaviour_tree.visitors.append(
        py_trees.visitors.DisplaySnapshotVisitor(
            display_blackboard=True,
            display_activity_stream=True)
    )
    behaviour_tree.setup(timeout=15)
    while True:
        try:
            behaviour_tree.tick()
            time.sleep(0.5)
        except KeyboardInterrupt:
            break
    print("\n")