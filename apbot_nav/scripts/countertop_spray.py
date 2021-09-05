#!/usr/bin/env python3 

import rospy
import actionlib
import subprocess
import math
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import tf
import sensor_msgs.point_cloud2 as pc2

from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import JointConstraint, Constraints
from nav_msgs.msg import Odometry 
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import PointCloud2, PointField
from water_rviz_marker import MarkerSpawner

class Ur5Moveit:
    
    # Constructor
    def __init__(self,x):
        
        # Initialize Node
        rospy.init_node('pickndplace', anonymous=True)
        rospy.sleep(1.5)

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

        # Initializing Tf listener object
        self.t = tf.TransformListener()

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
            self._group.set_named_target(arg_pose_name)
            plan = self._group.go()
        except:
            pass

    def cartesian_path(self, waypoints):
        (plan, fraction) = self._group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.0005,        # eef_step
                                        0.0)         # jump_threshold
        self._group.execute(plan, wait=True)

    def cartesian_path2(self, waypoints):
        (plan, fraction) = self._group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.001,        # eef_step
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

    def init_spray_constraints(self):
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

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        # rospy.loginfo(
        #     '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')
    
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

class Countertop():
    def __init__(self, pg):
        self.pg = pg
        self.plume = MarkerSpawner()

    def sink_clean(self, sink_xy, sink_height, top_dims, bottom_dims):
        self.plume.isDelete = True
        sink_x = sink_xy[0]
        sink_y = sink_xy[1]

        waypoints = []
        counter_pose = geometry_msgs.msg.Pose()
        counter_pose.position.x = sink_x - bottom_dims[1]/2 + .02
        counter_pose.position.y = sink_y - bottom_dims[0]/2 
        counter_pose.position.z = sink_height + .15
        qaut_angle = quaternion_from_euler(-1.57, 0, 3.14)
        counter_pose.orientation.x = qaut_angle[0]
        counter_pose.orientation.y = qaut_angle[1]
        counter_pose.orientation.z = qaut_angle[2]
        counter_pose.orientation.w = qaut_angle[3]

        self.pg.init_wipe_constraints()

        self.pg.go_to_predefined_pose("before_wipe_pose")

        self.pg.go_to_pose(counter_pose)

        i = 0
        while i < bottom_dims[1]//.12:
            if i%2!=0:
                counter_pose.position.y = sink_y - bottom_dims[0]/2 
            # counter_pose.position.x += .45*(1 - 2*(i%2))
            # elif abs(sink_y - counter_pose.position.y) < .1:
            #     counter_pose.position.x += .15
            else:
                counter_pose.position.y += bottom_dims[0] 
            qaut_angle = quaternion_from_euler(-1.57, 0, 3.14)
            counter_pose.orientation.x = qaut_angle[0]
            counter_pose.orientation.y = qaut_angle[1]
            counter_pose.orientation.z = qaut_angle[2]
            counter_pose.orientation.w = qaut_angle[3]
            waypoints.append(copy.deepcopy(counter_pose))

            # if i%2!=0 and abs(sink_y - counter_pose.position.y) < .1:
            #     counter_pose.position.x -= .2
            #     waypoints.append(copy.deepcopy(counter_pose))
            
            if i!=bottom_dims[1]//.12 - 1:
                counter_pose.position.x += .12
                waypoints.append(copy.deepcopy(counter_pose))
            else:
                counter_pose.position.x += .05 
                waypoints.append(copy.deepcopy(counter_pose))
                counter_pose.position.y += bottom_dims[0] 
                waypoints.append(copy.deepcopy(counter_pose))
            i += 1

        self.pg.cartesian_path(waypoints)

        self.pg._group.clear_path_constraints()

        self.pg.go_to_predefined_pose("up")

    def sink_side_clean_right(self, sink_xy, dimensions, top_dims):
        self.plume.isDelete = True
        sink_x = sink_xy[0]
        sink_y = sink_xy[1]

        waypoints = []
        counter_pose = geometry_msgs.msg.Pose()
        counter_pose.position.x = .3
        counter_pose.position.y = -(sink_y + top_dims[0]/2 + .05)
        counter_pose.position.z = dimensions[2] + .15
        qaut_angle = quaternion_from_euler(-1.57, 0, 1.57)
        counter_pose.orientation.x = qaut_angle[0]
        counter_pose.orientation.y = qaut_angle[1]
        counter_pose.orientation.z = qaut_angle[2]
        counter_pose.orientation.w = qaut_angle[3]

        # self.pg.init_stay_up_constraints()

        self.pg.init_wipe_constraints()

        self.pg.go_to_pose(counter_pose)
        rospy.sleep(.2)

        i = 0
        while i < 1:
            if i%2!=0:
                counter_pose.position.x = .35
            # counter_pose.position.x += .45*(1 - 2*(i%2))
            # elif abs(sink_y - counter_pose.position.y) < .1:
            #     counter_pose.position.x += .15
            else:
                counter_pose.position.x += dimensions[1] - .1
            qaut_angle = quaternion_from_euler(-1.57, 0, 1.57)
            counter_pose.orientation.x = qaut_angle[0]
            counter_pose.orientation.y = qaut_angle[1]
            counter_pose.orientation.z = qaut_angle[2]
            counter_pose.orientation.w = qaut_angle[3]
            waypoints.append(copy.deepcopy(counter_pose))

            qaut_angle = quaternion_from_euler(-1.57, 0, 3.14)
            counter_pose.orientation.x = qaut_angle[0]
            counter_pose.orientation.y = qaut_angle[1]
            counter_pose.orientation.z = qaut_angle[2]
            counter_pose.orientation.w = qaut_angle[3]

            waypoints.append(copy.deepcopy(counter_pose))

            # if i%2!=0 and abs(sink_y - counter_pose.position.y) < .1:
            #     counter_pose.position.x -= .2
            #     waypoints.append(copy.deepcopy(counter_pose))
            
            if i!=2:
                counter_pose.position.y = sink_y - .1 
                waypoints.append(copy.deepcopy(counter_pose))
            i += 1

        self.pg.cartesian_path(waypoints)

        self.pg._group.clear_path_constraints()

        self.pg.go_to_predefined_pose("before_wipe_pose")

    def spray_left(self,  dimensions, sink_xy=[0,0]):
        self.plume.isDelete = True
        sink_x = sink_xy[0]
        sink_y = sink_xy[1]

        waypoints = []
        counter_pose = geometry_msgs.msg.Pose()
        counter_pose.position.x = .3 
        counter_pose.position.y = 0
        counter_pose.position.z = dimensions[2]+.15
        qaut_angle = quaternion_from_euler(-1.57, 0, 1.57)
        counter_pose.orientation.x = qaut_angle[0]
        counter_pose.orientation.y = qaut_angle[1]
        counter_pose.orientation.z = qaut_angle[2]
        counter_pose.orientation.w = qaut_angle[3]

        # self.pg.init_stay_up_constraints()

        self.pg.init_wipe_constraints()

        self.pg.go_to_pose(counter_pose)
        rospy.sleep(.2)

        i = 0
        while i < 3:
            if i%2!=0:
                counter_pose.position.x = .3
            # counter_pose.position.x += .45*(1 - 2*(i%2))
            # elif abs(sink_y - counter_pose.position.y) < .1:
            #     counter_pose.position.x += .3
            else:
                counter_pose.position.x += dimensions[1] - .10
                
            qaut_angle = quaternion_from_euler(-1.57, 0, 1.57)
            counter_pose.orientation.x = qaut_angle[0]
            counter_pose.orientation.y = qaut_angle[1]
            counter_pose.orientation.z = qaut_angle[2]
            counter_pose.orientation.w = qaut_angle[3]
            waypoints.append(copy.deepcopy(counter_pose))

            # if i%2!=0 and abs(sink_y - counter_pose.position.y) < .1:
            #     counter_pose.position.x -= .2
            #     waypoints.append(copy.deepcopy(counter_pose))

            if i!=2:
                counter_pose.position.y += .13
                waypoints.append(copy.deepcopy(counter_pose))
            i += 1

        self.pg.cartesian_path(waypoints)

        self.pg._group.clear_path_constraints()

        self.pg.go_to_predefined_pose("up")

        # status_pub.publish("Spraying Done")

    def spray_right(self, dimensions, sink_xy=[0,0]):
        self.plume.isDelete = True
        sink_x = sink_xy[0]
        sink_y = sink_xy[1]

        waypoints = []
        counter_pose = geometry_msgs.msg.Pose()
        counter_pose.position.x = .3 
        counter_pose.position.y = -0.2
        counter_pose.position.z = dimensions[2] + .15
        qaut_angle = quaternion_from_euler(-1.57, 0, 1.57)
        counter_pose.orientation.x = qaut_angle[0]
        counter_pose.orientation.y = qaut_angle[1]
        counter_pose.orientation.z = qaut_angle[2]
        counter_pose.orientation.w = qaut_angle[3]

        # self.pg.init_stay_up_constraints()

        self.pg.init_wipe_constraints()

        self.pg.go_to_predefined_pose("before_wipe_pose")

        self.pg.go_to_pose(counter_pose)
        rospy.sleep(.2)

        i = 0
        while i < 2:
            if i%2!=0:
                counter_pose.position.x = .3
            # counter_pose.position.x += .45*(1 - 2*(i%2))
            # elif abs(sink_y - counter_pose.position.y) < .1:
            #     counter_pose.position.x += .3
            else:
                counter_pose.position.x += dimensions[1] - .10
            qaut_angle = quaternion_from_euler(-1.57, 0, 1.57)
            counter_pose.orientation.x = qaut_angle[0]
            counter_pose.orientation.y = qaut_angle[1]
            counter_pose.orientation.z = qaut_angle[2]
            counter_pose.orientation.w = qaut_angle[3]
            waypoints.append(copy.deepcopy(counter_pose))

            # if i%2!=0 and abs(sink_y - counter_pose.position.y) < .1:
            #     counter_pose.position.x -= .2
            #     waypoints.append(copy.deepcopy(counter_pose))

            if i!=1:
                counter_pose.position.y += .13
                waypoints.append(copy.deepcopy(counter_pose))
            i += 1

        self.pg.cartesian_path(waypoints)

        self.pg._group.clear_path_constraints()

        # self.pg.go_to_predefined_pose("up")

        # status_pub.publish("Spraying Done")

    def wipe_left(self, sink_xy):
        pass

    def wipe_right(self, dimensions):

        waypoints = []
        counter_pose = geometry_msgs.msg.Pose()
        counter_pose.position.x = .35 + dimensions[1]/2 
        counter_pose.position.y = -dimensions[0]/4
        counter_pose.position.z = dimensions[2] + .4
        qaut_angle = quaternion_from_euler(-1.57, .5, 3.14)
        counter_pose.orientation.x = qaut_angle[0]
        counter_pose.orientation.y = qaut_angle[1]
        counter_pose.orientation.z = qaut_angle[2]
        counter_pose.orientation.w = qaut_angle[3]

        self.pg.go_to_predefined_pose("sink_start")
        rospy.sleep(.2)

        self.pg.init_spray_constraints()

        cmd = ["roslaunch","uuv_plume_simulator","start_plume_example.launch"]
        self.proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        rospy.sleep(3)

        self.plume.register_plume()

        i = 0
        while i < 6:
            qaut_angle = quaternion_from_euler(-1.57, -.1, 3.14)
            counter_pose.orientation.x = qaut_angle[0]
            counter_pose.orientation.y = qaut_angle[1]
            counter_pose.orientation.z = qaut_angle[2]
            counter_pose.orientation.w = qaut_angle[3]
            counter_pose.position.x -= .1
            waypoints.append(copy.deepcopy(counter_pose))

            if i != 5:
                qaut_angle = quaternion_from_euler(-1.57, 1, 3.14)
                counter_pose.orientation.x = qaut_angle[0]
                counter_pose.orientation.y = qaut_angle[1]
                counter_pose.orientation.z = qaut_angle[2]
                counter_pose.orientation.w = qaut_angle[3]
                counter_pose.position.x += .1
                counter_pose.position.y += .105
                waypoints.append(copy.deepcopy(counter_pose))
            i += 1

        self.plume.start_spray()
        rospy.sleep(1)

        self.pg.cartesian_path2(waypoints)

        self.plume.stop_spray()

        self.proc.terminate()
        rospy.sleep(1)

        self.plume.unregister_plume()

        self.pg._group.clear_path_constraints()

        self.pg.go_to_predefined_pose("up")

if __name__=="__main__":

    mani = Ur5Moveit("arm")
    status_pub = rospy.Publisher('/spray_status', String, queue_size=10)
    cs = Countertop()

    mani.go_to_predefined_pose("start")
    rospy.sleep(.2)

    mani.go_to_predefined_pose("up")
    rospy.sleep(.2)

    cs.spray_left(sink_xy=[.5,-.02])
    rospy.spin()
