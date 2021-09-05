#!/usr/bin/env python3

import rospy
import actionlib
import math
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import tf

from std_msgs.msg import String, Int64MultiArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry 
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Quaternion,PoseStamped
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

global id_arr

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

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    # Function to go to specified position
    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    # Function to set joint angles
    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    # Function to go to pre defined position
    def go_to_predefined_pose(self, arg_pose_name):
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.go()

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')
    
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

def get_dist(x1,y1,x2,y2):
    return(math.sqrt((x1-x2) ** 2 + (y1-y2) ** 2))

# Pose callback
def get_pose(data):

    curr_x = data.pose.pose.position.x
    curr_y = data.pose.pose.position.y
    quaternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    euler = euler_from_quaternion(quaternion)
    curr_z = euler[2]

    #heck_entrance(curr_x, curr_y, curr_z)

    go_to_object(curr_x, curr_y, curr_z)

def check_entrance(curr_x, curr_y, curr_z):

    go_to_object(curr_x, curr_y, curr_z)

def go_to_object(curr_x, curr_y, curr_z,ids, mani, entrance = 0):
    # global ids,mani
    print("going......")
    try:
        pub = rospy.Publisher("/check", String,  queue_size=1)
        flag_on = String()
        
        flag_on.data = "Detect"
        
        pub.publish(flag_on)
        print("Start go to object")

        poses =[]
        listener = tf.TransformListener()
        print("Start reading the tfs")
        print(ids)
        for i in ids:
            while True:
                try :
                    t,q =listener.lookupTransform("/map","object_"+str(i), rospy.Time(0))
                    t2,_ =listener.lookupTransform("/base_footprint","object_"+str(i), rospy.Time(0))
                    dist = ( t2[0]**2 + t2[1]**2 )**0.5
                    pose_0 = PoseStamped()
                    pose_0.header.frame_id= "map"
                    pose_0.pose.position.x = t[0]
                    pose_0.pose.position.y = t[1]
                    pose_0.pose.position.z = t[2]

                    pose_0.pose.orientation.x = q[0]
                    pose_0.pose.orientation.y = q[1]
                    pose_0.pose.orientation.z = q[2]
                    pose_0.pose.orientation.w = q[3]
                    break
                except:
                    continue
            poses.append([pose_0,dist,i])

        poses = sorted(poses,key = lambda x: x[1]) 

        #print(poses)
        counter = 0 
        for count in range(len(poses)):

            poses = sorted(poses,key = lambda x: x[1])
            print([i[2] for i in poses])
            pose =poses[count][0]
            p = pose
            t = [p.pose.position.x,p.pose.position.y,p.pose.position.z]
            q = [p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w]

            polygon = Polygon([(0, .3), (0, -.3), (1, -.3), (1, .3)])

            can_point = Point(t[0], t[1])

            if not polygon.contains(can_point) and entrance==1:
                continue

            angle_diff = math.atan2((-t[1]+curr_y), (-t[0]+curr_x))

            angle = curr_z + angle_diff

            if angle > math.pi:
                angle -= 2*math.pi
            elif angle < -math.pi:
                angle += 2*math.pi

            quat = quaternion_from_euler(0,0,angle)
            
            quats=quaternion_from_euler(0,0,angle) 
            result = movebase_client(t[0] - (.15)*math.cos(angle), t[1] - (.15)*math.sin(angle), quats)
            print(result)

            if result:
                # mani.go_to_predefined_pose("trash_detect")

                # rospy.sleep(2)

                # while True:
                #     try :
                #         p = tes.transformPose("/base_footprint", pose)
                #         break
                #     except:
                #         continue
                # t = [p.pose.position.x,p.pose.position.y,p.pose.position.z]
                # q = [p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w]

                # mani_can_top = geometry_msgs.msg.Pose()
                # mani_can_top.position.x = t[0]- .05
                # mani_can_top.position.y = t[1]
                # mani_can_top.position.z = t[2] + .1

                # quat_angle_can = quaternion_from_euler(-1.57, 0, 3.14)

                # mani_can_top.orientation.x = q[0]
                # mani_can_top.orientation.y = q[1]
                # mani_can_top.orientation.z = q[2]
                # mani_can_top.orientation.w = q[3]

                # mani.go_to_pose(mani_can_top)

                # rospy.sleep(2)
                cnt=0
                for x_,y_,i in poses:
                    while True:
                        try :
                            t2,_ =listener.lookupTransform("/base_footprint","object_"+str(i), rospy.Time(0))
                            dist = ( t2[0]**2 + t2[1]**2 )**0.5
                            poses[cnt][1]=dist
                            cnt+=1
                            break
                        except:
                            continue

        # grip_left.go_to_predefined_pose("open_gripper_left")
        # grip_right.go_to_predefined_pose("open_gripper_right")

        # mani.go_to_predefined_pose("trash_detect")

        # rospy.sleep(2)

        # if mani.t.frameExists('can1'):
        #     (translation_can, rotation_can) = mani.t.lookupTransform('/base_footprint', '/object_041', rospy.Time(0))

        # mani_can_top = geometry_msgs.msg.Pose()
        # mani_can_top.position.x = translation_can[0] - .05
        # mani_can_top.position.y = translation_can[1] 
        # mani_can_top.position.z = translation_can[2] + .1
        # quat_angle_can = quaternion_from_euler(-1.57, 0, 3.14)
        # mani_can_top.orientation.x = quat_angle_can[0]
        # mani_can_top.orientation.y = quat_angle_can[1]
        # mani_can_top.orientation.z = quat_angle_can[2]
        # mani_can_top.orientation.w = quat_angle_can[3]

        # mani.go_to_pose(mani_can_top)
        # rospy.sleep(2)

        # result = movebase_client(curr_x + translation_can[0] - .1, curr_y + translation_can[1] - .1, rotation_can)

        # if result:
        #     rospy.loginfo("Success!!")

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

def get_ids(data):
    global ids
    ids = data.data 


if __name__ == "__main__":

    mani = Ur5Moveit("arm")
    #gripper = Ur5Moveit("gripper")
    rospy.Subscriber('/id_array', Int64MultiArray, get_ids, queue_size=10)
    rospy.Subscriber('/odom',Odometry,get_pose,queue_size=10)
    rospy.spin()

    