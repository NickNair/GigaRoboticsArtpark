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
from sensor_msgs.msg import LaserScan, Image
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry 
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Quaternion,PoseStamped, Twist
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from gazebo_msgs.msg import LinkStates, ModelState
from gazebo_msgs.srv import DeleteModel, SetModelState
from fusion_class import fusion
from name import BoundingBoxes
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

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

class PickPipeline:

    def __init__(self):
        self.picked_objects=[]
        self.cleaned_markings=[]
        self.sides = ["left", "", "right"]
        self.entrance = 0
        self.ids =[]
        self.trash_done = []
        self.markings_done = []
        self.trash_poses_got = False
        self.trash_poses2 = []
        rospy.Subscriber('/odom',Odometry,self.get_pose,queue_size=10)
        rospy.Subscriber('/id_array', Int64MultiArray, self.get_ids, queue_size=10)
        rospy.Subscriber('/id_array_', Int64MultiArray, self.get_ids_, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.get_laser_data, queue_size=10)
        rospy.Subscriber("/camera/depth/image_rect_raw", Image,self.depth_image_callback)
        self.states_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.trash_states_callback)
        self.markings_states_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.marking_states_callback)
        self.check_pub = rospy.Publisher("/check", String, queue_size=10)
        self.trash_poses = []
        self.marking_poses = []
        self.can_fusion = fusion()
        self.can_fusion.listener_2()
        self.trash_name = BoundingBoxes(trash_marking=False, trash_marking_pick=True)
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                    Attach)
        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach',
                                    Attach)
        self.set_model_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.attach_srv.wait_for_service()

    # Get distance between 2 points
    def get_dist(self,x1,y1,x2,y2):
        return(math.sqrt((x1-x2) ** 2 + (y1-y2) ** 2))

    def trash_states_callback(self, data):
        for i in range(10):
            try:
                ind = data.name.index("trash"+str(i+1)+"::link")
                self.trash_poses.append([i+1, data.pose[ind]])
            except ValueError:
                pass

    def marking_states_callback(self, data):
        for i in range(5):
            try:
                ind = data.name.index("marking"+str(i+1)+"::link")
                self.marking_poses.append([i+1, data.pose[ind]])
            except ValueError:
                pass

    def set_trash_state(self):
        state = ModelState()
        rospy.wait_for_service('/gazebo/set_model_state')
        for i in range(5):
            state.model_name = "trash"+str(i)
            state.reference_frame = 'map'

            state.twist.linear.x = 0
            state.twist.linear.y = 0
            state.twist.linear.z = 0
            state.twist.angular.x = 0
            state.twist.angular.y = 0
            state.twist.angular.z = 0

            try:
                result = self.set_model_state_srv(state)
            except rospy.ServiceException:
                print("/gazebo/get_model_state service call failed")

    def trash_poses_callback(self, data):
        for i in range(10):
            try:
                ind = data.name.index("trash"+str(i+1)+"::link")
                self.trash_poses2.append([i+1, data.pose[ind]])
            except ValueError:
                pass
        self.trash_poses_got = True
        self.trash_poses_sub.unregister()

    def trash_tray_joints(self, attach = True):
        # while not self.trash_poses_got:
        #     self.trash_poses_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.trash_poses_callback)
        # for i in self.trash_done:
        #     if attach:
        #         for j in self.trash_poses2:
        #             if j[0]==i:
        #                 d = self.get_dist(j[1].position.x, j[1].position.y, self.curr_x, self.curr_y)
        #                 if d < .3:
        #                     trash_req = AttachRequest()
        #                     trash_req.model_name_2 = "apbot"
        #                     trash_req.link_name_2 = "trash_tray_1"
        #                     trash_req.model_name_1 = "trash"+str(i)
        #                     trash_req.link_name_1 = "link"
        #                     self.attach_srv.call(trash_req)
        #     else:
        #         self.detach_srv.call(trash_req)
        for i in self.trash_done:
            trash_req = AttachRequest()
            trash_req.model_name_2 = "apbot"
            trash_req.link_name_2 = "trash_tray_1"
            trash_req.model_name_1 = "trash"+str(i)
            trash_req.link_name_1 = "link"
            if attach:
                self.attach_srv.call(trash_req)
            else:
                self.detach_srv.call(trash_req)

    def find_trash_num(self, p):
        min_d = [0,1000]
        for i in self.trash_poses:
            if i[0] not in self.trash_done:
                d = self.get_dist(i[1].position.x, i[1].position.y, p.pose.position.x, p.pose.position.y)
                if d<min_d[1]:
                    min_d = [i[0], d]
        if min_d[1] < .2:
            self.trash_done.append(min_d[0])
            return min_d[0]
        else:
            return False

    def delete_marking(self, curr_x, curr_y):
        delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        for i in self.marking_poses:
            if i[0] not in self.markings_done:
                d = self.get_dist(i[1].position.x, i[1].position.y, curr_x, curr_y)
                if d < .225:
                    self.markings_done.append(i[0])   
                    delete_model("marking"+str(i[0]))     

    def depth_image_callback(self, data):
        self.image_data = data

    def get_laser_data(self, data):
        # [left, left_front, front, right_front, right]
        self.regions = [
        min(min(data.ranges[360:365]),10),
        # min(min(data.ranges[144:287]),10),
        min(min(data.ranges[175:185]),10),
        # min(min(data.ranges[432:575]),10),
        min(min(data.ranges[0:5]),10),
        ]
        self.laser_data = data.ranges
    
    def get_pose(self,data):

        self.curr_x = data.pose.pose.position.x
        self.curr_y = data.pose.pose.position.y
        quaternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.curr_z = euler[2]
    
    def get_ids(self,data):
        self.ids = data.data 

    def get_ids_(self,data):
        self.ids_ = data.data 

    def go_to_object(self, mani, gripper):
        ids = self.ids
        curr_x,curr_y,curr_z= self.curr_x,self.curr_y,self.curr_z
        entrance = self.entrance
        try:
            # print("Start go to object")

            poses =[]
            listener = tf.TransformListener()
            # print("Start reading the tfs")
            # print(ids)
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
                if pose_0.pose.position.z < .2:
                    poses.append([pose_0,dist,i])

            poses = sorted(poses,key = lambda x: x[1]) 

            #print(poses)
            is_entrance_free = True
            counter = 0 
            while(len(poses)):             
                poses = sorted(poses,key = lambda x: x[1])
                # print([i[2] for i in poses])
                pose =poses[0][0]
                p = pose
                t = [p.pose.position.x,p.pose.position.y,p.pose.position.z]
                q = [p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w]

                polygon = Polygon([(0, .3), (0, -.3), (1, -.3), (1, .3)])

                while True:
                    try :
                        p_wrt_odom = listener.transformPose("/base_footprint", pose_0)
                        break
                    except:
                        continue

                can_point = Point(p_wrt_odom.pose.position.x, p_wrt_odom.pose.position.y)

                if not polygon.contains(can_point) and entrance==1:
                    print("NOT IN BOX")
                    continue

                is_entrance_free = False

                angle_diff = math.atan2((-t[1]+curr_y), (-t[0]+curr_x))

                angle = curr_z + angle_diff

                if angle > math.pi:
                    angle -= 2*math.pi
                elif angle < -math.pi:
                    angle += 2*math.pi
                
                quats=quaternion_from_euler(0,0,angle) 

                result = movebase_client(t[0] - (.4)*math.cos(angle), t[1] - (.4)*math.sin(angle), quats)

                if result:
                    bot_req = AttachRequest()
                    bot_req.model_name_1 = "apbot"
                    bot_req.link_name_1 = "base_footprint"
                    bot_req.model_name_2 = "artpark_world"
                    bot_req.link_name_2 = "wall_front_1"

                    self.attach_srv.call(bot_req)

                    mani.go_to_predefined_pose("trash_detect_pose_up_front")

                    trash_list = []

                    self.trash_name.listener(True)

                    rospy.sleep(3)

                    t0 = rospy.get_time()
                    while not (len(trash_list) or rospy.get_time() - t0 > 5):
                        trash_list = self.can_fusion.get_object_pose()

                    self.trash_name.listener(False)

                    max_side = self.regions.index(max(self.regions))

                    if self.regions[1] > 1.7:
                        max_side = 1

                    side = self.sides[max_side]                   

                    for i in trash_list:
                        print("x:", i[0].pose.position.x)
                        print("y:", i[0].pose.position.y)
                        if i[0].pose.position.x > .65 or i[0].pose.position.y > .4 or i[0].pose.position.x < .25:
                            continue

                        if side=="":
                            mani.go_to_predefined_pose("start")
                            mani.go_to_predefined_pose("trash_pick")
                        else:
                            mani.go_to_predefined_pose(side)
                            mani.go_to_predefined_pose("trash_pick_"+ side)      
                            mani.go_to_predefined_pose("trash_pick") 

                        while True:
                            try:
                                pose_in_map = listener.transformPose("/map", i[0])
                                break
                            except:
                                continue
                        mani_can_top = geometry_msgs.msg.Pose()
                        mani_can_top.position.x = i[0].pose.position.x 
                        mani_can_top.position.y = i[0].pose.position.y
                        mani_can_top.position.z = i[0].pose.position.z + .15

                        quat_angle_can = quaternion_from_euler(-1.57, 0, i[1])

                        mani_can_top.orientation.x = quat_angle_can[0]
                        mani_can_top.orientation.y = quat_angle_can[1]
                        mani_can_top.orientation.z = quat_angle_can[2]
                        mani_can_top.orientation.w = quat_angle_can[3]

                        mani.go_to_pose(mani_can_top)

                        gripper.go_to_predefined_pose("open_full")

                        mani_can_top.position.z = i[0].pose.position.z + .05

                        mani.go_to_pose(mani_can_top)

                        trash_num = self.find_trash_num(pose_in_map)

                        req = AttachRequest()

                        if trash_num:
                            req.model_name_1 = "apbot"
                            req.link_name_1 = "gripper_base_link"
                            req.model_name_2 = "trash" + str(trash_num)
                            req.link_name_2 = "link"

                            self.attach_srv.call(req)

                        # gripper.go_to_predefined_pose("can_grip")
                        # rospy.sleep(2)

                        if side=="":
                            mani.go_to_predefined_pose("trash_pick")
                            mani.go_to_predefined_pose("start")
                            mani.go_to_predefined_pose("drop_trash6")  
                        else:
                            mani.go_to_predefined_pose("trash_pick")
                            mani.go_to_predefined_pose("trash_pick_"+ side) 
                            mani.go_to_predefined_pose(side)
                            mani.go_to_predefined_pose("drop_trash_"+side) 
                            if side=="right":
                                mani.go_to_predefined_pose("drop_trash_right_back")               

                        mani.go_to_predefined_pose("drop_trash")

                        gripper.go_to_predefined_pose("open_full")

                        if trash_num:
                            self.detach_srv(req)

                        if side=="":
                            mani.go_to_predefined_pose("drop_trash6")
                            mani.go_to_predefined_pose("start")
                        else:
                            if side=="right":
                                mani.go_to_predefined_pose("drop_trash_right_back")
                            mani.go_to_predefined_pose("drop_trash_"+side)
                            mani.go_to_predefined_pose(side)
                            mani.go_to_predefined_pose("start")

                    self.detach_srv(bot_req)
                    
                    self.picked_objects.append(poses[0][2])
                    poses.pop(0)
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

            if entrance==1 and is_entrance_free == True:
                vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
                vel = Twist()
                vel.linear.x = .3

                vel_pub.publish(vel)

                rospy.sleep(2.5)

                vel.linear.x = 0

                vel_pub.publish(vel)

            # empty_arr = Int64MultiArray()
            # empty_arr.data = []
            # empty_arr_pub = rospy.Publisher('/id_array', Int64MultiArray, queue_size=10)
            # empty_arr_pub.publish(empty_arr)

        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation test finished.")

    def go_to_markings(self, mani):
        ids = self.ids_
        print(ids)
        curr_x,curr_y,curr_z= self.curr_x,self.curr_y,self.curr_z
        try:
            # print("Start go to object")

            poses =[]
            listener = tf.TransformListener()
            # print("Start reading the tfs")
            # print(ids)
            for i in ids:
                while True:
                    try :
                        t,q =listener.lookupTransform("/map","object_m_"+str(i), rospy.Time(0))
                        t2,_ =listener.lookupTransform("/base_footprint","object_m_"+str(i), rospy.Time(0))
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

            print("enters4")

            #print(poses)
            is_entrance_free = True
            counter = 0 
            while(len(poses)):
                print("enters5")                
                poses = sorted(poses,key = lambda x: x[1])
                # print([i[2] for i in poses])
                pose =poses[0][0]
                p = pose
                t = [p.pose.position.x,p.pose.position.y,p.pose.position.z]
                q = [p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w]

                polygon = Polygon([(0, .3), (0, -.3), (1, -.3), (1, .3)])

                while True:
                    try :
                        p_wrt_odom = listener.transformPose("/base_footprint", pose_0)
                        break
                    except:
                        continue

                can_point = Point(p_wrt_odom.pose.position.x, p_wrt_odom.pose.position.y)

                angle_diff = math.atan2((-t[1]+curr_y), (-t[0]+curr_x))

                angle = curr_z + angle_diff

                if angle > math.pi:
                    angle -= 2*math.pi
                elif angle < -math.pi:
                    angle += 2*math.pi
                
                quats=quaternion_from_euler(0,0,angle) 

                result = movebase_client(t[0] - (.4)*math.cos(angle), t[1] - (.4)*math.sin(angle), quats)

                if result:
                    # mani.go_to_predefined_pose("trash_detect_pose_up_front")

                    # marking_list = []

                    # self.trash_name.listener(True)

                    rospy.sleep(2)

                    vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
                    vel = Twist()
                    vel.linear.x = .1
                    vel_pub.publish(vel)
                    rospy.sleep(4)
                    vel.linear.x = 0
                    vel_pub.publish(vel)
                    rospy.sleep(1)
                    self.delete_marking(self.curr_x, self.curr_y)
                    vel.linear.x = -.1
                    vel_pub.publish(vel)
                    rospy.sleep(4)
                    vel.linear.x = 0
                    vel_pub.publish(vel)
                    rospy.sleep(2)

                    # while not len(marking_list):
                    #     marking_list = self.can_fusion.get_object_pose()

                    # while True:
                    #     try :
                    #         p = tes.transformPose("/base_footprint", pose)
                    #         break
                    #     except:
                    #         continue
                    # t = [p.pose.position.x,p.pose.position.y,p.pose.position.z]
                    # q = [p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w]

                    # self.trash_name.listener(False)               

                    # for i in marking_list:
                    #     while True:
                    #         try:
                    #             pose_in_map = listener.transformPose("/map", i[0])
                    #             break
                    #         except:
                    #             continue
                    #     trash_num = self.find_trash_num(pose_in_map)

                    self.cleaned_markings.append(poses[0][2])
                    poses.pop(0)
                    cnt=0
                    for x_,y_,i in poses:
                        while True:
                            try :
                                t2,_ =listener.lookupTransform("/base_footprint","object_m_"+str(i), rospy.Time(0))
                                dist = ( t2[0]**2 + t2[1]**2 )**0.5
                                poses[cnt][1]=dist
                                cnt+=1
                                break
                            except:
                                continue

        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation test finished.")


if __name__ == "__main__":

    # mani = Ur5Moveit("arm")
    #gripper = Ur5Moveit("gripper")
    rospy.init_node("pick_pipeline_node")
    p=PickPipeline()
    markings_states_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, p.marking_states_callback)
    # rospy.Subscriber('/id_array', Int64MultiArray, get_ids, queue_size=10)
    # rospy.Subscriber('/odom',Odometry,get_pose,queue_size=10)
    rospy.spin()

    

    