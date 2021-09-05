#!/bin/env python3

import rospy
import actionlib
import py_trees
import argparse
import sys
import time
import py_trees.console as console

from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry 
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion 
from py_trees_test.msg import battery_status

global battery_level

def description():
    content = "Py Trees demo with a simple navigation example in ROS 1\n"
    content += "An example where the bot moves to 3 different specified on the map\n"
    content += "while keeping in check its current battery level.\n"
    content += "\n"
    content += "EVENTS\n"
    content += "\n"
    content += " -  1 : Go to goal A\n"
    content += " -  2 : Go to goal B\n"
    content += " -  3 : Go to goal C\n"
    content += "\n"
    if py_trees.console.has_colours:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = "\n"
        s += banner_line
        s += console.bold_white + "Move Base - PyTrees".center(79) + "\n" + console.reset
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

class GoToGoal(py_trees.behaviour.Behaviour):
    def __init__(self, goal, name, task_status="Middle"):
        super().__init__(name=name)        
        self.goal = goal
        self.task_status = task_status
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("current_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("task_status", access=py_trees.common.Access.WRITE)
    
    def setup(self):
        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()
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
    
    def initialise(self):
        if self.task_status=="Started":
            self.blackboard.task_status = "Started"
        # Sends the goal to the action server
        self.client.send_goal(self.mb_goal)
    
    def update(self):
        print(console.green + "---------------------------")

        state = self.client.get_state()

        if state==3:
            new_status = py_trees.common.Status.SUCCESS
        else:
            new_status = py_trees.common.Status.RUNNING
        
        return new_status
    
    def terminate(self, new_status):
        if self.task_status == "End":
            self.blackboard.task_status = "End"
        if emergency_status:
            print(console.red + "WARNING: All goals cancelled!!")
            self.client.cancel_all_goals()
        else:
            pass

class BatteryCheck(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name="BatteryCheck")
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("battery_level", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("docking_status", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("docking_status", access=py_trees.common.Access.READ)
        self.blackboard.register_key("task_status", access=py_trees.common.Access.READ)
    
    def update(self):
        if battery_level<30:
            if self.blackboard.task_status == "End":
                if self.blackboard.docking_status != "Docked Successfully!":
                    print(console.red + "WARNING: LOW BATTERY")
                    self.blackboard.docking_status = "DOCK NOW!!"
                return py_trees.common.Status.FAILURE
            else:
                print(console.red + "WARNING: LOW BATTERY")
                self.blackboard.docking_status = "Dock after task is completed"
                return py_trees.common.Status.SUCCESS
        else:
            self.blackboard.docking_status = "Not required"
            return py_trees.common.Status.SUCCESS

class EmergencyCheck(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name="EmergencyCheck")
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("emergency_status", access=py_trees.common.Access.WRITE)
    
    def update(self):
        if emergency_status:
            print(console.red + "WARNING: EMERGENCY!!")
            return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.SUCCESS

class DetectObject(py_trees.behaviour.Behaviour):
    def __init__(self, object_id):
        super().__init__(name="DetectObject")
        self.object_id = object_id
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("object_to_detect", access=py_trees.common.Access.WRITE)

    def update(self):
        print(console.yellow + "Trying to detect object" + str(self.object_id))

        rospy.sleep(2)

        return py_trees.common.Status.SUCCESS

class ConfirmDetection(py_trees.behaviour.Behaviour):
    def __init__(self, object_id):
        super().__init__(name="ConfirmDetection")
        self.object_id = object_id
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("object_detected", access=py_trees.common.Access.WRITE)

    def update(self):
        has_detected = input(console.yellow + "Has object " + self.object_id + "been detected? (y/n)")

        if has_detected.lower()=="y":
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class PickObject(py_trees.behaviour.Behaviour):
    def __init__(self, object_id):
        super().__init__(name="PickObject")
        self.object_id = object_id
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("picked_object", access=py_trees.common.Access.WRITE)

    def update(self):
        print(console.yellow + "Trying to pick" + str(self.object_id))

        rospy.sleep(2)

        return py_trees.common.Status.SUCCESS

class ConfirmPicking(py_trees.behaviour.Behaviour):
    def __init__(self, object_id):
        super().__init__(name="ConfirmPicking")
        self.object_id = object_id
        self.blackboard = self.attach_blackboard_client(name=self.name)

    def update(self):
        has_picked = input(console.yellow + "Has object " + self.object_id + "been picked? (y/n)")

        if has_picked.lower()=="y":
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class DropObject(py_trees.behaviour.Behaviour):
    def __init__(self, object_id):
        super().__init__(name="DropObject")
        self.object_id = object_id
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("dropped_object", access=py_trees.common.Access.WRITE)

    def update(self):
        print(console.yellow + "Trying to drop" + str(self.object_id))
        rospy.sleep(2)
        return py_trees.common.Status.SUCCESS

class ConfirmDrop(py_trees.behaviour.Behaviour):
    def __init__(self, object_id):
        super().__init__(name="ConfirmDrop")
        self.object_id = object_id
        self.blackboard = self.attach_blackboard_client(name=self.name)

    def update(self):
        has_dropped = input(console.yellow + "Has object " + self.object_id + "been dropped? (y/n)")

        if has_dropped.lower()=="y":
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class Docking(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name="Docking")
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("docking_status", access=py_trees.common.Access.READ)
        self.blackboard.register_key("docking_status", access=py_trees.common.Access.WRITE)

    def update(self):
        print("\n"  +console.green + "---------------------------")


        if self.blackboard.docking_status == "DOCK NOW!!":
            has_docked = input(console.yellow + "Has the bot docked successfully? (y/n)\n")
            if has_docked.lower() == 'y':
                self.blackboard.docking_status = "Docked Successfully!"
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.SUCCESS

# Battery status callback
def battery_callback(data):
    global battery_level
    
    battery_level = data.battery_level

# Emergency status callback
def emergency_callback(data):
    global emergency_status
    
    if data.data == "No":
        emergency_status = False
    else:
        emergency_status = True

def create_tree():
    goalA = GoToGoal([5,0,0], "GOAL-A", "Started")
    goalB = GoToGoal([10,0,0], "GOAL-B", "End")

    docking_sequence = py_trees.composites.Sequence("DockingSequence")

    go_to_dock = GoToGoal([0,0,0], "Dock", "End")
    docking = Docking()

    docking_sequence.add_children([go_to_dock, docking])

    detect_sequence = py_trees.composites.Sequence("DetectSequence")

    detect_object = DetectObject("Obj-1")
    object_detected = ConfirmDetection("Obj-1")

    detect_sequence.add_child(detect_object)
    detect_sequence.add_child(object_detected)

    picking_sequence = py_trees.composites.Sequence("PickingSequence")

    pick_object = PickObject("Obj-1")
    object_picked = ConfirmPicking("Obj-1")

    picking_sequence.add_child(pick_object)
    picking_sequence.add_child(object_picked)

    drop_sequence = py_trees.composites.Sequence("DropSequence")

    drop_object = DropObject("Obj-1")
    object_dropped = ConfirmDrop("Obj-1")

    drop_sequence.add_child(drop_object)
    drop_sequence.add_child(object_dropped)

    sequence = py_trees.idioms.pick_up_where_you_left_off(
        name="Sequence",
        tasks=[goalA,
        detect_sequence,
        picking_sequence,
        goalB,
        drop_sequence
        ]
    )

    battery_check = BatteryCheck()
    emergency_check = EmergencyCheck()

    battery_guard = py_trees.idioms.eternal_guard(
        subtree=sequence,
        conditions=[
            battery_check
        ],
    )
    docking_selector = py_trees.composites.Selector("DockingSelector")

    docking_selector.add_children([battery_guard, docking_sequence])
    emergency_guard = py_trees.idioms.eternal_guard(
        subtree=docking_selector,
        conditions=[
           emergency_check 
        ],
    )
    root = emergency_guard
    return root


if __name__ == '__main__':

    battery_level = 100
    emergency_status = False

    rospy.init_node('py_trees_move_base', anonymous=True)

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
            rospy.Subscriber('/battery_status',battery_status, battery_callback)
            rospy.Subscriber('/emergency_status',String, emergency_callback)
            behaviour_tree.tick()
            time.sleep(0.5)
        except KeyboardInterrupt:
            break
    print("\n")

