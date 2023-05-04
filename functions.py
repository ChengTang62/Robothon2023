from __future__ import print_function
from six.moves import input
################################# edited

import sys
import time
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import move_constants as const
import subprocess
import franka_gripper.msg
import actionlib
import tf
import json
import math
import get_orien

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi


    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from controller_manager_msgs.srv import SwitchController
from controller_manager_msgs.srv import SwitchControllerRequest

from franka_msgs.srv import SetJointImpedance
from franka_msgs.srv import SetJointImpedanceRequest
from moveit_msgs.msg import MotionPlanRequest
from franka_gripper.msg import MoveAction, GraspAction, MoveGoal, GraspGoal


class RobotControl(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.plan = MotionPlanRequest()
        self.const = const
        self.math = math
        self.origin_x = 0.421
        self.origin_y = 0.002
        self.origin_z = 0.2153
        self.angle = 0

        group_name = "panda_arm"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        group_name_hand = "panda_hand"
        self.move_group_hand = moveit_commander.MoveGroupCommander(group_name_hand)

        self.display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20,
        )
        self.planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % self.planning_frame)

        # We can also print the name of the end-effector link for this group:
        self.eef_link = self.move_group.get_end_effector_link()
        #print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        self.group_names = self.robot.get_group_names()
        #print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        #print("============ Printing robot state")
        #print(robot.get_current_state())

        self.switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        self.set_joint_impedance = rospy.ServiceProxy('/franka_control/set_joint_impedance', SetJointImpedance)
        
        self.gripper_grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
        self.gripper_move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)

        self.go = get_orien.GetOrien()


        print("================== Initialization Complete")


    def recover(self):
        subprocess.call("rostopic pub -1 /franka_control/error_recovery/goal franka_msgs/ErrorRecoveryActionGoal \"{}\"",shell=True)
        print("================== Recovered from Error")

####################################################| GRIPPER |####################################################

    def grip(self):
        hand_goal = self.move_group_hand.get_current_joint_values()

        self.move_group_hand.set_max_velocity_scaling_factor(0.01)
        self.move_group_hand.set_max_acceleration_scaling_factor(0.01)
        
        hand_goal[0] = 0.000
        self.move_group_hand.go(hand_goal, wait=True)
        
        self.move_group_hand.stop()
        
        print("================== Gripper closed")
        

    def ungrip(self):
        
        goal = MoveGoal()
        goal.width = 0.1
        goal.speed = 0.1
        self.gripper_move_client.send_goal_and_wait(goal)
        self.gripper_move_client.get_result()

        print("================== Gripper opened")
        

    def midgrip(self,dist=0.02):
        
        self.move_group_hand.set_max_velocity_scaling_factor(0.01)
        self.move_group_hand.set_max_acceleration_scaling_factor(0.01)
    
        hand_goal = self.move_group_hand.get_current_joint_values()
        
        hand_goal[0] = dist
        self.move_group_hand.go(hand_goal, wait=True)
        
        self.move_group_hand.stop()

        print("================== Gripper opened")

####################################################| ORIGIN |####################################################

    # return feature x and y coordinate

    def get_feature_coor(self, feature, obj):

        for item in obj["predictions"]:

            class_name = item.get('class')

            if class_name == feature:

                feature_x_value = item.get('x')

                feature_y_value = item.get('y')

                return feature_x_value, feature_y_value


    ## origin at top left

    def adjust_in_x_img(self, param):

        step = 0

        step = math.log(abs(960-param))/100.0 # scale from pixel to actual distance

        if param < 960: 
            #move right in img, in cart move down, in negative y direction

            self.move_cart(0, step, 0, 0.05)

        else:

            self.move_cart(0, -step, 0, 0.05)

    def adjust_in_y_img(self, param):

        step = 0

        step = math.log(abs(540-param))/100.0 # scale from pixel to actual distanc

        if param < 540:
            #move in negative x dir on gripper

            self.move_cart(-step, 0, 0, 0.05)

        else:

            self.move_cart(step, 0, 0, 0.05)


    def adjust(self, param_x, param_y):

        while(abs(param_x - 960) > 10):

            adjust_in_x_img(self, param_x)

        while(abs(param_y - 540) > 10):

            adjust_in_y_img(self, param_y)


####################################################| STIFFNESS |####################################################
        
    def start_controllers(self):
        print("================== Adjusting stiffness: 3/4 (Starting controllers)")
        req = SwitchControllerRequest()
        req.start_controllers = ['franka_state_controller','position_joint_trajectory_controller']
        req.strictness = 1
        self.switch_controller(req)

    def set_joint_stiffness(self,stiffness):
        print("================== Adjusting stiffness: 2/4 (Setting joint stiffness)")
        req = SetJointImpedanceRequest()
        req.joint_stiffness = stiffness
        self.set_joint_impedance(req)

    def stop_controllers(self):
        print("================== Adjusting stiffness: 1/4 (Stopping controllers)")
        req = SwitchControllerRequest()
        req.stop_controllers = ['franka_state_controller','position_joint_trajectory_controller']
        req.strictness = 1
        self.switch_controller(req)
        
    def rigid(self):
        print("================== Adjusting stiffness: 0/4")
        self.stop_controllers()
        stiffness = const.STIFFNESS_RIGID
        self.set_joint_stiffness(stiffness)
        self.start_controllers()
        print("================== Adjusting stiffness: 4/4 (Stiffness adjusted to rigid)")

    def soft(self,type=0):
        print("================== Adjusting stiffness: 0/4")
        self.stop_controllers()
        if type == 0:
            stiffness = const.STIFFNESS_SOFT
        elif type == 1:
            stiffness = const.STIFFNESS_SOFT_DOOR
        self.set_joint_stiffness(stiffness)
        self.start_controllers()
        print("================== Adjusting stiffness: 4/4 (Stiffness adjusted to soft)")

####################################################| RESET |####################################################

    def set_vel(self, vel1=0.3, acc1=0.3, vel2=0.02, acc2=0.02):
        self.move_group.set_max_velocity_scaling_factor(vel1)
        self.move_group.set_max_acceleration_scaling_factor(acc1)
        self.move_group_hand.set_max_velocity_scaling_factor(vel2)
        self.move_group_hand.set_max_acceleration_scaling_factor(acc2)
        
        print("================== Joints speed: " + str(vel1))
        print("================== Joints acceleration: " + str(acc1))
        print("================== Hands speed: " + str(vel2))
        print("================== Hands acceleration: " + str(acc2))

    def reset(self):
        self.move_group.set_max_velocity_scaling_factor(0.5)
        self.move_group.set_max_acceleration_scaling_factor(0.5)
        
        
        waypoints = []

        wpose = self.move_group.get_current_pose().pose
        wpose.position.x += 0  
        wpose.position.y += 0
        wpose.position.z += 0.1
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:

        new_plan=self.move_group.retime_trajectory(self.move_group.get_current_state(),
                                    plan,velocity_scaling_factor=0.5,
                                    acceleration_scaling_factor=0.5)
        
        #display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        #display_trajectory.trajectory_start = robot.get_current_state()
        #display_trajectory.trajectory.append(new_plan)
        # Publish
        #display_trajectory_publisher.publish(display_trajectory)
        
        self.move_group.execute(new_plan, wait=True)
    
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = 0 * pi /180
        joint_goal[1] = -48 * pi / 180
        joint_goal[2] = 0 * pi / 180
        joint_goal[3] = -153 * pi / 180
        joint_goal[4] = 0
        joint_goal[5] = 106 * pi / 180
        joint_goal[6] = 46 * pi / 180

        self.move_group.go(joint_goal, wait=True)

        self.move_group.stop()

        self.grip()

        print("================== Reset to home position")

    def good_view(self):

        pose_goal = self.move_group.get_current_pose().pose

        pose_goal.position.x = 0.40242925036
        pose_goal.position.y = 0.0999230329406
        pose_goal.position.z = 0.51043307492


        pose_goal.orientation.x = 0.9238795
        pose_goal.orientation.y = -0.3826834
        pose_goal.orientation.z = 0
        pose_goal.orientation.w = 0 

        self.move_group.set_pose_target(pose_goal)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        print("================== Arrive at good view")

        time.sleep(0.1)

        #self.set_angle()


    def set_origin(self):

        self.origin_x = self.move_group.get_current_pose().pose.position.x
        self.origin_y = self.move_group.get_current_pose().pose.position.y
        print (str(self.origin_x)+","+str(self.origin_y))

    def set_angle(self):
        self.angle = self.go.get_angle_diff()
        print("================== Set angle difference to: " + str(self.angle))

####################################################| MOVEMENT |####################################################

    def cart_to(self,x,y,z,v):

        waypoints = []

        wpose = self.move_group.get_current_pose().pose

        #x_offset = math.cos(-self.angle) * abs(x) + math.sin(-self.angle) * (-abs(y))
        #y_offset = math.cos(-self.angle) * (-abs(y)) + math.sin(-self.angle) * abs(x) 

        #new_x = self.origin_x - x_offset
        #new_y = self.origin_y - y_offset
        
        wpose.position.x = x
        wpose.position.y = y
        wpose.position.z = z

        #print("Rel: " + str(x) + "," + str(y))
        #print("Origin: " + str(self.origin_x) +","+ str(self.origin_y))
        #print("Offset: "+str(x_offset) + "," + str(y_offset))
        #print("Absolute: "+str(new_x) + "," + str(new_y))


        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        new_plan=self.move_group.retime_trajectory(self.move_group.get_current_state(),
                                    plan,velocity_scaling_factor=v,
                                    acceleration_scaling_factor=v)
        
        self.move_group.execute(new_plan, wait=True)

        print("================== Moving in cartesian")

    def pose_to(self,x,y,z,w):
        waypoints = []

        wpose = self.move_group.get_current_pose().pose
        wpose.orientation.x = x  
        wpose.orientation.y = y  
        wpose.orientation.z = z  
        wpose.orientation.w = w
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        new_plan=self.move_group.retime_trajectory(self.move_group.get_current_state(),
                                    plan,velocity_scaling_factor=0.2,
                                    acceleration_scaling_factor=0.2)
        
        self.move_group.execute(new_plan, wait=True)

        print("================== Rotating")

    def move_cart(self,x,y,z,v):
        waypoints = []

        wpose = self.move_group.get_current_pose().pose
        wpose.position.x += x  #out of page if +
        wpose.position.y -= y  #left if +
        wpose.position.z -= z  #downward if +
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        new_plan=self.move_group.retime_trajectory(self.move_group.get_current_state(),
                                    plan,velocity_scaling_factor=v,
                                    acceleration_scaling_factor=v)
        self.move_group.execute(new_plan, wait=True)

        print("================== Moving in cartesian")


####################################################| SLIDER |####################################################

    def move_slider(self,x):

        print("================== Moving slider")

        waypoints = []

        wpose = self.move_group.get_current_pose().pose
        wpose.position.x += x  #out of page if +
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        new_plan=self.move_group.retime_trajectory(self.move_group.get_current_state(),
                                    plan,velocity_scaling_factor=0.007,
                                    acceleration_scaling_factor=0.007)

        self.move_group.execute(new_plan, wait=True)


        

        



####################################################| TASK 1 |####################################################


    def cart_to_blue_button(self): #Task1-1

        x = const.POSE_GOAL_BLUE_BUTTON[0]
        y = const.POSE_GOAL_BLUE_BUTTON[1]
        z = const.POSE_GOAL_BLUE_BUTTON[2]
        qx = const.POSE_GOAL_BLUE_BUTTON[3]
        qy = const.POSE_GOAL_BLUE_BUTTON[4]
        qz = const.POSE_GOAL_BLUE_BUTTON[5]
        qw = const.POSE_GOAL_BLUE_BUTTON[6]
        v = 0.5

        self.pose_to(qx,qy,qz,qw)
        self.cart_to(x,y,z,v)


    def push_button(self): #Task1-2
        
        self.soft()
        self.grip()        
        
        waypoints = []

        wpose = self.move_group.get_current_pose().pose
        
        wpose.position.z -= 0.04
        wpose.position.x += -0.01
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.z += 0.08  
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        new_plan=self.move_group.retime_trajectory(self.move_group.get_current_state(),
                                    plan,velocity_scaling_factor=0.1,
                                    acceleration_scaling_factor=0.1)
        self.move_group.execute(new_plan, wait=True)
        
        self.rigid()


####################################################| TASK 2 |####################################################


    def cart_to_screen(self): #Task 2-1

        x = const.POSE_GOAL_SCREEN[0]
        y = const.POSE_GOAL_SCREEN[1]
        z = const.POSE_GOAL_SCREEN[2]
        qx = const.POSE_GOAL_SCREEN[3]
        qy = const.POSE_GOAL_SCREEN[4]
        qz = const.POSE_GOAL_SCREEN[5]
        qw = const.POSE_GOAL_SCREEN[6]
        v = 0.5

        self.cart_to(x,y,z,v)
        self.pose_to(qx,qy,qz,qw)

    def cart_to_slider(self): #Task 2-2

        x = const.POSE_GOAL_SLIDER_START[0]
        y = const.POSE_GOAL_SLIDER_START[1]
        z = const.POSE_GOAL_SLIDER_START[2]
        qx = const.POSE_GOAL_SLIDER_START[3]
        qy = const.POSE_GOAL_SLIDER_START[4]
        qz = const.POSE_GOAL_SLIDER_START[5]
        qw = const.POSE_GOAL_SLIDER_START[6]
        v = 0.5

        self.pose_to(qx,qy,qz,qw)
        self.cart_to(x,y,z,v)

    def adjust_slider_mid(self): #Task 2-3
        self.move_slider(0.0325)


    def adjust_slider_same_side(self,dist): #Task 2-4-1

        self.cart_to_slider()
        #self.adjust_slider_mid()
        self.move_slider(dist+0.0325)

    def cart_to_slider_mid(self): #Task 2-4-2

        x = const.POSE_GOAL_SLIDER_MID[0]
        y = const.POSE_GOAL_SLIDER_MID[1]
        z = const.POSE_GOAL_SLIDER_MID[2]
        qx = const.POSE_GOAL_SLIDER_MID[3]
        qy = const.POSE_GOAL_SLIDER_MID[4]
        qz = const.POSE_GOAL_SLIDER_MID[5]
        qw = const.POSE_GOAL_SLIDER_MID[6]
        v = 0.25

        self.pose_to(qx,qy,qz,qw)
        self.cart_to(x,y,z,v)

    def adjust_slider_other_side(self,dist): #Task 2-4-3
        
        self.move_cart(0.0,0,-0.05,0.1)

        self.grip()
        self.cart_to_slider_mid()
        
        self.move_slider(dist)


####################################################| TASK 3 |####################################################


    def cart_to_plug(self):

        self.move_cart(0,0,-0.03,0.1)

        x = const.POSE_GOAL_PLUG[0]
        y = const.POSE_GOAL_PLUG[1]
        z = const.POSE_GOAL_PLUG[2]
        qx = const.POSE_GOAL_PLUG[3]
        qy = const.POSE_GOAL_PLUG[4]
        qz = const.POSE_GOAL_PLUG[5]
        qw = const.POSE_GOAL_PLUG[6]
        v = 0.3

        self.pose_to(qx,qy,qz,qw)
        self.cart_to(x,y,z,v)    

    

    def grab_plug(self):
        goal = GraspGoal()
        goal.width = 0.0106
        goal.epsilon.inner = 0.005
        goal.epsilon.outer = 0.005
        goal.speed = 0.25
        goal.force = 10.
        self.gripper_grasp_client.send_goal_and_wait(goal)
        self.gripper_grasp_client.get_result()


    def unplug(self):

        self.midgrip(0.02)
        self.move_cart(0,0,0.02,0.5)
        self.grab_plug()
        self.move_cart(0,0,-0.03,0.5)

    def cart_to_plugin(self):

        x = const.POSE_GOAL_PLUGIN[0]
        y = const.POSE_GOAL_PLUGIN[1]
        z = const.POSE_GOAL_PLUGIN[2]
        qx = const.POSE_GOAL_PLUGIN[3]
        qy = const.POSE_GOAL_PLUGIN[4]
        qz = const.POSE_GOAL_PLUGIN[5]
        qw = const.POSE_GOAL_PLUGIN[6]
        v = 0.1

        self.pose_to(qx,qy,qz,qw)
        self.cart_to(x,y,z,v)    
        
    def plugin(self):

        self.move_cart(0,0,0.021,0.005)

        self.midgrip(0.015)
        self.move_cart(0,0,-0.024,0.25)

        #self.grip()
        #self.move_cart(0,0,0.017,0.05)

####################################################| TASK 4-1 |####################################################

    def grab_door(self):
    
        goal = GraspGoal()
        goal.width = 0.011
        goal.epsilon.inner = 0.005
        goal.epsilon.outer = 0.005
        goal.speed = 0.1
        goal.force = 15.
        self.gripper_grasp_client.send_goal_and_wait(goal)
        self.gripper_grasp_client.get_result()

    def cart_to_door(self):
        
        self.move_cart(0,0,-0.02,0.1)
        self.grip()

        x = const.POSE_GOAL_DOOR_START[0]
        y = const.POSE_GOAL_DOOR_START[1]
        z = const.POSE_GOAL_DOOR_START[2]
        
        qx = const.POSE_GOAL_DOOR_START[3]
        qy = const.POSE_GOAL_DOOR_START[4]
        qz = const.POSE_GOAL_DOOR_START[5]
        qw = const.POSE_GOAL_DOOR_START[6]
        
        v = 0.5
        
        self.pose_to(qx,qy,qz,qw)
        self.cart_to(x,y,z,v) 
    

    def cart_to_door_stage1(self):

        x = const.POSE_GOAL_DOOR_STAGE1[0]
        y = const.POSE_GOAL_DOOR_STAGE1[1]
        z = const.POSE_GOAL_DOOR_STAGE1[2]
        qx = const.POSE_GOAL_DOOR_STAGE1[3]
        qy = const.POSE_GOAL_DOOR_STAGE1[4]
        qz = const.POSE_GOAL_DOOR_STAGE1[5]
        qw = const.POSE_GOAL_DOOR_STAGE1[6]
        v = 0.5

        self.pose_to(qx,qy,qz,qw)
        self.cart_to(x,y,z,v) 

    def cart_to_door_stage2(self):

        x = const.POSE_GOAL_DOOR_STAGE2[0]
        y = const.POSE_GOAL_DOOR_STAGE2[1]
        z = const.POSE_GOAL_DOOR_STAGE2[2]
        qx = const.POSE_GOAL_DOOR_STAGE2[3]
        qy = const.POSE_GOAL_DOOR_STAGE2[4]
        qz = const.POSE_GOAL_DOOR_STAGE2[5]
        qw = const.POSE_GOAL_DOOR_STAGE2[6]
        v = 0.5

        self.pose_to(qx,qy,qz,qw)
        self.cart_to(x,y,z,v) 

    def cart_to_door_finish(self):

        x = const.POSE_GOAL_DOOR_FINISH[0]
        y = const.POSE_GOAL_DOOR_FINISH[1]
        z = const.POSE_GOAL_DOOR_FINISH[2]
        qx = const.POSE_GOAL_DOOR_FINISH[3]
        qy = const.POSE_GOAL_DOOR_FINISH[4]
        qz = const.POSE_GOAL_DOOR_FINISH[5]
        qw = const.POSE_GOAL_DOOR_FINISH[6]
        v = 0.5

        self.pose_to(qx,qy,qz,qw)
        self.cart_to(x,y,z,v)

    def open_door(self):

        self.midgrip(0.02)
        self.soft(1)
        self.move_cart(0,0,0.0225,0.5)
        self.grab_door()
        self.move_cart(0,0,-0.04,0.5)
        self.cart_to_door_stage1()
        self.cart_to_door_stage2()
        self.rigid()
        self.midgrip()
        self.move_cart(0,0,-0.05,0.5)
        self.cart_to_door_finish()
        self.move_cart(0,0,0.03,0.5)
        self.move_cart(-0.02,0,0,0.5)
        

####################################################| TASK 4-2 |####################################################

    def grab_probe(self):
    
        goal = GraspGoal()
        goal.width = 0.010
        goal.epsilon.inner = 0.005
        goal.epsilon.outer = 0.005
        goal.speed = 0.1
        goal.force = 20.
        self.gripper_grasp_client.send_goal_and_wait(goal)
        self.gripper_grasp_client.get_result()

    def cart_to_probe_in(self):

        self.move_cart(0,0,-0.1,0.1)
        '''
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = -2.76810106704692
        joint_goal[1] = -0.7191907249920623
        joint_goal[2] = 2.7773669349582524
        joint_goal[3] = -1.7086031594766102
        joint_goal[4] = 0.5557237032517116
        joint_goal[5] = 0.902706339524257
        joint_goal[6] = -2.313783632827302

        self.move_group.go(joint_goal, wait=True)

        self.move_group.stop()
        '''
        x = const.POSE_GOAL_PROBE_IN[0]
        y = const.POSE_GOAL_PROBE_IN[1]
        z = const.POSE_GOAL_PROBE_IN[2]
        qx = const.POSE_GOAL_PROBE_IN[3]
        qy = const.POSE_GOAL_PROBE_IN[4]
        qz = const.POSE_GOAL_PROBE_IN[5]
        qw = const.POSE_GOAL_PROBE_IN[6]
        v = 0.1

        self.pose_to(qx,qy,qz,qw)
        self.cart_to(x,y,z,v)

    def cart_to_probe_grab(self):

        self.midgrip(0.015)
        '''
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = -2.419019707805593
        joint_goal[1] = -0.08182549919832943
        joint_goal[2] = 2.814151454282088
        joint_goal[3] = -2.514985725786662
        joint_goal[4] = 0.08528800381031285
        joint_goal[5] = 2.6450291269090442
        joint_goal[6] = -2.0106674916368967

        self.move_group.go(joint_goal, wait=True)

        self.move_group.stop()
        '''
        x = const.POSE_GOAL_PROBE_GRAB[0]
        y = const.POSE_GOAL_PROBE_GRAB[1]
        z = const.POSE_GOAL_PROBE_GRAB[2]
        qx = const.POSE_GOAL_PROBE_GRAB[3]
        qy = const.POSE_GOAL_PROBE_GRAB[4]
        qz = const.POSE_GOAL_PROBE_GRAB[5]
        qw = const.POSE_GOAL_PROBE_GRAB[6]
        v = 0.1


        

        self.cart_to(x,y,z,v)
        
        self.pose_to(qx,qy,qz,qw)

    def cart_to_probe_stage1(self):

        x = const.POSE_GOAL_PROBE_STAGE1[0]
        y = const.POSE_GOAL_PROBE_STAGE1[1]
        z = const.POSE_GOAL_PROBE_STAGE1[2]
        qx = const.POSE_GOAL_PROBE_STAGE1[3]
        qy = const.POSE_GOAL_PROBE_STAGE1[4]
        qz = const.POSE_GOAL_PROBE_STAGE1[5]
        qw = const.POSE_GOAL_PROBE_STAGE1[6]
        v = 0.1

        self.pose_to(qx,qy,qz,qw)
        self.cart_to(x,y,z,v)

    def pull_out_probe(self):

        self.cart_to_probe_grab()
        self.move_cart(0,0,0.02,0.05)
        self.grab_probe()
        self.cart_to_probe_stage1()
        self.move_cart(0,0,-0.1,0.1)

####################################################| TASK 6 |####################################################


    def cart_to_red_button(self): #Task6

        x = const.POSE_GOAL_RED_BUTTON[0]
        y = const.POSE_GOAL_RED_BUTTON[1]
        z = const.POSE_GOAL_RED_BUTTON[2]
        qx = const.POSE_GOAL_RED_BUTTON[3]
        qy = const.POSE_GOAL_RED_BUTTON[4]
        qz = const.POSE_GOAL_RED_BUTTON[5]
        qw = const.POSE_GOAL_RED_BUTTON[6]
        v = 0.5

        self.pose_to(qx,qy,qz,qw)
        self.cart_to(x,y,z,v)
