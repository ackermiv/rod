#!/usr/bin/env python3
import copy
import rospy
import roslaunch
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
import sys
import math
import time 
from math import gamma, pi, tau, dist, fabs, cos

class demo_robot:
    def __init__(self,nodename="demo_robot",groupname="robot"):
        rospy.init_node(nodename, anonymous=False)
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.move_group = moveit_commander.MoveGroupCommander(groupname)
        
        #Acceleration and Speed factors
        self.move_group.set_max_acceleration_scaling_factor(1)
        self.move_group.set_max_velocity_scaling_factor(1)
        
        self.move_group.set_goal_tolerance(0.001) #For real robot set the tolerance to 1 mm
        self.goals = []

    def getPoseAndPrint(self):
        self.cp = self.move_group.get_current_pose().pose
        print("Info: current pose ")
        print("x: ",self.cp.position.x)
        print("y: ",self.cp.position.y)
        print("z: ",self.cp.position.z)
        print("qx: ",self.cp.orientation.x)
        print("qy: ",self.cp.orientation.y)
        print("qz: ",self.cp.orientation.z)
        print("qw: ",self.cp.orientation.w)

    def setTargetScara(self,j1,j2,j3,j4):
        goal = self.move_group.get_current_joint_values()

        goal[0] = j1
        goal[1] = j2
        goal[2] = j3
        goal[3] = j4

        try:
            self.goals.append(copy.deepcopy(goal))
    
        except:
            print("Target not set")
    
    def setTarget6Achs(self,j1,j2,j3,j4,j5,j6):
        goal = self.move_group.get_current_joint_values()

        goal[0] = j1
        goal[1] = j2
        goal[2] = j3
        goal[3] = j4
        goal[4] = j5
        goal[5] = j6

        try:
            self.goals.append(copy.deepcopy(goal))
    
        except:
            print("Target not set")

    def setTargetStandford(self,j1,j2,j3,j4,j5):
        goal = self.move_group.get_current_joint_values()

        goal[0] = j1
        goal[1] = j2
        goal[2] = j3
        goal[3] = j4
        goal[4] = j5

        try:
            self.goals.append(copy.deepcopy(goal))
    
        except:
            print("Target not set")

    def setTargetSfHand(self,j6b,j6a):
        goal = self.move_group.get_current_joint_values()

        goal[0] = j6a
        goal[1] = j6b

        try:
            self.goals.append(copy.deepcopy(goal))
    
        except:
            print("Target not set")

    def move(self):

        self.move_group.clear_pose_targets()
        try:
            for g in self.goals:
                self.move_group.go(g, wait=True)
        except:
            print("Targets not reachable")
        finally:
            print("Stopping")
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            self.goals = []

if __name__ == "__main__":
    scara = demo_robot(nodename="demo_robot",groupname="scara_group")
    achs = demo_robot(nodename="demo_robot",groupname="arm")
    sf = demo_robot(nodename="demo_robot",groupname="standford_arm_group")
    sf_hand = demo_robot(nodename="demo_robot",groupname="standford_hand")

    achs.setTarget6Achs(0,0,0,0,0,0)
    achs.setTarget6Achs(-2.15,0.5,0,0,-1.07,0)
    achs.setTarget6Achs(-1.969,0.6,-0.173,0,-0.797,0.180)
    achs.setTarget6Achs(-2.15,0.5,0,0,-1.07,0)
    achs.setTarget6Achs(0,0,0,0,0,0)
    achs.move()

    scara.setTargetScara(0,0,0,0)
    scara.setTargetScara(0.19563,1.2341,0,0)
    scara.setTargetScara(0.19563,1.2341,0.05,0)
    scara.setTargetScara(0.19563,1.2341,0,0)
    scara.setTargetScara(0,0,0,0)
    scara.move()
    
    sf_hand.setTargetSfHand(0,0.00001)
    sf_hand.move()

    sf.setTargetStandford(0,0,0,0,0)
    sf.setTargetStandford(0,-2.07,-0.2,0.5,0)
    sf.setTargetStandford(0,-2.27,-0.1,0.7,0)
    sf.setTargetStandford(0,-2.17,0.005,0.6,0)
    sf.setTargetStandford(0,-2.17,0.008,0.6,0)
    sf.move()

    sf_hand.setTargetSfHand(-0.01,0.01)
    sf_hand.move()

    sf.setTargetStandford(0,-1.07,0,0.5,0)
    sf.setTargetStandford(1.57,-1.07,0,0.5,0)
    sf.move()

    del scara
    del achs
    del sf
    del sf_hand