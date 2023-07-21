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

    def setTargetSfHand(self,j6a,j6b):
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

    achs.setTarget6Achs(0, 0, 0, 0, 0, 0)
    achs.setTarget6Achs(-1.941, 0.396, -0.368, -0.604, -0.213, 0.761)
    achs.setTarget6Achs(-1.94, 0.541, -0.394, -1.486, -0.119, 1.653)
    achs.setTarget6Achs(-1.940, 0.623, -0.241, -0.988, -0.145, 1.152)
    achs.setTarget6Achs(-1.982, 0.704, -0.086, -0.581, -0.177, 0.708)
    achs.setTarget6Achs(-2.053, 0.775, 0.0494, -0.266, -0.216, 0.337)
    achs.setTarget6Achs(-2.165, 0.812, 0.120, 0.035, -0.241, -0.0472)
    achs.setTarget6Achs(-2.232, 0.7919, 0.080, 0.209, -0.228, -0.270)
    achs.setTarget6Achs(-2.277, 0.760, 0.018, 0.360, -0.209, -0.456)
    achs.setTarget6Achs(-2.357, 0.658, -0.179, 0.832, -0.160, -0.993)
    achs.setTarget6Achs(-2.380, 0.585, -0.319, 1.236, -0.138, -1.419)
    achs.setTarget6Achs(-2.356, 0.512, -0.455, 1.728, -0.118, -1.897)
    achs.setTarget6Achs(-2.332, 0.481, -0.515, 2.015, -0.116, -2.165)
    achs.setTarget6Achs(-2.081, 0.443, -0.576, -2.694, -0.088, 2.748)
    achs.setTarget6Achs(-1.979, 0.489, -0.490, -1.933, -0.103, 2.069)
    achs.setTarget6Achs(-1.94, 0.541, -0.394, -1.486, -0.119, 1.653)
    achs.setTarget6Achs(-1.941, 0.396, -0.368, -0.604, -0.213, 0.761)
    achs.setTarget6Achs(0, 0, 0, 0, 0, 0)
    achs.move()
    scara.setTargetScara(0, 0, 0, 0)
    scara.setTargetScara(0.19563, 1.2341, 0, 0)
    scara.setTargetScara(0.19563, 1.2341, 0.05, 0)
    scara.move()
    time.sleep(3)
    scara.setTargetScara(0.19563, 1.2341, 0, 0)
    scara.setTargetScara(0, 0, 0, 0)
    scara.move()

    sf_hand.setTargetSfHand(-0.01, 0.01)
    sf_hand.move()

    sf.setTargetStandford(0, 0, 0, 0, 0)
    sf.setTargetStandford(0, -tau / 4, -0.038, 0, 0)
    sf.setTargetStandford(0, -tau / 4 - 0.5, -0.038, 0.5, 0)
    sf.setTargetStandford(0, -2.27, -0.01, 0.7, 0)
    sf.setTargetStandford(0, -2.17, 0.005, 0.6, 0)
    sf.setTargetStandford(0, -2.17, 0.008, 0.6, 0)
    sf.move()

    sf_hand.setTargetSfHand(0, 0.00001)
    sf_hand.move()

    sf.move_group.set_max_acceleration_scaling_factor(0.4)
    sf.move_group.set_max_velocity_scaling_factor(0.4)

    sf.setTargetStandford(0, -1.521, -0.019, -0.049, 0)
    sf.setTargetStandford(-1.57, -1.521, -0.019, -0.049, 0)
    sf.setTargetStandford(-1.57, -2.075, 0.034, 0.504, 0)

    sf.move()

    sf_hand.setTargetSfHand(-0.01, 0.01)
    sf_hand.move()

    sf.setTargetStandford(-1.57, -2.17, 0.005, 0.6, 0)
    sf.setTargetStandford(-1.57, -2.27, -0.01, 0.7, 0)
    sf.setTargetStandford(-1.57, -tau / 4 - 0.5, -0.038, 0.5, 0)
    sf.setTargetStandford(-1.57, -1.34, -0.038, -0.959, 0)
    sf.setTargetStandford(-1.57, -0.75, -0.038, -0.959, 0)
    sf.setTargetStandford(0, 0, 0, 0, 0)
    sf.move()

    del scara
    del achs
    del sf
    del sf_hand