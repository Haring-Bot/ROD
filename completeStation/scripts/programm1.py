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
import tf.transformations as tf_trans


class robot:
    def __init__(self,nodename="standardPythonController",groupname="standardRobot"):
        rospy.init_node(nodename, anonymous=False)
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.move_group = moveit_commander.MoveGroupCommander(groupname)
        
        #Acceleration and Speed factors
        self.move_group.set_max_acceleration_scaling_factor(1)
        self.move_group.set_max_velocity_scaling_factor(1)
        
        self.move_group.set_goal_tolerance(0.1) #For real robot set the tolerance to 1 mm
        self.move_group.set_planning_time(5)
        self.goals = []

    def setTarget(self,x,y,z,roll,pitch,yaw):
        goal = geometry_msgs.msg.Pose()

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        goal.position.x = x
        goal.position.y = y
        goal.position.z = z
        goal.orientation.x = qx
        goal.orientation.y = qy
        goal.orientation.z = qz
        goal.orientation.w = qw

        try:
            self.goals.append(copy.deepcopy(goal))
        except:
            print("Target not set")

    def move(self):

        self.move_group.clear_pose_targets()
        try:
            for g in self.goals:
                self.move_group.set_pose_target(g)
                self.move_group.go(wait=True)
                """print("x: ", self.cp.position.x)
                print("y: ", self.cp.position.y)
                print("z: ", self.cp.position.z)"""
        except:
            print("Targets not reachable")
        finally:
            print("Stopping")
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            self.goals = []
   

    def getPoseAndPrint(self):
        self.cp = self.move_group.get_current_pose().pose
        print("Info: current pose ")
        print("x: ", self.cp.position.x)
        print("y: ", self.cp.position.y)
        print("z: ", self.cp.position.z)

        # Convert quaternion to Euler angles (in radians)
        q = self.cp.orientation
        roll, pitch, yaw = tf_trans.euler_from_quaternion([q.x, q.y, q.z, q.w])

        print("roll (rad): ", roll)
        print("pitch (rad):", pitch)
        print("yaw (rad):  ", yaw)

if __name__ == "__main__":
    #s = robot(nodename="pythonController",groupname="scara_planner")
    #h = robot(nodename="pythonController",groupname="handler_planner")
    p = robot(nodename="pythonController",groupname="polisher_planner")
    
    #s.getPoseAndPrint()
    #h.getPoseAndPrint()
    p.getPoseAndPrint()
    #s.setTarget(0.5, 0.4, 0.92,   -1.5708000000573443,0,-1.5) #dropoff scara
    #h.setTarget(1.1, 0.6, 0.7,   1.57,0,-1.57) #pickup handler
    p.setTarget(0.6, 0.6, 1.2,   -3.6, 0 ,-1.57) #pickup handler
    #s.move()
    #h.move()
    p.move()
    #h.getPoseAndPrint()
    
    #del s
    #del h
    del p