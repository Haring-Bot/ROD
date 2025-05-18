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
import pdb

class robot:
    def __init__(self,nodename="standardPythonController",groupname="standardRobot"):
        rospy.init_node(nodename, anonymous=False)
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.MoveGroupCommander(groupname)
        self.move_group = moveit_commander.MoveGroupCommander(groupname)

        self.move_group.set_max_acceleration_scaling_factor(0.2)
        self.move_group.set_max_velocity_scaling_factor(0.2)

        self.move_group.set_planning_time(10.0)
        self.move_group.allow_replanning(True)
        self.move_group.set_goal_tolerance(0.01)

        self.goals = []

    def moveL(self, offsetX, offsetY, offsetZ, offsetRoll=0, offsetPitch=0, offsetYaw=0):
        waypoints = []
        wpose = self.move_group.get_current_pose().pose

        # Update position
        wpose.position.x += offsetX
        wpose.position.y += offsetY
        wpose.position.z += offsetZ

        # Update orientation
        q = wpose.orientation
        curRoll, curPitch, curYaw = tf_trans.euler_from_quaternion([q.x, q.y, q.z, q.w])
        newRoll = curRoll + offsetRoll
        newPitch = curPitch + offsetPitch
        newYaw = curYaw + offsetYaw
        new_q = tf_trans.quaternion_from_euler(newRoll, newPitch, newYaw)
        wpose.orientation.x = new_q[0]
        wpose.orientation.y = new_q[1]
        wpose.orientation.z = new_q[2]
        wpose.orientation.w = new_q[3]

        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)

        self.move_group.execute(plan, wait=True)
        # move_group.compute_cartesian_path(waypoints, eef_step(stepsize to interpolate), jump_threshold)

    def movetoPose(self, goalX = 0, goalY = 0, goalZ = 0, goalRoll = None, goalPitch = None, goalYaw = None):
        self.move_group.set_start_state_to_current_state()
        self.curPos = self.move_group.get_current_pose().pose
        curX = self.curPos.position.x
        curY = self.curPos.position.y
        curZ = self.curPos.position.z
        q = self.curPos.orientation
        curRoll, curPitch, curYaw = tf_trans.euler_from_quaternion([q.x, q.y, q.z, q.w])

        offsetX = goalX - curX
        offsetY = goalY - curY
        offsetZ = goalZ - curZ

        if goalRoll is not None and goalPitch is not None and goalYaw is not None:
            offsetRoll = goalRoll - curRoll
            offsetPitch = goalPitch - curPitch
            offsetYaw = goalYaw - curYaw
            self.moveL(offsetX=offsetX, offsetY=offsetY, offsetZ=offsetZ, offsetRoll=offsetRoll, offsetPitch=offsetPitch, offsetYaw=offsetYaw)
        else:
            self.moveL(offsetX=offsetX, offsetY=offsetY, offsetZ=offsetZ)
        
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
                success = self.move_group.set_pose_target(g)
                if not success:
                    print("⚠️  IK could not find a solution for the target pose.")
                    continue

                result = self.move_group.go(wait=True)
                if not result:
                    print("⚠️  MoveIt could not execute the trajectory.")
                    continue  # Added continue to fix indentation error

                self.move_group.go(wait=True)
        except:
            print("Targets not reachable")
        finally:
            print("Stopping")
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            self.goals = []

def printWorkspaceSize():
    min_corner = rospy.get_param("/move_group/workspace_parameters/min_corner", None)
    max_corner = rospy.get_param("/move_group/workspace_parameters/max_corner", None)

    print("Workspace min_corner:", min_corner)
    print("Workspace max_corner:", max_corner)

def movetoHome(scara, handler, polisher):
    print("moving robots to home position")
    scara.move_group.set_start_state_to_current_state()
    handler.move_group.set_start_state_to_current_state()
    polisher.move_group.set_start_state_to_current_state()
    scara.move_group.set_named_target("home")
    handler.move_group.set_named_target("home")
    polisher.move_group.set_named_target("home")
    scara.move_group.go(wait=True)
    handler.move_group.go(wait=True)
    polisher.move_group.go(wait=True)

def movetoPickup(scara, handler, polisher):
    print("moving robots to pickup position")
    scara.move_group.set_start_state_to_current_state()
    scara.move_group.set_named_target("pickup")
    scara.move_group.go(wait=True)

def movetoSwitch(scara, handler, polisher):
    print("moving robots to switch position")
    handler.move_group.set_start_state_to_current_state()
    scara.move_group.set_start_state_to_current_state()
    handler.move_group.set_named_target("switch")
    scara.move_group.set_named_target("switch")
    handler.move_group.go(wait=True)
    scara.move_group.go(wait=True)

def movetoPolish(scara, handler, polisher):
    print("moving robots to polish position")
    scara.move_group.set_start_state_to_current_state()
    polisher.move_group.set_start_state_to_current_state()
    handler.move_group.set_start_state_to_current_state()
    scara.move_group.set_named_target("home")
    polisher.move_group.set_named_target("standby")
    handler.move_group.set_named_target("polish")
    scara.move_group.go(wait=True)
    polisher.move_group.go(wait=True)
    handler.move_group.go(wait=True)

if __name__ == "__main__":
    h = robot(nodename="pythonController",groupname="handler_planner")
    p = robot(nodename="pythonController",groupname="polisher_planner")
    s = robot(nodename="pythonController",groupname="scara_planner")

    printWorkspaceSize()

    #s.setTarget(x=0.600, y=0.471, z=0.930, roll=-1.570, pitch=7.346, yaw=-1.570)#home
    #s.setTarget(x=0.307, y=0.188, z=0.933, roll=-1.57, pitch=1.371, yaw=-1.570)#pickup
    #s.moveL(offsetX=-0.293, offsetY=-0.283, offsetZ=0.003)
    
    #s.getPoseAndPrint()
    h.getPoseAndPrint()

    movetoHome(s,h,p)
    time.sleep(0.5)
    movetoPickup(s, h, p)
    time.sleep(0.5)
    movetoSwitch(s, h, p)
    time.sleep(0.5)
    movetoPolish(s, h, p)
    time.sleep(0.5)
    #movetoSwitch(s, h, p)

    del h, p, s

