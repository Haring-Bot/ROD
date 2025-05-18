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
        
        self.move_group.set_planning_pipeline_id("ompl")
        self.move_group.set_start_state_to_current_state()


        #Acceleration and Speed factors
        self.move_group.set_max_acceleration_scaling_factor(0.2)
        self.move_group.set_max_velocity_scaling_factor(0.2)
        
        self.move_group.set_goal_tolerance(0.01) #For real robot set the tolerance to 1 mm
        self.move_group.set_planning_time(10)
        self.move_group.allow_replanning(True)
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
            # Set the start state to the current state
            self.move_group.set_start_state_to_current_state()

            # Set a different planner for better performance
            self.move_group.set_planner_id("RRTConnectkConfigDefault")

            # Relax the goal tolerance for debugging
            self.move_group.set_goal_tolerance(0.05)  # Increased from 0.01 to 0.05 meters

            # Increase planning time
            self.move_group.set_planning_time(10)  # Increased from 10 to 20 seconds

            # Print joint limits
            print("Joint limits:")
            for joint in self.move_group.get_active_joints():
                joint_limits = self.robot.get_joint(joint)  # Get joint object
                print(f"{joint}:")
                print(f"  Lower limit: {joint_limits.min_bound()}")
                print(f"  Upper limit: {joint_limits.max_bound()}")

            # Print current joint values
            current_joint_values = self.move_group.get_current_joint_values()
            print("Current joint values:", current_joint_values)

            for g in self.goals:
                print("Attempting to move to target:")
                print("Position: x={}, y={}, z={}".format(g.position.x, g.position.y, g.position.z))
                print("Orientation: x={}, y={}, z={}, w={}".format(g.orientation.x, g.orientation.y, g.orientation.z, g.orientation.w))
                self.move_group.set_pose_target(g)

                # Attempt to execute the motion
                success = self.move_group.go(wait=True)
                if not success:
                    print("Path planning failed for this target.")
                else:
                    print("Successfully reached the target.")
        except Exception as e:
            print("Targets not reachable:", e)
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

if __name__ == "__main__":
    s = robot(nodename="pythonController",groupname="scara_planner")
    h = robot(nodename="pythonController",groupname="handler_planner")
    p = robot(nodename="pythonController",groupname="polisher_planner")

    movetoHome(s, h, p)

    s.getPoseAndPrint()
    #h.getPoseAndPrint()
    #p.getPoseAndPrint()
    #s.setTarget(0.631649, 0.31028, 0.983296,   -1.5707,-3.364,1.1061) #home scara
    s.setTarget(0.310, 0.212, 0.930,   -1.571,-2.504,-1.576) #pickup scara
    #h.setTarget(1.1, 0.6, 0.7,   1.57,0,-1.57) #pickup handler
    #p.setTarget(0.6, 0.6, 1.2,   -3.6, 0 ,-1.57) #pickup handler
    s.move()
    #h.move()
    #p.move()
    #h.getPoseAndPrint()
    
    del s, h, p