#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header

def move_robot(robot_namespace, joint_names, positions, time_from_start):
    # Initialize the node
    rospy.init_node('move_robots', anonymous=True)

    # Create a publisher for the robot's JointTrajectoryController
    pub = rospy.Publisher(f'/{robot_namespace}/joint_trajectory_controller/command', JointTrajectory, queue_size=10)
    
    # Wait until the publisher is connected
    rospy.sleep(1)

    # Create the JointTrajectory message
    traj = JointTrajectory()
    traj.header = Header()
    traj.header.stamp = rospy.Time.now()
    traj.joint_names = joint_names

    # Create a JointTrajectoryPoint with the desired joint positions
    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start = rospy.Duration(time_from_start)  # time in seconds
    
    # Append the point to the trajectory
    traj.points.append(point)
    
    # Publish the trajectory
    rospy.loginfo(f"Moving {robot_namespace} robot joints to: {positions}")
    pub.publish(traj)

def main():
    # Joint names for each robot
    polisher_joints = ['J1', 'J2', 'J3', 'J4', 'J5']
    handler_joints = ['J1', 'J2', 'J3', 'J4', 'J5']

    # Desired positions for each robot
    polisher_positions = [0.5, 0.5, 0.5, 0.5, 0.5]  # Example positions (in radians or your specific units)
    handler_positions = [0.2, 0.2, 0.2, 0.2, 0.2]  # Example positions (in radians or your specific units)

    # Move polisher robot
    move_robot('polisher', polisher_joints, polisher_positions, time_from_start=2.0)
    
    # Move handler robot
    move_robot('handler', handler_joints, handler_positions, time_from_start=2.0)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
