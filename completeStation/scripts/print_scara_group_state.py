#!/usr/bin/env python3
import moveit_commander
import sys
import rospy

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('print_scara_group_state', anonymous=True)
group = moveit_commander.MoveGroupCommander('scara_planner')

joint_names = group.get_active_joints()
joint_values = group.get_current_joint_values()

print('    <group_state name="current" group="scara_planner">')
for name, value in zip(joint_names, joint_values):
    print(f'        <joint name="{name}" value="{value}"/>')
print('    </group_state>')
