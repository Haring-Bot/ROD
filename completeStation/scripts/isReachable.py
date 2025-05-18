import moveit_commander
import sys
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

# Single pose to check (x, y, z, roll, pitch, yaw)
x, y, z = 0.310, 0.212, 0.930
roll, pitch, yaw = -1.571, -2.504, -1.576

moveit_commander.roscpp_initialize(sys.argv)
group = moveit_commander.MoveGroupCommander('scara_planner')

pose = Pose()
pose.position.x = x
pose.position.y = y
pose.position.z = z
q = quaternion_from_euler(roll, pitch, yaw)
pose.orientation.x = q[0]
pose.orientation.y = q[1]
pose.orientation.z = q[2]
pose.orientation.w = q[3]

ik_result = group.set_pose_target(pose)
print(f"IK solution: {ik_result} (x={x}, y={y}, z={z}, roll={roll}, pitch={pitch}, yaw={yaw})")
group.clear_pose_targets()