import moveit_commander
import sys
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import rospy
import numpy as np

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('check_scara_pose', anonymous=True)
group = moveit_commander.MoveGroupCommander('scara_planner')
robot = moveit_commander.RobotCommander()

# Get and print current pose
current_pose = group.get_current_pose().pose
print("Current pose:")
print(f"  Position: x={current_pose.position.x}, y={current_pose.position.y}, z={current_pose.position.z}")
q = current_pose.orientation
roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
print(f"  Orientation (rpy): roll={roll}, pitch={pitch}, yaw={yaw}")

# Print current joint values
current_joints = group.get_current_joint_values()
print("Current joint values:")
for name, val in zip(group.get_active_joints(), current_joints):
    print(f"  {name}: {val}")

# Print joint limits
print("Joint limits:")
for joint in group.get_active_joints():
    joint_obj = robot.get_joint(joint)
    print(f"  {joint}: lower={joint_obj.min_bound()}, upper={joint_obj.max_bound()}")
    if not (joint_obj.min_bound() <= group.get_current_joint_values()[group.get_active_joints().index(joint)] <= joint_obj.max_bound()):
        print(f"    ⚠️ Current value out of limits!")

# Check for singularity (Jacobian determinant)
def is_near_singularity(group):
    try:
        jac = group.get_jacobian_matrix(current_joints)
        det = np.linalg.det(jac @ jac.T)
        print(f"Jacobian determinant: {det}")
        if abs(det) < 1e-6:
            print("⚠️ Robot is near a singularity!")
    except Exception as e:
        print(f"Could not compute Jacobian: {e}")

is_near_singularity(group)

# Check if this pose is reachable by IK (with relaxed tolerance)
group.set_goal_tolerance(0.05)  # Relax tolerance
group.set_pose_reference_frame('scara_base_link')
ik_result = group.set_pose_target(current_pose)
print(f"IK solution for current pose: {ik_result}")
group.clear_pose_targets()

# If singularity or IK failed, nudge pose and retry
if not ik_result:
    print("\nTrying to nudge the pose away from singularity...")
    nudges = [
        (0.005, 0, 0),   # +X
        (-0.005, 0, 0),  # -X
        (0, 0.005, 0),   # +Y
        (0, -0.005, 0),  # -Y
        (0, 0, 0.005),   # +Z
        (0, 0, -0.005),  # -Z
    ]
    for dx, dy, dz in nudges:
        nudged_pose = Pose()
        nudged_pose.position.x = current_pose.position.x + dx
        nudged_pose.position.y = current_pose.position.y + dy
        nudged_pose.position.z = current_pose.position.z + dz
        nudged_pose.orientation = current_pose.orientation
        ik_result_nudge = group.set_pose_target(nudged_pose)
        print(f"Nudge (dx={dx}, dy={dy}, dz={dz}): IK solution: {ik_result_nudge}")
        group.clear_pose_targets()
        if ik_result_nudge:
            print("✅ IK succeeded after nudging the pose.")
            break
    else:
        print("❌ IK failed for all nudged poses.")
        # Try nudging J1 and J2 in joint space
        print("\nTrying to nudge J1 and J2 in joint space...")
        joint_nudges = [0.05, -0.05, 0.1, -0.1]
        orig_joints = group.get_current_joint_values()
        for j1_nudge in joint_nudges:
            for j2_nudge in joint_nudges:
                new_joints = orig_joints[:]
                new_joints[0] += j1_nudge  # J1
                new_joints[1] += j2_nudge  # J2
                group.set_joint_value_target(new_joints)
                try:
                    group.go(wait=True)
                    nudged_pose = group.get_current_pose().pose
                    ik_result_joint = group.set_pose_target(nudged_pose)
                    print(f"Joint nudge (J1={j1_nudge}, J2={j2_nudge}): IK solution: {ik_result_joint}")
                    group.clear_pose_targets()
                    if ik_result_joint:
                        print("✅ IK succeeded after nudging joints.")
                        break
                except Exception as e:
                    print(f"Joint nudge (J1={j1_nudge}, J2={j2_nudge}) failed: {e}")
            else:
                continue
            break
        else:
            print("❌ IK failed for all joint nudges.")

# Compare FK of current joints to current_pose (numerical closeness)
fk_pose = group.get_current_pose().pose  # Should match current_pose
def pose_distance(p1, p2):
    pos_dist = np.linalg.norm([
        p1.position.x - p2.position.x,
        p1.position.y - p2.position.y,
        p1.position.z - p2.position.z
    ])
    q1 = [p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w]
    q2 = [p2.orientation.x, p2.orientation.y, p2.orientation.z, p2.orientation.w]
    ori_dist = np.linalg.norm(np.array(q1) - np.array(q2))
    return pos_dist, ori_dist

pos_dist, ori_dist = pose_distance(current_pose, fk_pose)
print(f"Numerical FK check: position diff={pos_dist}, orientation diff={ori_dist}")
if pos_dist > 1e-4 or ori_dist > 1e-4:
    print("⚠️ FK and current pose differ more than expected (possible state sync issue)")

# Additional diagnostics for SCARA-specific singularities and configuration
print("\n--- Additional SCARA Diagnostics ---")

# 1. Warn if J1/J2 are aligned (typical SCARA singularity)
if len(current_joints) >= 2:
    j1 = current_joints[0]
    j2 = current_joints[1]
    diff = abs((j1 - j2) % (2 * np.pi))
    if diff < 1e-2 or abs(diff - np.pi) < 1e-2:
        print(f"⚠️ J1 and J2 are nearly aligned (diff={diff:.4f} rad): possible SCARA singularity!")
    else:
        print(f"J1/J2 alignment check: diff={diff:.4f} rad (OK)")
else:
    print("Not enough joints to check J1/J2 alignment.")

# 2. Print planning group base and tip link
try:
    print(f"Planning group base link: {group.get_pose_reference_frame()}")
    print(f"Planning group end effector link: {group.get_end_effector_link()}")
except Exception as e:
    print(f"Could not get base/tip link: {e}")

# 3. Print joint limits for all joints in the group (type/axis/origin not available in this API)
print("\nJoint limits (again):")
for joint_name in group.get_active_joints():
    joint_obj = robot.get_joint(joint_name)
    print(f"  {joint_name}: lower={joint_obj.min_bound()}, upper={joint_obj.max_bound()}")

# 4. Print link origins and check for zero-length or colinear vectors
print("\nLink origins (for consecutive joints): (not available in MoveIt Python API)")
print("  (To check link origins, parse the URDF directly with urdfpy or urdf_parser_py)")

# 5. Warn if any two consecutive joints have the same origin (not available)
print("  (To check for duplicate origins, parse the URDF directly)")

print("\n--- End of diagnostics ---\n")
