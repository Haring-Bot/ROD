#!/usr/bin/env python

import rospy
import moveit_commander
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive, Mesh
from geometry_msgs.msg import Pose, Point
import moveit_msgs.msg
import tf.transformations as tf_trans

class ObjectAttacher:
    def __init__(self):
        # Initialize publishers
        self.scene = moveit_commander.PlanningSceneInterface()
        self.attached_objects = []

    def load_mesh_as_collision_object(self, object_id, mesh_path, pose):
        """
        Load mesh as a collision object
        Args:
            object_id: unique identifier for the object
            mesh_path: path to the mesh file
            pose: pose of the object
        """
        # Add the mesh to the scene using the PlanningSceneInterface's add_mesh method
        self.scene.add_mesh(object_id, pose, mesh_path, (1, 1, 1))  # Last parameter is scale
        rospy.loginfo("Added collision object: {}".format(object_id))
        rospy.sleep(0.5)  # Give time for the scene to update

    def attach_object_to_robot(self, object_id, robot_link, touch_links=None):
        """
        Attach an object to the robot
        Args:
            object_id: The ID of the collision object
            robot_link: Link name to attach the object to
            touch_links: List of links that can touch the object
        """
        if touch_links is None:
            touch_links = []

        # Attach the object
        self.scene.attach_object(robot_link, object_id, touch_links=touch_links)
        self.attached_objects.append(object_id)
        rospy.loginfo("Attached object {} to {}".format(object_id, robot_link))
        rospy.sleep(0.5)  # Give time for the scene to update

    def detach_all_objects(self):
        """Detach all attached objects"""
        for obj_id in self.attached_objects:
            self.scene.remove_attached_object(obj_id)
        self.attached_objects = []
        rospy.loginfo("Detached all objects")
        rospy.sleep(0.5)  # Give time for the scene to update

    def clear_all_objects(self):
        """Remove all collision objects from the scene"""
        self.scene.remove_attached_objects([])
        self.scene.remove_world_object()
        self.attached_objects = []
        rospy.loginfo("Cleared all objects")
        rospy.sleep(0.5)  # Give time for the scene to update
