#!/usr/bin/env python3
"""
Publishes static scene objects (like tables) to the MoveIt planning scene.
This allows MoveIt to consider these objects during collision checking and motion planning.
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from geometry_msgs.msg import Pose
from shape_msgs.msg import Mesh, MeshTriangle
from geometry_msgs.msg import Point
import os
from ament_index_python.packages import get_package_share_directory


class ScenePublisher(Node):
    def __init__(self):
        super().__init__('scene_publisher')
        
        # Client for applying planning scene diffs synchronously (robust)
        self.apply_scene_client = self.create_client(ApplyPlanningScene, 'apply_planning_scene')

        self.get_logger().info('Waiting for apply_planning_scene service...')
        while not self.apply_scene_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info('...still waiting for apply_planning_scene')

        self.get_logger().info('Applying scene objects...')
        self.apply_table()
        
    def apply_table(self):
        """Apply the robot table as a collision object via service"""
        # Create planning scene message
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        
        # Create collision object for the table
        table = CollisionObject()
        table.header.frame_id = 'base_link'  # or 'world' depending on your setup
        table.id = 'robot_table'
        
        # Define the table pose (position and orientation)
        table_pose = Pose()
        table_pose.position.x = 0.0
        table_pose.position.y = 0.0
        table_pose.position.z = 0.0  
        table_pose.orientation.w = 1.0
        
        # Load mesh from package
        try:
            package_share = get_package_share_directory('hello_moveit')
            mesh_path = os.path.join(package_share, 'meshes', 'collision', 'robottable.stl')
            
            # For simplicity, we'll use a box primitive instead of loading the mesh
            # MeshTriangle and full mesh loading would require parsing the STL file
            # You can use a library like trimesh or pymesh for that
            
            # Alternative: Use a simple box as a placeholder
            from shape_msgs.msg import SolidPrimitive
            
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [1.0, 1.0, 0.1]  # width, depth, height in meters
            
            table.primitives.append(box)
            table.primitive_poses.append(table_pose)
            
            self.get_logger().info(f'Using box primitive for table (1.0m x 1.0m x 0.1m)')
            
        except Exception as e:
            self.get_logger().error(f'Error loading mesh: {e}')
            return
        
        # Add the table to the planning scene
        table.operation = CollisionObject.ADD
        planning_scene.world.collision_objects.append(table)
        
        # Call ApplyPlanningScene service synchronously
        req = ApplyPlanningScene.Request()
        req.scene = planning_scene

        future = self.apply_scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info(f'Applied planning scene (table at z={table_pose.position.z})')
        else:
            self.get_logger().error('Failed to apply planning scene. Will retry once in 2s...')
            # Retry once after short delay
            timer = self.create_timer(2.0, lambda: self._retry_apply(req))
            # Store timer to prevent it from being garbage collected
            self._retry_timer = timer

    def _retry_apply(self, req: ApplyPlanningScene.Request):
        self._retry_timer.cancel()
        future = self.apply_scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info('Re-applied planning scene successfully on retry')
        else:
            self.get_logger().error('Retry to apply planning scene also failed')


def main(args=None):
    rclpy.init(args=args)
    
    scene_publisher = ScenePublisher()
    
    # Keep the node alive for a bit to ensure message is sent
    rclpy.spin_once(scene_publisher, timeout_sec=2.0)
    
    scene_publisher.get_logger().info('Scene objects published successfully')
    scene_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
