#!/usr/bin/env python3
"""
Example script showing how to dynamically add/remove mesh collision objects
during task execution using native ROS 2 messages.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from shape_msgs.msg import Mesh, MeshTriangle
from moveit_msgs.msg import CollisionObject
import struct
import time


class DynamicSceneManager(Node):
    """Manages dynamic addition/removal of collision objects in the planning scene."""
    
    def __init__(self):
        super().__init__('dynamic_scene_manager')
        
        # Publisher for collision objects
        self.collision_pub = self.create_publisher(CollisionObject, '/collision_object', 10)
        
        # Wait for move_group to be ready
        time.sleep(1.0)
        
        self.get_logger().info('Dynamic Scene Manager ready')
    
    def load_stl_mesh(self, filepath):
        """Load binary STL file and return Mesh message."""
        mesh = Mesh()
        
        with open(filepath, 'rb') as f:
            # Skip STL header (80 bytes)
            f.read(80)
            # Read number of triangles
            num_triangles = struct.unpack('<I', f.read(4))[0]
            
            for _ in range(num_triangles):
                # Skip normal vector
                f.read(12)
                
                # Read 3 vertices
                triangle = MeshTriangle()
                for i in range(3):
                    x, y, z = struct.unpack('<fff', f.read(12))
                    point = Point()
                    point.x = float(x)
                    point.y = float(y)
                    point.z = float(z)
                    mesh.vertices.append(point)
                    triangle.vertex_indices[i] = len(mesh.vertices) - 1
                
                mesh.triangles.append(triangle)
                
                # Skip attribute byte count
                f.read(2)
        
        return mesh
    
    def add_mesh_object(self, object_id, mesh_path, frame_id='base_link', 
                       pos=(0.0, 0.0, 0.0), quat=(0.0, 0.0, 0.0, 1.0),
                       scale=(1.0, 1.0, 1.0)):
        """
        Add a mesh collision object to the planning scene.
        
        Args:
            object_id: Unique name for the object
            mesh_path: Absolute path to STL file
            frame_id: Reference frame
            pos: Position (x, y, z)
            quat: Orientation quaternion (x, y, z, w)
            scale: Scale factors (x, y, z)
        """
        # Load and scale mesh
        mesh = self.load_stl_mesh(mesh_path)
        for vertex in mesh.vertices:
            vertex.x *= scale[0]
            vertex.y *= scale[1]
            vertex.z *= scale[2]
        
        # Create collision object
        collision_obj = CollisionObject()
        collision_obj.header.frame_id = frame_id
        collision_obj.header.stamp = self.get_clock().now().to_msg()
        collision_obj.id = object_id
        collision_obj.operation = CollisionObject.ADD
        
        # Set pose
        pose = Pose()
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        
        collision_obj.meshes.append(mesh)
        collision_obj.mesh_poses.append(pose)
        
        # Publish
        self.collision_pub.publish(collision_obj)
        self.get_logger().info(f'Added object: {object_id}')
        time.sleep(0.5)  # Wait for move_group to process
    
    def remove_object(self, object_id):
        """Remove an object from the planning scene."""
        collision_obj = CollisionObject()
        collision_obj.id = object_id
        collision_obj.operation = CollisionObject.REMOVE
        
        self.collision_pub.publish(collision_obj)
        self.get_logger().info(f'Removed object: {object_id}')
        time.sleep(0.5)
    
    def clear_all_objects(self):
        """Remove all objects from the planning scene."""
        collision_obj = CollisionObject()
        collision_obj.id = "all"
        collision_obj.operation = CollisionObject.REMOVE
        
        self.collision_pub.publish(collision_obj)
        self.get_logger().info('Cleared all objects')
        time.sleep(0.5)


def main():
    """Example usage demonstrating dynamic scene management."""
    rclpy.init()
    
    scene = DynamicSceneManager()
    
    # Example: Add table
    scene.add_mesh_object(
        object_id='robot_table',
        mesh_path='/home/shermin/ws_moveit/src/hello_moveit/meshes/collision/robottable.stl',
        pos=(0.5, 0.0, 0.0),
        scale=(1.0, 1.0, 1.0)
    )
    
    # Example: Add first object on table
    scene.add_mesh_object(
        object_id='object_1',
        mesh_path='/path/to/object.stl',  # Replace with your object path
        pos=(0.6, 0.0, 0.1),
        scale=(0.01, 0.01, 0.01)
    )
    
    scene.get_logger().info('Scene setup complete. Press Ctrl+C to exit.')
    
    try:
        rclpy.spin(scene)
    except KeyboardInterrupt:
        scene.get_logger().info('Shutting down...')
        # Optional: clean up on exit
        # scene.clear_all_objects()
    
    scene.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
