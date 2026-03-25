#!/usr/bin/env python3
"""
Publishes a mesh collision object (STL/DAE) to the MoveIt planning scene
using the ApplyPlanningScene service (no moveit_commander dependency).
"""
import os
import sys
import struct
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from shape_msgs.msg import Mesh, MeshTriangle
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from ament_index_python.packages import get_package_share_directory


def load_stl_mesh(filepath):
    """Load binary STL file and return Mesh message."""
    mesh = Mesh()
    
    with open(filepath, 'rb') as f:
        # Skip STL header (80 bytes)
        f.read(80)
        # Read number of triangles (4 bytes, little-endian unsigned int)
        num_triangles = struct.unpack('<I', f.read(4))[0]
        
        for _ in range(num_triangles):
            # Skip normal vector (3 floats = 12 bytes)
            f.read(12)
            
            # Read 3 vertices (each is 3 floats = 12 bytes)
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
            
            # Skip attribute byte count (2 bytes)
            f.read(2)
    
    return mesh


class MeshScenePublisher(Node):
    def __init__(self):
        super().__init__('mesh_scene_publisher')
        
        # Declare parameters
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('mesh_path', '')
        self.declare_parameter('scale_x', 1.0)
        self.declare_parameter('scale_y', 1.0)
        self.declare_parameter('scale_z', 1.0)
        self.declare_parameter('pos_x', 0.0)
        self.declare_parameter('pos_y', 0.0)
        self.declare_parameter('pos_z', 0.0)
        self.declare_parameter('quat_x', 0.0)
        self.declare_parameter('quat_y', 0.0)
        self.declare_parameter('quat_z', 0.0)
        self.declare_parameter('quat_w', 1.0)
        
        # Get parameters
        frame_id = self.get_parameter('frame_id').value
        mesh_path = self.get_parameter('mesh_path').value
        scale_x = self.get_parameter('scale_x').value
        scale_y = self.get_parameter('scale_y').value
        scale_z = self.get_parameter('scale_z').value
        pos_x = self.get_parameter('pos_x').value
        pos_y = self.get_parameter('pos_y').value
        pos_z = self.get_parameter('pos_z').value
        quat_x = self.get_parameter('quat_x').value
        quat_y = self.get_parameter('quat_y').value
        quat_z = self.get_parameter('quat_z').value
        quat_w = self.get_parameter('quat_w').value
        
        # Resolve mesh path
        if not mesh_path:
            pkg_share = get_package_share_directory('hello_moveit')
            mesh_path = os.path.join(pkg_share, 'meshes', 'collision', 'robottable.stl')
        
        if mesh_path.startswith('package://'):
            pkg, rel = mesh_path[len('package://'):].split('/', 1)
            pkg_share = get_package_share_directory(pkg)
            mesh_path = os.path.join(pkg_share, rel)
        
        if not os.path.exists(mesh_path):
            self.get_logger().error(f'Mesh file does not exist: {mesh_path}')
            raise FileNotFoundError(mesh_path)
        
        self.get_logger().info(f'Loading mesh from: {mesh_path}')
        
        # Load mesh
        if mesh_path.lower().endswith('.stl'):
            mesh = load_stl_mesh(mesh_path)
        else:
            self.get_logger().error(f'Only STL meshes are supported currently. Got: {mesh_path}')
            raise ValueError('Unsupported mesh format')
        
        # Scale vertices
        for vertex in mesh.vertices:
            vertex.x *= scale_x
            vertex.y *= scale_y
            vertex.z *= scale_z
        
        self.get_logger().info(f'Loaded mesh with {len(mesh.triangles)} triangles, {len(mesh.vertices)} vertices')
        
        # Create collision object
        collision_obj = CollisionObject()
        collision_obj.header.frame_id = frame_id
        collision_obj.id = 'robot_table_mesh'
        collision_obj.operation = CollisionObject.ADD
        
        # Set pose
        pose = Pose()
        pose.position.x = pos_x
        pose.position.y = pos_y
        pose.position.z = pos_z
        pose.orientation.x = quat_x
        pose.orientation.y = quat_y
        pose.orientation.z = quat_z
        pose.orientation.w = quat_w
        
        collision_obj.meshes.append(mesh)
        collision_obj.mesh_poses.append(pose)
        
        # Publish to /collision_object topic (persistent approach)
        self.collision_pub = self.create_publisher(CollisionObject, '/collision_object', 10)
        
        # Wait a bit for move_group to be ready
        import time
        time.sleep(2.0)
        
        # First, remove the old box table if it exists
        remove_box = CollisionObject()
        remove_box.id = 'robot_table'
        remove_box.operation = CollisionObject.REMOVE
        self.collision_pub.publish(remove_box)
        self.get_logger().info('Removing old box table if present...')
        time.sleep(0.5)
        
        # Publish multiple times to ensure it's received
        self.get_logger().info(f'Publishing mesh collision object: {collision_obj.id}')
        for i in range(5):
            self.collision_pub.publish(collision_obj)
            time.sleep(0.5)
        
        self.get_logger().info(f'Mesh object "{collision_obj.id}" published to /collision_object')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MeshScenePublisher()
        # Keep spinning to maintain the publisher
        rclpy.spin(node)
    except Exception as e:
        print(f'Error: {e}', file=sys.stderr)
        rclpy.shutdown()
        return
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
