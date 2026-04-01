#!/usr/bin/env python3
"""
Example script showing how to dynamically add/remove mesh collision objects
during task execution using native ROS 2 messages.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from shape_msgs.msg import Mesh, MeshTriangle
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
import struct
import time
import os


# Default directory for object meshes (collision)
MESH_DIR = '/home/shermin/ws_moveit/src/hello_moveit/meshes/collision/dynamic_objects'


class DynamicSceneManager(Node):
    """Manages dynamic addition/removal of collision objects in the planning scene."""
    
    def __init__(self):
        super().__init__('dynamic_scene_manager')
        
        # Publisher for collision objects
        self.collision_pub = self.create_publisher(CollisionObject, '/collision_object', 10)
        self.attached_pub = self.create_publisher(AttachedCollisionObject, '/attached_collision_object', 10)
        
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
    
    def load_obj_mesh(self, filepath, object_name=None):
        """
        Load Wavefront .obj file and return Mesh message.
        
        If object_name is given, only the geometry belonging to that named
        object (defined by 'o <name>' lines in the OBJ) is returned.
        If object_name is None, the entire file is loaded as one mesh.
        """
        all_vertices = []   # global vertex list (shared across all objects)
        # Per-object face collection: object_name -> list of face index lists
        objects = {}
        current_obj = None
        
        with open(filepath, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if not parts:
                    continue
                if parts[0] == 'o' or parts[0] == 'g':
                    current_obj = parts[1] if len(parts) > 1 else None
                    if current_obj and current_obj not in objects:
                        objects[current_obj] = []
                elif parts[0] == 'v':
                    point = Point()
                    point.x = float(parts[1])
                    point.y = float(parts[2])
                    point.z = float(parts[3])
                    all_vertices.append(point)
                elif parts[0] == 'f':
                    face_indices = []
                    for p in parts[1:]:
                        idx = int(p.split('/')[0]) - 1  # convert to 0-indexed
                        face_indices.append(idx)
                    if current_obj is not None:
                        if current_obj not in objects:
                            objects[current_obj] = []
                        objects[current_obj].append(face_indices)
                    else:
                        # No object defined yet — store under None key
                        if None not in objects:
                            objects[None] = []
                        objects[None].append(face_indices)
        
        # If a specific object is requested, extract only its faces
        if object_name is not None:
            if object_name not in objects:
                available = [k for k in objects.keys() if k is not None]
                raise KeyError(
                    f"Object '{object_name}' not found in {filepath}. "
                    f"Available objects: {available}"
                )
            face_lists = objects[object_name]
        else:
            # All faces from all objects
            face_lists = []
            for faces in objects.values():
                face_lists.extend(faces)
        
        # Collect only the vertices used by the selected faces and re-index
        used_indices = set()
        for face in face_lists:
            used_indices.update(face)
        
        # Build re-index map: old global index -> new compact index
        sorted_indices = sorted(used_indices)
        remap = {old: new for new, old in enumerate(sorted_indices)}
        
        mesh = Mesh()
        mesh.vertices = [all_vertices[i] for i in sorted_indices]
        
        for face in face_lists:
            remapped = [remap[i] for i in face]
            for i in range(1, len(remapped) - 1):
                triangle = MeshTriangle()
                triangle.vertex_indices[0] = remapped[0]
                triangle.vertex_indices[1] = remapped[i]
                triangle.vertex_indices[2] = remapped[i + 1]
                mesh.triangles.append(triangle)
        
        return mesh
    
    def list_obj_objects(self, filepath):
        """List all named objects ('o' lines) in a .obj file."""
        names = []
        with open(filepath, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if parts and (parts[0] == 'o' or parts[0] == 'g') and len(parts) > 1:
                    names.append(parts[1])
        return names
    
    def load_mesh(self, filepath, object_name=None):
        """Load a mesh file (.stl or .obj) and return Mesh message.
        
        For .obj files, if object_name is specified, only that named object
        is extracted from the file.
        """
        ext = os.path.splitext(filepath)[1].lower()
        if ext == '.stl':
            return self.load_stl_mesh(filepath)
        elif ext == '.obj':
            return self.load_obj_mesh(filepath, object_name=object_name)
        else:
            raise ValueError(f'Unsupported mesh format: {ext} (use .stl or .obj)')
    
    def resolve_mesh_path(self, mesh_name):
        """
        Resolve a mesh name to a full file path.
        Accepts:
          - Full absolute path (returned as-is)
          - Filename with extension (looked up in MESH_DIR)
          - Name without extension (searches for .stl then .obj in MESH_DIR)
        """
        if os.path.isabs(mesh_name) and os.path.isfile(mesh_name):
            return mesh_name
        
        # Try as filename in MESH_DIR
        candidate = os.path.join(MESH_DIR, mesh_name)
        if os.path.isfile(candidate):
            return candidate
        
        # Try adding extensions
        for ext in ['.stl', '.obj']:
            candidate = os.path.join(MESH_DIR, mesh_name + ext)
            if os.path.isfile(candidate):
                return candidate
        
        raise FileNotFoundError(
            f"Mesh '{mesh_name}' not found in {MESH_DIR}. "
            f"Available: {[f for f in os.listdir(MESH_DIR) if f.endswith(('.stl', '.obj'))]}"
        )
    
    def add_mesh_object(self, object_id, mesh_path, frame_id='base_link', 
                       pos=(0.0, 0.0, 0.0), quat=(0.0, 0.0, 0.0, 1.0),
                       scale=(1.0, 1.0, 1.0), obj_name=None):
        """
        Add a mesh collision object to the planning scene.
        
        Args:
            object_id: Unique name for the object
            mesh_path: Path or name of mesh file (.stl/.obj). Can be absolute path,
                       filename, or name without extension (resolved from MESH_DIR)
            frame_id: Reference frame
            pos: Position (x, y, z)
            quat: Orientation quaternion (x, y, z, w)
            scale: Scale factors (x, y, z)
            obj_name: For multi-object .obj files, the name of the object to extract
        """
        # Load and scale mesh
        resolved_path = self.resolve_mesh_path(mesh_path)
        mesh = self.load_mesh(resolved_path, object_name=obj_name)
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
    
    def attach_mesh_object(self, object_id, mesh_path, link_name='tool0',
                           pos=(0.0, 0.0, 0.0), quat=(0.0, 0.0, 0.0, 1.0),
                           scale=(1.0, 1.0, 1.0), touch_links=None, obj_name=None):
        """
        Attach a mesh collision object to a robot link so it moves with the robot.
        
        Args:
            object_id: Unique name for the object
            mesh_path: Path or name of mesh file (.stl/.obj). Can be absolute path,
                       filename, or name without extension (resolved from MESH_DIR)
            link_name: Robot link to attach to
            pos: Position relative to the link
            quat: Orientation quaternion (x, y, z, w) relative to the link
            scale: Scale factors (x, y, z)
            touch_links: Links allowed to touch the object (no collision checked)
            obj_name: For multi-object .obj files, the name of the object to extract
        """
        resolved_path = self.resolve_mesh_path(mesh_path)
        mesh = self.load_mesh(resolved_path, object_name=obj_name)
        for vertex in mesh.vertices:
            vertex.x *= scale[0]
            vertex.y *= scale[1]
            vertex.z *= scale[2]
        
        collision_obj = CollisionObject()
        collision_obj.header.frame_id = link_name
        collision_obj.header.stamp = self.get_clock().now().to_msg()
        collision_obj.id = object_id
        collision_obj.operation = CollisionObject.ADD
        
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
        
        attached_obj = AttachedCollisionObject()
        attached_obj.link_name = link_name
        attached_obj.object = collision_obj
        if touch_links:
            attached_obj.touch_links = touch_links
        
        self.attached_pub.publish(attached_obj)
        self.get_logger().info(f'Attached object: {object_id} to {link_name}')
        time.sleep(0.5)
    
    def detach_object(self, object_id, link_name='tool0'):
        """
        Detach an object from the robot and leave it in the scene as a
        static collision object at its current world position.
        """
        attached_obj = AttachedCollisionObject()
        attached_obj.link_name = link_name
        attached_obj.object.id = object_id
        attached_obj.object.operation = CollisionObject.REMOVE
        
        self.attached_pub.publish(attached_obj)
        self.get_logger().info(f'Detached object: {object_id} from {link_name}')
        time.sleep(0.5)
    
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
    
    # Note: Table is published by the launch file (mesh_scene_publisher_node)
    # This script is for dynamic objects only
    
    # Example: Add first object on table
    scene.add_mesh_object(
        object_id='object_1',
        mesh_path='/home/shermin/ws_moveit/src/hello_moveit/meshes/collision/object.stl',
        pos=(0.0, 0.0, 0.0),  # On the table surface
        scale=(1.0, 1.0, 1.0)
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
