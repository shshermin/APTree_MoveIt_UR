#!/usr/bin/env python3
"""
Sequential pick task with better structure (MTC-inspired).
Chains tasks in a clear, organized way similar to MTC stages.
"""
import rclpy
from geometry_msgs.msg import Pose
import time

from move_to_task import MoveToTask
from pick_task import PickTask
from dynamic_scene_example import DynamicSceneManager


class StructuredPickTask:
    """
    Structured pick task that chains stages like MTC.
    Each stage is a separate method for clarity.
    """
    
    def __init__(self):
        self.scene = DynamicSceneManager()
        self.move_task = MoveToTask()
        self.pick_task = PickTask()
        
        # Task parameters
        self.object_id = 'target_object'
        self.object_position = (0.199, 0.597, 0.02)
        self.mesh_path = '/home/shermin/ws_moveit/src/hello_moveit/meshes/collision/object.stl'
        
    def stage_1_add_object(self):
        """Stage 1: Add object to planning scene."""
        self.move_task.get_logger().info('=== STAGE 1: Add Object ===')
        
        self.scene.add_mesh_object(
            object_id=self.object_id,
            mesh_path=self.mesh_path,
            pos=self.object_position,
            scale=(1.0, 1.0, 1.0)
        )
        time.sleep(1.0)
        return True
    
    def stage_2_open_gripper(self):
        """Stage 2: Open gripper."""
        self.move_task.get_logger().info('=== STAGE 2: Open Gripper ===')
        return self.pick_task.open_gripper()
    
    def stage_3_move_to_approach(self):
        """Stage 3: Move to approach pose (above object)."""
        self.move_task.get_logger().info('=== STAGE 3: Move to Approach ===')
        
        approach_pose = self._create_pose(z_offset=0.15)
        return self.move_task.move_to(
            approach_pose,
            velocity_scaling=0.2,
            acceleration_scaling=0.2,
            planning_time=10.0
        )
    
    def stage_4_approach_object(self):
        """Stage 4: Move down to grasp position."""
        self.move_task.get_logger().info('=== STAGE 4: Approach Object ===')
        
        grasp_pose = self._create_pose(z_offset=0.05)
        return self.move_task.move_to(
            grasp_pose,
            velocity_scaling=0.1,
            acceleration_scaling=0.1,
            planning_time=10.0
        )
    
    def stage_5_close_gripper(self):
        """Stage 5: Close gripper."""
        self.move_task.get_logger().info('=== STAGE 5: Close Gripper ===')
        return self.pick_task.close_gripper()
    
    def stage_6_attach_object(self):
        """Stage 6: Attach object to gripper."""
        self.move_task.get_logger().info('=== STAGE 6: Attach Object ===')
        
        time.sleep(1.0)  # Wait for gripper to close
        return self.pick_task.attach_object(self.object_id)
    
    def stage_7_lift_object(self):
        """Stage 7: Lift object upward."""
        self.move_task.get_logger().info('=== STAGE 7: Lift Object ===')
        
        time.sleep(2.0)  # Wait for planning scene update
        
        lift_pose = self._create_pose(z_offset=0.50)
        return self.move_task.move_to(
            lift_pose,
            velocity_scaling=0.1,
            acceleration_scaling=0.1,
            planning_time=10.0
        )
    
    def _create_pose(self, z_offset=0.0):
        """Helper: Create pose above object."""
        pose = Pose()
        pose.position.x = self.object_position[0]
        pose.position.y = self.object_position[1]
        pose.position.z = self.object_position[2] + z_offset
        
        # Orientation: pointing down
        pose.orientation.x = 0.0
        pose.orientation.y = 1.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0
        
        return pose
    
    def execute(self):
        """Execute all stages in sequence."""
        self.move_task.get_logger().info('========================================')
        self.move_task.get_logger().info('  STRUCTURED PICK TASK (7 STAGES)')
        self.move_task.get_logger().info('========================================')
        
        stages = [
            self.stage_1_add_object,
            self.stage_2_open_gripper,
            self.stage_3_move_to_approach,
            self.stage_4_approach_object,
            self.stage_5_close_gripper,
            self.stage_6_attach_object,
            self.stage_7_lift_object,
        ]
        
        for i, stage in enumerate(stages, 1):
            if not stage():
                self.move_task.get_logger().error(f'Stage {i} failed: {stage.__name__}')
                return False
            time.sleep(0.5)
        
        self.move_task.get_logger().info('========================================')
        self.move_task.get_logger().info('  ALL STAGES COMPLETED SUCCESSFULLY!')
        self.move_task.get_logger().info('========================================')
        return True
    
    def cleanup(self):
        """Cleanup resources."""
        self.scene.destroy_node()
        self.move_task.destroy_node()
        self.pick_task.destroy_node()


def main():
    """Main function: Execute structured pick task."""
    rclpy.init()
    
    task = StructuredPickTask()
    
    success = task.execute()
    
    if not success:
        task.move_task.get_logger().error('Task failed!')
    
    task.cleanup()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
