#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
import tf_transformations

class PSOParticle:
    def __init__(self, dimensions):
        self.position = np.random.uniform(0, 1, dimensions)
        self.velocity = np.random.uniform(-0.1, 0.1, dimensions)
        self.best_position = np.copy(self.position)
        self.best_score = float('inf')
        self.current_score = float('inf')

class PSOVisualNode(Node):
    def __init__(self):
        super().__init__('pso_visual_node')
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Fixed parameters
        self.num_particles = 20
        self.max_iterations = 50
        self.inertia_weight = 0.729
        self.cognitive_weight = 1.49445
        self.social_weight = 1.49445
        
        # Environment setup
        self.map_width = 10.0  # meters
        self.map_height = 10.0
        self.obstacles = [
            (2, 2, 6, 1),  # (x, y, width, height)
            (3, 6, 1, 3),
            (7, 3, 2, 5)
        ]
        
        # Start and goal positions
        self.start_pos = np.array([1.0, 1.0])  # [x, y] in meters
        self.goal_pos = np.array([8.0, 8.0])
        
        # Publisher for visualization
        self.markers_pub = self.create_publisher(MarkerArray, '/pso_markers', 10)
        
        # Publish static transforms
        self.publish_static_transforms()
        
        # Timer for visualization updates
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.current_iteration = 0
        self.particles = [PSOParticle(2) for _ in range(self.num_particles)]
        self.global_best_pos = None
        self.global_best_score = float('inf')
        self.algorithm_complete = False
        
        # Print initial setup
        self.get_logger().info("\n\n=== PSO Path Planning ===")
        self.get_logger().info(f"Start Position: {self.start_pos}")
        self.get_logger().info(f"Goal Position: {self.goal_pos}")
        self.get_logger().info(f"Number of Particles: {self.num_particles}")
        self.get_logger().info(f"Max Iterations: {self.max_iterations}")
        
    def publish_static_transforms(self):
        """Publish necessary TF frames for visualization"""
        # Map frame (our reference frame)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'pso_world'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)
        
    def is_obstacle(self, x, y):
        """Check if position (x,y) is inside an obstacle"""
        for ox, oy, ow, oh in self.obstacles:
            if ox <= x <= ox+ow and oy <= y <= oy+oh:
                return True
        return False
        
    def fitness_function(self, position):
        """Calculate fitness score for a position"""
        x, y = position
        
        # Check if position is in obstacle
        if self.is_obstacle(x, y):
            return float('inf')
            
        # Distance to goal (primary fitness)
        goal_dist = math.sqrt((x - self.goal_pos[0])**2 + (y - self.goal_pos[1])**2)
        
        # Distance from start (encourage exploration toward goal)
        start_dist = math.sqrt((x - self.start_pos[0])**2 + (y - self.start_pos[1])**2)
        
        # Combine factors (you can adjust weights)
        return goal_dist + 0.1 * start_dist
        
    def timer_callback(self):
        """Run one iteration of PSO each timer callback"""
        if self.algorithm_complete:
            return
            
        if self.current_iteration < self.max_iterations:
            self.get_logger().info(f"\n=== Iteration {self.current_iteration + 1} ===")
            
            # Evaluate all particles
            for i, particle in enumerate(self.particles):
                # Convert normalized [0,1] position to map coordinates
                map_x = particle.position[0] * self.map_width
                map_y = particle.position[1] * self.map_height
                
                # Calculate fitness
                particle.current_score = self.fitness_function((map_x, map_y))
                
                # Update personal best
                if particle.current_score < particle.best_score:
                    particle.best_score = particle.current_score
                    particle.best_position = particle.position.copy()
                    
                # Update global best
                if particle.current_score < self.global_best_score:
                    self.global_best_score = particle.current_score
                    self.global_best_pos = particle.position.copy()
                    
                # Print particle info
                self.get_logger().info(
                    f"Particle {i+1}: Pos=({map_x:.2f}, {map_y:.2f}) "
                    f"Fitness={particle.current_score:.2f} "
                    f"Best={particle.best_score:.2f}")
            
            # Print global best for this iteration
            if self.global_best_pos is not None:
                gb_x = self.global_best_pos[0] * self.map_width
                gb_y = self.global_best_pos[1] * self.map_height
                self.get_logger().info(
                    f"Global Best: Pos=({gb_x:.2f}, {gb_y:.2f}) "
                    f"Fitness={self.global_best_score:.2f}")
            
            # Update velocities and positions
            for particle in self.particles:
                # Inertia component
                particle.velocity *= self.inertia_weight
                
                # Cognitive component
                r1 = np.random.random(2)
                cognitive = self.cognitive_weight * r1 * (
                    particle.best_position - particle.position)
                
                # Social component
                r2 = np.random.random(2)
                social = self.social_weight * r2 * (
                    self.global_best_pos - particle.position) if self.global_best_pos is not None else 0
                    
                particle.velocity += cognitive + social
                particle.position += particle.velocity
                
                # Clamp to [0,1] range
                particle.position = np.clip(particle.position, 0, 1)
            
            self.current_iteration += 1
        else:
            # Algorithm complete
            self.algorithm_complete = True
            self.get_logger().info("\n=== Final Results ===")
            if self.global_best_pos is not None:
                final_x = self.global_best_pos[0] * self.map_width
                final_y = self.global_best_pos[1] * self.map_height
                self.get_logger().info(
                    f"Best Path Found: Pos=({final_x:.2f}, {final_y:.2f}) "
                    f"Fitness={self.global_best_score:.2f}")
            else:
                self.get_logger().info("No valid path found!")
        
        # Visualize particles and best path
        self.visualize_particles()
    
    def visualize_particles(self):
        """Create visualization markers for RViz"""
        marker_array = MarkerArray()
        
        # Clear all markers
        clear_marker = Marker()
        clear_marker.header.frame_id = "map"
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        # Add particles
        for i, particle in enumerate(self.particles):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i + 1  # Start from 1 since 0 is the clear marker
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 0.7
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            
            # Convert normalized position to map coordinates
            x = particle.position[0] * self.map_width
            y = particle.position[1] * self.map_height
            
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.1
            
            marker_array.markers.append(marker)
        
        # Add global best
        if self.global_best_pos is not None:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = len(self.particles) + 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            
            x = self.global_best_pos[0] * self.map_width
            y = self.global_best_pos[1] * self.map_height
            
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.2
            
            marker_array.markers.append(marker)
        
        # Add start and goal markers
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = len(self.particles) + 2
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.position.x = self.start_pos[0]
        marker.pose.position.y = self.start_pos[1]
        marker.pose.position.z = 0.3
        marker_array.markers.append(marker)
        
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = len(self.particles) + 3
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.position.x = self.goal_pos[0]
        marker.pose.position.y = self.goal_pos[1]
        marker.pose.position.z = 0.3
        marker_array.markers.append(marker)
        
        # Add obstacles
        for i, (x, y, w, h) in enumerate(self.obstacles):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = len(self.particles) + 4 + i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.scale.x = float(w)
            marker.scale.y = float(h)
            marker.scale.z = 0.1
            marker.color.a = 0.5
            marker.color.r = 0.5
            marker.color.g = 0.5
            marker.color.b = 0.5
            marker.pose.position.x = x + w/2
            marker.pose.position.y = y + h/2
            marker.pose.position.z = 0.05
            marker_array.markers.append(marker)
        
        self.markers_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = PSOVisualNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
