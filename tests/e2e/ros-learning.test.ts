import { describe, test, expect, beforeEach, afterEach, vi } from 'vitest';
import { parseROSCommand } from '../../src/utils/ros';

describe('ROS Learning Center E2E Tests', () => {
  let page: any;

  test.beforeEach(async ({ page: testPage }) => {
    page = testPage;
    await page.goto('http://localhost:3000');
    await page.waitForLoadState('networkidle');
  });

  test.describe('ROS2 Basics Course', () => {
    test('should complete ROS2 basics introduction', async () => {
      // Navigate to ROS learning center
      await page.click('[data-testid="ros-learning-button"]');
      await page.waitForURL('**/ros-learning');
      
      // Select ROS2 basics course
      await page.click('text=ROS2 Basics');
      await page.waitForURL('**/ros-learning/basics');
      
      // Verify course content
      await expect(page.locator('h1')).toContainText('ROS2 Basics');
      await expect(page.locator('text=What is ROS2?')).toBeVisible();
      
      // Complete introduction section
      await page.click('text=Start Introduction');
      await expect(page.locator('[data-testid="course-progress"]')).toContainText('10%');
    });

    test('should complete nodes and topics exercise', async () => {
      // Navigate to ROS2 basics course
      await page.click('[data-testid="ros-learning-button"]');
      await page.click('text=ROS2 Basics');
      
      // Navigate to nodes and topics section
      await page.click('text=Nodes and Topics');
      
      // Read theory content
      await expect(page.locator('text=ROS nodes are executable programs')).toBeVisible();
      
      // Start interactive exercise
      await page.click('text=Start Exercise');
      
      // Exercise 1: Create a simple node
      await page.click('[data-testid="create-node-button"]');
      await page.waitForSelector('[data-testid="code-editor"]');
      
      // Type node code
      await page.type('[data-testid="code-editor"]', `
import rclpy
from rclpy.node import Node

class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')
        self.get_logger().info('Hello from NAVΛ Studio!')

def main():
    rclpy.init()
    node = SimpleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
      `);
      
      // Run the node
      await page.click('[data-testid="run-code-button"]');
      
      // Verify output
      await expect(page.locator('[data-testid="execution-output"]')).toContainText('Hello from NAVΛ Studio!');
      
      // Complete exercise
      await page.click('[data-testid="complete-exercise-button"]');
      await expect(page.locator('[data-testid="course-progress"]')).toContainText('25%');
    });

    test('should complete services and parameters exercise', async () => {
      // Navigate to services and parameters section
      await page.click('[data-testid="ros-learning-button"]');
      await page.click('text=ROS2 Basics');
      await page.click('text=Services and Parameters');
      
      // Read theory content
      await expect(page.locator('text=ROS services provide request-response communication')).toBeVisible();
      
      // Start interactive exercise
      await page.click('text=Start Exercise');
      
      // Exercise: Create a service server
      await page.click('[data-testid="create-service-button"]');
      await page.waitForSelector('[data-testid="service-editor"]');
      
      // Define service
      await page.type('[data-testid="service-editor"]', `
from example_interfaces.srv import AddTwoInts

class AddTwoIntsService(Node):
    def __init__(self):
        super().__init__('add_two_ints_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
    
    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: {request.a} + {request.b}')
        return response
      `);
      
      // Test the service
      await page.click('[data-testid="test-service-button"]');
      await page.type('[data-testid="test-input"]', '{"a": 5, "b": 3}');
      await page.click('[data-testid="send-request-button"]');
      
      // Verify response
      await expect(page.locator('[data-testid="service-response"]')).toContainText('8');
      
      // Complete exercise
      await page.click('[data-testid="complete-exercise-button"]');
      await expect(page.locator('[data-testid="course-progress"]')).toContainText('50%');
    });

    test('should complete final assessment', async () => {
      // Navigate to assessment section
      await page.click('[data-testid="ros-learning-button"]');
      await page.click('text=ROS2 Basics');
      await page.click('text=Final Assessment');
      
      // Question 1: What is a ROS node?
      await page.click('[data-testid="question-1"]');
      await page.click('text=An executable program that communicates with other nodes');
      
      // Question 2: What is the purpose of ROS topics?
      await page.click('[data-testid="question-2"]');
      await page.click('text=To provide publish-subscribe communication between nodes');
      
      // Question 3: How do ROS services work?
      await page.click('[data-testid="question-3"]');
      await page.click('text=They provide request-response communication pattern');
      
      // Submit assessment
      await page.click('[data-testid="submit-assessment-button"]');
      
      // Verify results
      await expect(page.locator('[data-testid="assessment-results"]')).toContainText('Congratulations!');
      await expect(page.locator('[data-testid="course-progress"]')).toContainText('100%');
      
      // Download certificate
      await page.click('[data-testid="download-certificate-button"]');
      await expect(page.locator('[data-testid="certificate-downloaded"]')).toBeVisible();
    });
  });

  test.describe('Advanced Navigation Course', () => {
    test('should complete path planning algorithms section', async () => {
      // Navigate to advanced navigation course
      await page.click('[data-testid="ros-learning-button"]');
      await page.click('text=Advanced Navigation');
      await page.waitForURL('**/ros-learning/advanced');
      
      // Verify course content
      await expect(page.locator('h1')).toContainText('Advanced Navigation');
      await expect(page.locator('text=Path Planning Algorithms')).toBeVisible();
      
      // Navigate to A* algorithm section
      await page.click('text=A* Algorithm');
      
      // Read theory content
      await expect(page.locator('text=A* is an informed search algorithm')).toBeVisible();
      
      // Start interactive implementation
      await page.click('text=Implement A*');
      
      // Implement A* algorithm
      await page.type('[data-testid="algorithm-editor"]', `
import heapq
from typing import List, Tuple

def astar(start: Tuple[int, int], goal: Tuple[int, int], obstacles: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
    """A* pathfinding algorithm implementation"""
    
    def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def get_neighbors(node: Tuple[int, int]) -> List[Tuple[int, int]]:
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]
        neighbors = []
        for dx, dy in directions:
            neighbor = (node[0] + dx, node[1] + dy)
            if neighbor not in obstacles:
                neighbors.append(neighbor)
        return neighbors
    
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    
    while open_set:
        current = heapq.heappop(open_set)[1]
        
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path
        
        for neighbor in get_neighbors(current):
            tentative_g_score = g_score[current] + 1
            
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    return []  # No path found
      `);
      
      // Test the algorithm
      await page.click('[data-testid="test-algorithm-button"]');
      await page.type('[data-testid="test-start"]', '(0, 0)');
      await page.type('[data-testid="test-goal"]', '(5, 5)');
      await page.type('[data-testid="test-obstacles"]', '[(2, 2), (3, 3)]');
      await page.click('[data-testid="run-test-button"]');
      
      // Verify results
      await expect(page.locator('[data-testid="algorithm-result"]')).toContainText('Path found');
      await expect(page.locator('[data-testid="path-visualization"]')).toBeVisible();
      
      // Complete section
      await page.click('[data-testid="complete-section-button"]');
      await expect(page.locator('[data-testid="course-progress"]')).toContainText('20%');
    });

    test('should complete SLAM implementation section', async () => {
      // Navigate to SLAM section
      await page.click('[data-testid="ros-learning-button"]');
      await page.click('text=Advanced Navigation');
      await page.click('text=SLAM Implementation');
      
      // Read theory content
      await expect(page.locator('text=Simultaneous Localization and Mapping')).toBeVisible();
      
      // Start interactive implementation
      await page.click('text=Implement SLAM');
      
      // Implement EKF SLAM
      await page.type('[data-testid="slam-editor"]', `
import numpy as np
from typing import List, Tuple

class EKF_SLAM:
    def __init__(self, n_landmarks: int):
        self.n_landmarks = n_landmarks
        self.state_dim = 3 + 2 * n_landmarks  # robot pose + landmark positions
        self.state = np.zeros(self.state_dim)
        self.covariance = np.eye(self.state_dim) * 1000
        
    def predict(self, control: np.ndarray, dt: float):
        """Prediction step of EKF SLAM"""
        # Robot motion model
        v, omega = control
        theta = self.state[2]
        
        if abs(omega) < 1e-6:
            # Straight line motion
            self.state[0] += v * dt * np.cos(theta)
            self.state[1] += v * dt * np.sin(theta)
        else:
            # Circular motion
            self.state[0] += v / omega * (np.sin(theta + omega * dt) - np.sin(theta))
            self.state[1] += v / omega * (np.cos(theta) - np.cos(theta + omega * dt))
            self.state[2] += omega * dt
        
        # Update covariance
        # Jacobian of motion model
        G = np.eye(self.state_dim)
        G[0, 2] = -v * dt * np.sin(theta)
        G[1, 2] = v * dt * np.cos(theta)
        
        # Process noise
        R = np.diag([0.1, 0.1, 0.01])
        self.covariance[:3, :3] = G[:3, :3] @ self.covariance[:3, :3] @ G[:3, :3].T + R
    
    def update(self, measurements: List[Tuple[int, float, float]]):
        """Update step of EKF SLAM"""
        for landmark_id, distance, bearing in measurements:
            # Calculate expected measurement
            landmark_idx = 3 + 2 * landmark_id
            landmark_x = self.state[landmark_idx]
            landmark_y = self.state[landmark_idx + 1]
            
            robot_x, robot_y, robot_theta = self.state[0], self.state[1], self.state[2]
            
            dx = landmark_x - robot_x
            dy = landmark_y - robot_y
            
            expected_distance = np.sqrt(dx**2 + dy**2)
            expected_bearing = np.arctan2(dy, dx) - robot_theta
            
            # Measurement model Jacobian
            H = np.zeros((2, self.state_dim))
            H[0, 0] = -dx / expected_distance
            H[0, 1] = -dy / expected_distance
            H[0, 2] = 0
            H[0, landmark_idx] = dx / expected_distance
            H[0, landmark_idx + 1] = dy / expected_distance
            
            H[1, 0] = dy / (expected_distance**2)
            H[1, 1] = -dx / (expected_distance**2)
            H[1, 2] = -1
            H[1, landmark_idx] = -dy / (expected_distance**2)
            H[1, landmark_idx + 1] = dx / (expected_distance**2)
            
            # Innovation
            z = np.array([distance, bearing])
            z_expected = np.array([expected_distance, expected_bearing])
            innovation = z - z_expected
            
            # Innovation covariance
            S = H @ self.covariance @ H.T + np.diag([0.1, 0.05])
            
            # Kalman gain
            K = self.covariance @ H.T @ np.linalg.inv(S)
            
            # Update state and covariance
            self.state += K @ innovation
            self.covariance = (np.eye(self.state_dim) - K @ H) @ self.covariance
      `);
      
      // Test SLAM implementation
      await page.click('[data-testid="test-slam-button"]');
      await page.click('[data-testid="run-slam-simulation"]');
      
      // Verify results
      await expect(page.locator('[data-testid="slam-result"]')).toContainText('SLAM completed');
      await expect(page.locator('[data-testid="map-visualization"]')).toBeVisible();
      await expect(page.locator('[data-testid="trajectory-plot"]')).toBeVisible();
      
      // Complete section
      await page.click('[data-testid="complete-section-button"]');
      await expect(page.locator('[data-testid="course-progress"]')).toContainText('40%');
    });

    test('should complete path optimization section', async () => {
      // Navigate to path optimization section
      await page.click('[data-testid="ros-learning-button"]');
      await page.click('text=Advanced Navigation');
      await page.click('text=Path Optimization');
      
      // Read theory content
      await expect(page.locator('text=Optimize navigation paths for efficiency')).toBeVisible();
      
      // Start interactive implementation
      await page.click('text=Implement Optimization');
      
      // Implement gradient descent optimization
      await page.type('[data-testid="optimization-editor"]', `
import numpy as np
from typing import List, Tuple, Callable

def gradient_descent_optimization(
    path: List[Tuple[float, float]], 
    cost_function: Callable[[Tuple[float, float]], float],
    gradient_function: Callable[[Tuple[float, float]], Tuple[float, float]],
    learning_rate: float = 0.01,
    max_iterations: int = 1000,
    tolerance: float = 1e-6
) -> List[Tuple[float, float]]:
    """Optimize a path using gradient descent"""
    
    optimized_path = path.copy()
    
    for iteration in range(max_iterations):
        total_cost = 0
        gradients = []
        
        # Calculate cost and gradients for each point
        for i, point in enumerate(optimized_path):
            cost = cost_function(point)
            total_cost += cost
            gradient = gradient_function(point)
            gradients.append(gradient)
        
        # Update path points
        new_path = []
        for i, (point, gradient) in enumerate(zip(optimized_path, gradients)):
            # Smooth gradient with neighbors
            if i > 0 and i < len(optimized_path) - 1:
                smooth_gradient = (
                    0.5 * gradient[0] + 0.25 * gradients[i-1][0] + 0.25 * gradients[i+1][0],
                    0.5 * gradient[1] + 0.25 * gradients[i-1][1] + 0.25 * gradients[i+1][1]
                )
            else:
                smooth_gradient = gradient
            
            new_point = (
                point[0] - learning_rate * smooth_gradient[0],
                point[1] - learning_rate * smooth_gradient[1]
            )
            new_path.append(new_point)
        
        optimized_path = new_path
        
        # Check convergence
        if total_cost < tolerance:
            break
    
    return optimized_path

def optimize_navigation_path(
    start: Tuple[float, float],
    goal: Tuple[float, float],
    obstacles: List[Tuple[float, float, float]],  # (x, y, radius)
    num_waypoints: int = 20
) -> List[Tuple[float, float]]:
    """Optimize a navigation path avoiding obstacles"""
    
    def cost_function(point: Tuple[float, float]) -> float:
        """Calculate cost based on obstacle proximity"""
        cost = 0
        for ox, oy, radius in obstacles:
            distance = np.sqrt((point[0] - ox)**2 + (point[1] - oy)**2)
            if distance < radius:
                return float('inf')  # Inside obstacle
            elif distance < radius + 2.0:
                # Cost increases as we get closer to obstacles
                cost += (radius + 2.0 - distance) / (radius + 2.0)
        return cost
    
    def gradient_function(point: Tuple[float, float]) -> Tuple[float, float]:
        """Calculate gradient of cost function"""
        gradient_x = 0
        gradient_y = 0
        epsilon = 0.01
        
        # Numerical gradient
        cost_x_plus = cost_function((point[0] + epsilon, point[1]))
        cost_x_minus = cost_function((point[0] - epsilon, point[1]))
        cost_y_plus = cost_function((point[0], point[1] + epsilon))
        cost_y_minus = cost_function((point[0], point[1] - epsilon))
        
        if cost_x_plus != float('inf') and cost_x_minus != float('inf'):
            gradient_x = (cost_x_plus - cost_x_minus) / (2 * epsilon)
        
        if cost_y_plus != float('inf') and cost_y_minus != float('inf'):
            gradient_y = (cost_y_plus - cost_y_minus) / (2 * epsilon)
        
        return (gradient_x, gradient_y)
    
    # Initialize path with straight line
    path = []
    for i in range(num_waypoints):
        t = i / (num_waypoints - 1)
        x = start[0] + t * (goal[0] - start[0])
        y = start[1] + t * (goal[1] - start[1])
        path.append((x, y))
    
    # Optimize path
    optimized_path = gradient_descent_optimization(
        path, cost_function, gradient_function
    )
    
    return optimized_path
      `);
      
      // Test optimization
      await page.click('[data-testid="test-optimization-button"]');
      await page.click('[data-testid="run-optimization"]');
      
      // Verify results
      await expect(page.locator('[data-testid="optimization-result"]')).toContainText('Path optimized');
      await expect(page.locator('[data-testid="before-after-plot"]')).toBeVisible();
      await expect(page.locator('[data-testid="cost-reduction-plot"]')).toBeVisible();
      
      // Complete section
      await page.click('[data-testid="complete-section-button"]');
      await expect(page.locator('[data-testid="course-progress"]')).toContainText('60%');
    });

    test('should complete final project', async () => {
      // Navigate to final project
      await page.click('[data-testid="ros-learning-button"]');
      await page.click('text=Advanced Navigation');
      await page.click('text=Final Project');
      
      // Read project requirements
      await expect(page.locator('text=Implement a complete navigation system')).toBeVisible();
      
      // Start project implementation
      await page.click('[data-testid="start-project-button"]');
      
      // Implement complete navigation system
      await page.type('[data-testid="project-editor"]', `
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Bool
import numpy as np
from typing import List, Tuple, Optional

class AdvancedNavigationSystem(Node):
    def __init__(self):
        super().__init__('advanced_navigation_system')
        
        # Publishers
        self.path_publisher = self.create_publisher(Path, 'planned_path', 10)
        self.goal_publisher = self.create_publisher(PoseStamped, 'current_goal', 10)
        self.navigation_active_publisher = self.create_publisher(Bool, 'navigation_active', 10)
        
        # Subscribers
        self.goal_subscriber = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_callback, 10
        )
        self.map_subscriber = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10
        )
        self.pose_subscriber = self.create_subscription(
            PoseStamped, 'current_pose', self.pose_callback, 10
        )
        
        # Internal state
        self.current_pose = None
        self.current_goal = None
        self.current_map = None
        self.planned_path = None
        self.navigation_active = False
        
        # Navigation parameters
        self.declare_parameter('planning_algorithm', 'astar')
        self.declare_parameter('optimization_iterations', 100)
        self.declare_parameter('safety_distance', 0.5)
        
        self.get_logger().info('Advanced Navigation System initialized')
    
    def pose_callback(self, msg: PoseStamped):
        """Handle current pose updates"""
        self.current_pose = msg
        
        if self.navigation_active and self.planned_path:
            # Check if goal reached
            if self.is_goal_reached():
                self.stop_navigation()
            else:
                # Update path following
                self.update_path_following()
    
    def goal_callback(self, msg: PoseStamped):
        """Handle new navigation goals"""
        self.current_goal = msg
        self.get_logger().info(f'New goal received: ({msg.pose.position.x}, {msg.pose.position.y})')
        
        if self.current_map and self.current_pose:
            self.plan_path()
    
    def map_callback(self, msg: OccupancyGrid):
        """Handle map updates"""
        self.current_map = msg
        self.get_logger().info('Map updated')
        
        # Replan if navigation is active
        if self.navigation_active and self.current_goal:
            self.plan_path()
    
    def plan_path(self):
        """Plan path from current pose to goal"""
        if not self.current_pose or not self.current_goal or not self.current_map:
            self.get_logger().warn('Cannot plan path: missing required data')
            return
        
        start = (self.current_pose.pose.position.x, self.current_pose.pose.position.y)
        goal = (self.current_goal.pose.position.x, self.current_goal.pose.position.y)
        
        # Convert occupancy grid to obstacle list
        obstacles = self.get_obstacles_from_map()
        
        # Plan path using selected algorithm
        algorithm = self.get_parameter('planning_algorithm').value
        
        if algorithm == 'astar':
            path = self.plan_astar(start, goal, obstacles)
        elif algorithm == 'rrt':
            path = self.plan_rrt(start, goal, obstacles)
        elif algorithm == 'gradient_descent':
            path = self.plan_gradient_descent(start, goal, obstacles)
        else:
            self.get_logger().error(f'Unknown planning algorithm: {algorithm}')
            return
        
        if path:
            self.planned_path = self.create_path_message(path)
            self.path_publisher.publish(self.planned_path)
            self.start_navigation()
            self.get_logger().info(f'Path planned successfully with {len(path)} waypoints')
        else:
            self.get_logger().warn('Failed to plan path')
    
    def plan_astar(self, start: Tuple[float, float], goal: Tuple[float, float], 
                   obstacles: List[Tuple[float, float, float]]) -> List[Tuple[float, float]]:
        """A* path planning"""
        # Implementation of A* algorithm
        open_set = []
        closed_set = set()
        came_from = {}
        
        # Use priority queue for open set
        import heapq
        heapq.heappush(open_set, (0, start))
        
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            
            if self.distance(current, goal) < 0.1:  # Goal reached
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path
            
            closed_set.add(current)
            
            for neighbor in self.get_neighbors(current):
                if neighbor in closed_set:
                    continue
                
                tentative_g_score = g_score[current] + self.distance(current, neighbor)
                
                if neighbor not in [item[1] for item in open_set]:
                    heapq.heappush(open_set, (tentative_g_score + self.heuristic(neighbor, goal), neighbor))
                elif tentative_g_score >= g_score.get(neighbor, float('inf')):
                    continue
                
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
        
        return []  # No path found
    
    def plan_gradient_descent(self, start: Tuple[float, float], goal: Tuple[float, float],
                                obstacles: List[Tuple[float, float, float]]) -> List[Tuple[float, float]]:
        """Gradient descent path optimization"""
        # Initialize path as straight line
        num_waypoints = 20
        path = []
        for i in range(num_waypoints):
            t = i / (num_waypoints - 1)
            x = start[0] + t * (goal[0] - start[0])
            y = start[1] + t * (goal[1] - start[1])
            path.append((x, y))
        
        # Optimize path
        for iteration in range(self.get_parameter('optimization_iterations').value):
            new_path = []
            for i, point in enumerate(path):
                # Calculate gradient
                gradient = self.calculate_gradient(point, obstacles)
                
                # Update point
                new_point = (point[0] - 0.01 * gradient[0], point[1] - 0.01 * gradient[1])
                new_path.append(new_point)
            
            path = new_path
        
        return path
    
    def get_obstacles_from_map(self) -> List[Tuple[float, float, float]]:
        """Extract obstacles from occupancy grid"""
        obstacles = []
        
        if not self.current_map:
            return obstacles
        
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        
        width = self.current_map.info.width
        height = self.current_map.info.height
        
        for y in range(height):
            for x in range(width):
                index = y * width + x
                if self.current_map.data[index] > 50:  # Occupied cell
                    world_x = origin_x + x * resolution
                    world_y = origin_y + y * resolution
                    obstacles.append((world_x, world_y, resolution))
        
        return obstacles
    
    def calculate_gradient(self, point: Tuple[float, float], 
                          obstacles: List[Tuple[float, float, float]]) -> Tuple[float, float]:
        """Calculate gradient for path optimization"""
        gradient_x = 0
        gradient_y = 0
        
        # Repulsion from obstacles
        for ox, oy, radius in obstacles:
            dx = point[0] - ox
            dy = point[1] - oy
            distance = np.sqrt(dx**2 + dy**2)
            
            if distance < radius + 1.0:
                # Strong repulsion near obstacles
                force = 10.0 / (distance**2 + 0.01)
                gradient_x += force * dx / distance
                gradient_y += force * dy / distance
        
        return (gradient_x, gradient_y)
    
    def get_neighbors(self, point: Tuple[float, float]) -> List[Tuple[float, float]]:
        """Get neighboring points for path planning"""
        neighbors = []
        directions = [(0, 0.1), (0.1, 0), (0, -0.1), (-0.1, 0), (0.1, 0.1), (-0.1, -0.1), (0.1, -0.1), (-0.1, 0.1)]
        
        for dx, dy in directions:
            neighbors.append((point[0] + dx, point[1] + dy))
        
        return neighbors
    
    def heuristic(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        """Heuristic function for A*"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def distance(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        """Calculate distance between two points"""
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def create_path_message(self, path: List[Tuple[float, float]]) -> Path:
        """Create ROS Path message from list of waypoints"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for x, y in path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        return path_msg
    
    def is_goal_reached(self) -> bool:
        """Check if current goal is reached"""
        if not self.current_pose or not self.current_goal:
            return False
        
        distance = self.distance(
            (self.current_pose.pose.position.x, self.current_pose.pose.position.y),
            (self.current_goal.pose.position.x, self.current_goal.pose.position.y)
        )
        
        return distance < 0.2  # 20cm tolerance
    
    def start_navigation(self):
        """Start navigation to current goal"""
        self.navigation_active = True
        msg = Bool()
        msg.data = True
        self.navigation_active_publisher.publish(msg)
        self.get_logger().info('Navigation started')
    
    def stop_navigation(self):
        """Stop current navigation"""
        self.navigation_active = False
        msg = Bool()
        msg.data = False
        self.navigation_active_publisher.publish(msg)
        self.get_logger().info('Goal reached! Navigation stopped')
    
    def update_path_following(self):
        """Update path following behavior"""
        # This would typically involve sending velocity commands
        # For this example, we'll just log the current progress
        if self.planned_path and self.current_pose:
            current_index = self.get_current_path_index()
            self.get_logger().info(f'Following path: waypoint {current_index}/{len(self.planned_path.poses)}')
    
    def get_current_path_index(self) -> int:
        """Get current index along the planned path"""
        if not self.planned_path or not self.current_pose:
            return 0
        
        current_pos = (self.current_pose.pose.position.x, self.current_pose.pose.position.y)
        min_distance = float('inf')
        current_index = 0
        
        for i, pose in enumerate(self.planned_path.poses):
            waypoint_pos = (pose.pose.position.x, pose.pose.position.y)
            distance = self.distance(current_pos, waypoint_pos)
            if distance < min_distance:
                min_distance = distance
                current_index = i
        
        return current_index

def main(args=None):
    rclpy.init(args=args)
    navigation_system = AdvancedNavigationSystem()
    
    try:
        rclpy.spin(navigation_system)
    except KeyboardInterrupt:
        pass
    finally:
        navigation_system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
      `);
      
      // Test complete system
      await page.click('[data-testid="test-system-button"]');
      await page.click('[data-testid="run-system-simulation"]');
      
      // Verify results
      await expect(page.locator('[data-testid="system-result"]')).toContainText('Navigation system working');
      await expect(page.locator('[data-testid="path-planned"]')).toBeVisible();
      await expect(page.locator('[data-testid="goal-reached"]')).toBeVisible();
      
      // Complete project
      await page.click('[data-testid="complete-project-button"]');
      await expect(page.locator('[data-testid="course-progress"]')).toContainText('100%');
      
      // Download project certificate
      await page.click('[data-testid="download-certificate-button"]');
      await expect(page.locator('[data-testid="project-certificate-downloaded"]')).toBeVisible();
    });
  });

  test.describe('Gazebo Simulation Course', () => {
    test('should complete Gazebo basics section', async () => {
      // Navigate to Gazebo course
      await page.click('[data-testid="ros-learning-button"]');
      await page.click('text=Gazebo Simulation');
      await page.waitForURL('**/ros-learning/gazebo');
      
      // Verify course content
      await expect(page.locator('h1')).toContainText('Gazebo Simulation');
      await expect(page.locator('text=Robot Simulation Basics')).toBeVisible();
      
      // Navigate to Gazebo basics
      await page.click('text=Gazebo Basics');
      
      // Start Gazebo simulation
      await page.click('[data-testid="start-simulation-button"]');
      
      // Wait for simulation to load
      await page.waitForSelector('[data-testid="gazebo-viewer"]');
      await expect(page.locator('[data-testid="simulation-loaded"]')).toBeVisible();
      
      // Interact with simulation
      await page.click('[data-testid="add-robot-button"]');
      await page.click('[data-testid="add-obstacle-button"]');
      
      // Complete section
      await page.click('[data-testid="complete-section-button"]');
      await expect(page.locator('[data-testid="course-progress"]')).toContainText('25%');
    });

    test('should complete robot modeling section', async () => {
      // Navigate to robot modeling
      await page.click('[data-testid="ros-learning-button"]');
      await page.click('text=Gazebo Simulation');
      await page.click('text=Robot Modeling');
      
      // Create robot model
      await page.click('[data-testid="create-robot-model-button"]');
      
      // Define robot URDF
      await page.type('[data-testid="urdf-editor"]', `
<?xml version="1.0"?>
<robot name="navlambda_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>
  
  <!-- Wheel links -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
  
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
  
  <!-- Wheel joints -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="-0.2 0.15 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
  
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="-0.2 -0.15 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
  
  <!-- Sensor links -->
  <link name="lidar_sensor">
    <visual>
      <geometry>
        <cylinder radius="0.02" length="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.02" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>
  
  <link name="camera_sensor">
    <visual>
      <geometry>
        <box size="0.03 0.03 0.02"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.03 0.03 0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.02"/>
      <inertia ixx="0.00001" ixy="0.0" ixz="0.0" iyy="0.00001" iyz="0.0" izz="0.00001"/>
    </inertial>
  </link>
  
  <!-- Sensor joints -->
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_sensor"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
  </joint>
  
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_sensor"/>
    <origin xyz="0.25 0 0.05" rpy="0 0 0"/>
  </joint>
</robot>
      `);
      
      // Validate robot model
      await page.click('[data-testid="validate-urdf-button"]');
      
      // Verify validation results
      await expect(page.locator('[data-testid="validation-result"]')).toContainText('URDF valid');
      
      // Load robot in simulation
      await page.click('[data-testid="load-robot-button"]');
      await expect(page.locator('[data-testid="robot-loaded"]')).toBeVisible();
      
      // Complete section
      await page.click('[data-testid="complete-section-button"]');
      await expect(page.locator('[data-testid="course-progress"]')).toContainText('50%');
    });
  });

  test.describe('Interactive Terminal Commands', () => {
    test('should execute ros2 topic commands', async () => {
      // Open terminal
      await page.click('[data-testid="terminal-tab"]');
      
      // Execute ros2 topic list
      await page.type('[data-testid="terminal-input"]', 'ros2 topic list');
      await page.keyboard.press('Enter');
      
      // Verify output
      await expect(page.locator('[data-testid="terminal-output"]')).toContainText('/parameter_events');
      await expect(page.locator('[data-testid="terminal-output"]')).toContainText('/rosout');
      
      // Execute ros2 topic info
      await page.type('[data-testid="terminal-input"]', 'ros2 topic info /rosout');
      await page.keyboard.press('Enter');
      
      // Verify detailed output
      await expect(page.locator('[data-testid="terminal-output"]')).toContainText('Type: rcl_interfaces/msg/Log');
    });

    test('should execute ros2 node commands', async () => {
      // Execute ros2 node list
      await page.type('[data-testid="terminal-input"]', 'ros2 node list');
      await page.keyboard.press('Enter');
      
      // Verify output
      await expect(page.locator('[data-testid="terminal-output"]')).toContainText('/_ros2cli_node_');
      
      // Execute ros2 node info
      await page.type('[data-testid="terminal-input"]', 'ros2 node info /_ros2cli_node_');
      await page.keyboard.press('Enter');
      
      // Verify detailed output
      await expect(page.locator('[data-testid="terminal-output"]')).toContainText('Subscribers:');
      await expect(page.locator('[data-testid="terminal-output"]')).toContainText('Publishers:');
    });

    test('should execute ros2 service commands', async () => {
      // Execute ros2 service list
      await page.type('[data-testid="terminal-input"]', 'ros2 service list');
      await page.keyboard.press('Enter');
      
      // Verify output
      await expect(page.locator('[data-testid="terminal-output"]')).toContainText('/_ros2cli_node_');
      
      // Execute ros2 service type
      await page.type('[data-testid="terminal-input"]', 'ros2 service type /_ros2cli_node_');
      await page.keyboard.press('Enter');
      
      // Verify output
      await expect(page.locator('[data-testid="terminal-output"]')).toContainText('Type:');
    });

    test('should execute ros2 param commands', async () => {
      // Execute ros2 param list
      await page.type('[data-testid="terminal-input"]', 'ros2 param list');
      await page.keyboard.press('Enter');
      
      // Verify output
      await expect(page.locator('[data-testid="terminal-output"]')).toContainText('use_sim_time');
      
      // Execute ros2 param get
      await page.type('[data-testid="terminal-input"]', 'ros2 param get /_ros2cli_node_ use_sim_time');
      await page.keyboard.press('Enter');
      
      // Verify output
      await expect(page.locator('[data-testid="terminal-output"]')).toContainText('Boolean value is:');
    });

    test('should execute ros2 launch commands', async () => {
      // Create a simple launch file
      await page.type('[data-testid="terminal-input"]', 'touch simple_launch.py');
      await page.keyboard.press('Enter');
      
      // Execute ros2 launch
      await page.type('[data-testid="terminal-input"]', 'ros2 launch simple_launch.py');
      await page.keyboard.press('Enter');
      
      // Verify launch output
      await expect(page.locator('[data-testid="terminal-output"]')).toContainText('Launch file not found');
    });

    test('should execute custom VNC commands', async () => {
      // Execute custom VNC compilation
      await page.type('[data-testid="terminal-input"]', 'navlambda compile navigation.vnc --target cpp');
      await page.keyboard.press('Enter');
      
      // Verify compilation output
      await expect(page.locator('[data-testid="terminal-output"]')).toContainText('Compilation successful');
      
      // Execute VNC execution
      await page.type('[data-testid="terminal-input"]', 'navlambda execute navigation.vnc');
      await page.keyboard.press('Enter');
      
      // Verify execution output
      await expect(page.locator('[data-testid="terminal-output"]')).toContainText('Execution completed');
    });
  });
});