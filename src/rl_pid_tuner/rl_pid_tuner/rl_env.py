import gymnasium as gym
import numpy as np
from gymnasium import spaces
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
import time
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState

class LineFollowerEnv(gym.Env):
    def __init__(self):
        super().__init__()

        # Initialize ROS node
        if not rclpy.ok():
            rclpy.init()
        self.node = Node('line_follower_env')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        # Create callback group
        self.callback_group = ReentrantCallbackGroup()

        # Reset service client
        self.reset_sim_client = self.node.create_client(
            Empty, '/reset_simulation', callback_group=self.callback_group
        )

        # Define action space (Kp, Ki, Kd)
        self.action_space = spaces.Box(
            low=np.array([0.1, 0.0, 0.0]), 
            high=np.array([2.0, 0.1, 0.5]), 
            dtype=np.float32
        )

        # Observation space (5 sensors + error + error_derivative + angular_velocity)
        self.observation_space = spaces.Box(
            low=-1.0, high=1.0, shape=(8,), dtype=np.float32
        )

        # Publishers and subscribers
        self.cmd_vel_pub = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.sensor_values = [0] * 5
        for i in range(1, 6):
            self.node.create_subscription(
                LaserScan, f'ir_sensor_{i}',
                lambda msg, idx=i-1: self.sensor_callback(msg, idx), 10
            )

        # PID control variables
        self.error = 0
        self.last_error = 0
        self.angular_velocity = 0
        self.kp, self.ki, self.kd = 1.0, 0.0, 0.1

        # Episode tracking
        self.steps_in_episode = 0
        self.max_steps = 50

        # Wait for services and topics to be ready
        self._wait_for_services()

    def randomize_robot_position(self):
        """Randomly move the robot to a new starting position in Gazebo"""
        set_state_client = self.node.create_client(SetEntityState, '/set_entity_state')
        if not set_state_client.wait_for_service(timeout_sec=3.0):
            self.node.get_logger().error("SetEntityState service not available!")
            return
        new_state = EntityState()
        new_state.name = "line_follower_robot"  # Replace with your actual robot name
        new_state.pose.position.x = np.random.uniform(-0.2, 0.2)  # Random X position
        new_state.pose.position.y = np.random.uniform(-0.2, 0.2)  # Random Y position
        new_state.pose.orientation.z = np.random.uniform(-0.1, 0.1)  # Random orientation
        request = SetEntityState.Request()
        request.state = new_state
        future = set_state_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        if future.done():
            self.node.get_logger().info("Robot position randomized successfully!")
        else:
            self.node.get_logger().error("Failed to randomize robot position!")

    
    
    def _wait_for_services(self):
        """Wait for required services to become available"""
        self.node.get_logger().info("Waiting for reset simulation service...")
        while not self.reset_sim_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Reset service not available, waiting...")
        self.node.get_logger().info("Services ready!")

    def sensor_callback(self, msg, sensor_idx):
        """Process sensor data"""
        self.sensor_values[sensor_idx] = sum(msg.intensities) / 255.0

        print(f"Sensor {sensor_idx}: {self.sensor_values[sensor_idx]}")

        # Calculate weighted error (-1 to 1 range)  
        weights = [-2, -1, 0, 1, 2]
        total_value = sum(self.sensor_values)

        if total_value > 0:
            self.error = sum(w * v for w, v in zip(weights, self.sensor_values)) / (total_value * 2)
        else:
            self.error = 1.0 if self.last_error > 0 else -1.0
        print(f"Updated Error: {self.error}")


    def reset(self, seed=None, options=None):
        """Reset the environment and return initial observation"""

        self.node.get_logger().info("Calling reset_simulation service...")
        future = self.reset_sim_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self.node, future)

        if future.done():
            self.node.get_logger().info("Reset service completed successfully!")
        else:
            self.node.get_logger().error("Reset service did NOT complete!")

        # Reset internal variables
        #self.error = 0
        #self.last_error = 0
        self.angular_velocity = 0
        self.steps_in_episode = 0

        self.randomize_robot_position()

        # Stop the robot
        self._stop_robot()
        time.sleep(0.5)  # Let the robot stabilize

        # Process callbacks to get fresh sensor data
        for _ in range(10):
            self.executor.spin_once(timeout_sec=0.1)

        return self._get_observation(), {}

    def _get_observation(self):
        """Create observation vector from current state"""
        error_derivative = self.error - self.last_error
        print(f"Obs - Error: {self.error}, Error Derivative: {error_derivative}, Angular Vel: {self.angular_velocity}")
        return np.array(
            self.sensor_values + [self.error, error_derivative, self.angular_velocity],
            dtype=np.float32
        )

    def _stop_robot(self):
        """Stop robot movement"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def step(self, action):
        """Execute one step in the environment"""

        # Update PID gains from action
        #self.kp, self.ki, self.kd = action
        self.kp = 0.9 * self.kp + 0.1 * action[0]
        self.ki = 0.9 * self.ki + 0.1 * action[1]
        self.kd = 0.9 * self.kd + 0.1 * action[2]
        self.executor.spin_once(timeout_sec=0.1)

        # Calculate error derivative
        error_derivative = self.error - self.last_error
        self.last_error = self.error
        print(f"Step: Error before update: {self.error}, Last error: {self.last_error}")

        # Compute PID correction
        correction = (
            self.kp * self.error + 
            self.ki * (self.error + self.last_error) + 
            self.kd * error_derivative
        )

        # Apply control to robot
        cmd = Twist()
        cmd.linear.x = 0.3  # Constant forward speed
        cmd.angular.z = -correction * 1.5 # Steering control
        self.cmd_vel_pub.publish(cmd)

        # Save angular velocity for observation
        self.angular_velocity = cmd.angular.z

        # Simulate time passage
        time.sleep(0.05)
        self.executor.spin_once(timeout_sec=0.1)

        # Get new observation
        observation = self._get_observation()

        # Reward function: encourage staying on track, penalize oscillations & excessive turns
        on_line_reward = 1.0 - abs(self.error)
        smoothness_penalty = 0.3 * abs(error_derivative)
        efficiency_penalty = 0.2 * abs(cmd.angular.z)
        reward = on_line_reward - smoothness_penalty - efficiency_penalty

        # Increment step count
        self.steps_in_episode += 1

        # Episode termination conditions
        done = abs(self.error) > 0.9 or self.steps_in_episode >= self.max_steps

        # Debugging info
        info = {
            'error': self.error,
            'sensors': self.sensor_values,
            'reward': reward,
            'kp': self.kp,
            'ki': self.ki,
            'kd': self.kd
        }

        print(f"Step {self.steps_in_episode}, Reward: {reward:.4f}, Error: {self.error:.4f}")
        print(f"PID: Kp={self.kp:.4f}, Ki={self.ki:.4f}, Kd={self.kd:.4f}")

        return observation, reward, done, False, info

    def close(self):
        """Clean up resources"""
        self._stop_robot()
        self.node.destroy_node()
        rclpy.shutdown()
