import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# Configuration Parameters
LINEAR_SPEED = 0.3          # Base forward speed
MAX_ANGULAR_SPEED = 0.5     # Maximum turning speed
SENSOR_THRESHOLD = 180      # Threshold for line detection
TIMER_PERIOD = 0.04         # Control loop frequency

# PID Gains
KP = 0.3                   # Proportional gain
KI = 0.001                  # Integral gain (small value to prevent oscillation)
KD = 0.04                   # Derivative gain

class PIDLineFollower(Node):
    def __init__(self):
        super().__init__('pid_line_follower')
        
        self.num_sensors = 5
        self.sensor_values = [0] * self.num_sensors
        
        for i in range(1, self.num_sensors + 1):
            self.create_subscription(
                LaserScan, 
                f'ir_sensor_{i}', 
                lambda msg, idx=i: self.sensor_callback(msg, idx),
                10
            )
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # PID Variables
        self.last_error = 0
        self.integral = 0

        self.timer = self.create_timer(TIMER_PERIOD, self.control_loop)
        
        self.get_logger().info('PID line follower initialized')

    def sensor_callback(self, msg, sensor_idx):
        value = 1 if sum(msg.intensities) >= SENSOR_THRESHOLD else 0
        self.sensor_values[sensor_idx - 1] = value
        print(f"Sensor {sensor_idx}: {value}")  # Debugging sensor values

    def calculate_position_error(self):
        if sum(self.sensor_values) == 0:
            return None
            
        weights = [-2, -1, 0, 1, 2]
        error_sum = sum(w * v for w, v in zip(weights, self.sensor_values))
        active_sensors = sum(self.sensor_values)

        error = error_sum / active_sensors if active_sensors > 0 else 0
        print(f"Sensor Values: {self.sensor_values} -> Position Error: {error}")  # Debugging error calculation
        
        return error

    def control_loop(self):
        error = self.calculate_position_error()
        cmd = Twist()
        
        if error is not None:
            self.integral += error * TIMER_PERIOD  # Integrate the error over time
            derivative = (error - self.last_error) / TIMER_PERIOD  # Compute derivative
            correction = KP * error + KI * self.integral + KD * derivative  # PID equation
            
            print(f"PID: P={KP*error:.4f}, I={KI*self.integral:.4f}, D={KD*derivative:.4f}")  # Debugging PID terms
            print(f"Correction: {correction:.4f}")  # Debugging correction value

            cmd.linear.x = LINEAR_SPEED
            cmd.angular.z = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, -correction))
            print(f"Command: Linear={cmd.linear.x:.2f}, Angular={cmd.angular.z:.2f}")  # Debugging motor commands
            
            self.last_error = error
        else:
            cmd.linear.x = LINEAR_SPEED * 0.8
            cmd.angular.z = -self.last_error * KP * 1.5  # Continue correcting if line is lost
            print("Line lost! Applying last known correction.")  # Debugging lost line case

        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PIDLineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist())
        rclpy.shutdown()

if __name__ == '__main__':
    main()
