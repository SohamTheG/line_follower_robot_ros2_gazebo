import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range

class IRSensorSubscriber(Node):
    def __init__(self):
        super().__init__('ir_sensor_subscriber')
        
        # LaserScan type subscribers for ir_sensor_1 and ir_sensor_2
        self.ir_sensor_1_sub = self.create_subscription(
            LaserScan, 
            '/ir_sensor_1', 
            self.ir_sensor_1_callback, 
            10
        )
        
        self.ir_sensor_2_sub = self.create_subscription(
            LaserScan, 
            '/ir_sensor_2', 
            self.ir_sensor_2_callback, 
            10
        )
        
        # Range type subscribers for ir_sensor_3, ir_sensor_4, and ir_sensor_5
        self.ir_sensor_3_sub = self.create_subscription(
            Range, 
            '/ir_sensor_3', 
            self.ir_sensor_3_callback, 
            10
        )
        
        self.ir_sensor_4_sub = self.create_subscription(
            Range, 
            '/ir_sensor_4', 
            self.ir_sensor_4_callback, 
            10
        )
        
        self.ir_sensor_5_sub = self.create_subscription(
            Range, 
            '/ir_sensor_5', 
            self.ir_sensor_5_callback, 
            10
        )
    
    def ir_sensor_1_callback(self, msg):
        # Note the difference in message types for LaserScan
        self.get_logger().info(f'IR Sensor 1 Ranges: {msg.ranges}')
    
    def ir_sensor_2_callback(self, msg):
        self.get_logger().info(f'IR Sensor 2 Ranges: {msg.ranges}')
    
    def ir_sensor_3_callback(self, msg):
        # For Range type, we can directly access the range
        self.get_logger().info(f'IR Sensor 3 Range: {msg.range} meters')
    
    def ir_sensor_4_callback(self, msg):
        self.get_logger().info(f'IR Sensor 4 Range: {msg.range} meters')
    
    def ir_sensor_5_callback(self, msg):
        self.get_logger().info(f'IR Sensor 5 Range: {msg.range} meters')

def main(args=None):
    rclpy.init(args=args)
    ir_sensor_subscriber = IRSensorSubscriber()
    rclpy.spin(ir_sensor_subscriber)
    ir_sensor_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()