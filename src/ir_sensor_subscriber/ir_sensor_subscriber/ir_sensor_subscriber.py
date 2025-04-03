import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class IRSensorSubscriber(Node):
    def __init__(self):
        super().__init__('ir_sensor_subscriber')
        
        # All sensors are publishing LaserScan messages
        self.sensor_subs = []
        
        # Create subscribers for all 5 IR sensors
        for i in range(1, 6):
            topic_name = f'/ir_sensor_{i}'
            sub = self.create_subscription(
                LaserScan,
                topic_name,
                lambda msg, sensor_id=i: self.ir_sensor_callback(msg, sensor_id),
                10
            )
            self.sensor_subs.append(sub)
        
        self.get_logger().info('IR Sensor Subscriber initialized')
    
    def ir_sensor_callback(self, msg, sensor_id):
        # Extract range and intensity data
        # LaserScan message contains ranges and intensities arrays
        
        # Get the minimum range value (closest object)
        min_range = float('inf')
        min_index = -1
        
        for i, range_val in enumerate(msg.ranges):
            if range_val < min_range:
                min_range = range_val
                min_index = i
        
        # Get the corresponding intensity if available
        intensity = None
        if min_index >= 0 and len(msg.intensities) > min_index:
            intensity = msg.intensities[min_index]
        
        # Log the data
        self.get_logger().info(f'IR Sensor {sensor_id}:')
        self.get_logger().info(f'  - Range: {min_range:.3f} meters')
        if intensity is not None:
            self.get_logger().info(f'  - Intensity: {intensity:.3f}')
        else:
            self.get_logger().info('  - Intensity: Not available')
        
        # You could also log the full arrays if needed
        # self.get_logger().info(f'  - All ranges: {msg.ranges}')
        # self.get_logger().info(f'  - All intensities: {msg.intensities}')

def main(args=None):
    rclpy.init(args=args)
    ir_sensor_subscriber = IRSensorSubscriber()
    rclpy.spin(ir_sensor_subscriber)
    ir_sensor_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()