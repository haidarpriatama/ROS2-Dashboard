#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math
import time

class SinusoidalPublisher(Node):
    def __init__(self):
        super().__init__('sinusoidal_publisher')
        
        # Create a publisher
        self.publisher_ = self.create_publisher(String, 'graph_topic', 10)
        
        # Create a timer to publish data every 0.1 seconds
        self.timer_ = self.create_timer(0.1, self.publish_sinusoidal_data)
        
        # Initial time
        self.start_time = time.time()

    def publish_sinusoidal_data(self):
        # Calculate elapsed time
        current_time = time.time() - self.start_time
        
        # Generate multiple sinusoidal signals with different frequencies and amplitudes
        # Reference signal
        reference = math.sin(current_time)
        
        # Multiple signals with varying frequencies and phase shifts
        x = math.sin(current_time)
        y = 0.8 * math.sin(current_time + math.pi/4)
        yaw = 1.2 * math.sin(current_time + math.pi/2)
        spx = 0.5 * math.sin(2 * current_time)
        spy = 0.7 * math.sin(2 * current_time + math.pi/3)
        spyaw = 0.6 * math.sin(0.5 * current_time)
        
        # Format the message: 'L [x] [y] [yaw] [spx] [spy] [spyaw]'
        msg_data = f"L {x:.2f} {y:.2f} {yaw:.2f} {spx:.2f} {spy:.2f} {spyaw:.2f}"
        
        # Create and publish the message
        msg = String()
        msg.data = msg_data
        self.publisher_.publish(msg)
        
        # Optional: Log the published message
        self.get_logger().info(f'Publishing: {msg_data}')

def main(args=None):
    rclpy.init(args=args)
    
    sinusoidal_publisher = SinusoidalPublisher()
    
    try:
        rclpy.spin(sinusoidal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        sinusoidal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()