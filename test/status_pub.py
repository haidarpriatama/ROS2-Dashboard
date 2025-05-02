import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class SimpleStatusPublisher(Node):
    def __init__(self):
        super().__init__('simple_status_publisher')
        # Status publishers
        self.publisher1 = self.create_publisher(String, '/ROBOT1/STATUS', 10)
        self.publisher2 = self.create_publisher(String, '/ROBOT2/STATUS', 10)
        
        # Battery publishers
        self.battery_pub1 = self.create_publisher(String, '/ROBOT1/battery', 10)
        self.battery_pub2 = self.create_publisher(String, '/ROBOT2/battery', 10)

        # Initialize battery levels
        self.robot1_battery = 0
        self.robot2_battery = 100

        # Direction of battery change
        self.robot1_direction = 1  # Increment
        self.robot2_direction = -1  # Decrement

        # Timers
        self.timer = self.create_timer(1.0, self.publish_status)  # Publish status every second
        self.battery_timer = self.create_timer(0.5, self.publish_battery)  # Publish battery every 0.5 seconds

    def generate_status(self):
        cpu = random.randint(0, 100)
        mem = random.randint(0, 100)
        temp = random.randint(40, 90)
        return f'ST {cpu} {mem} {temp}'

    def publish_status(self):
        status1 = self.generate_status()
        status2 = self.generate_status()

        self.publisher1.publish(String(data=status1))
        self.publisher2.publish(String(data=status2))

        self.get_logger().info(f'[ROBOT1] {status1}')
        self.get_logger().info(f'[ROBOT2] {status2}')

    def publish_battery(self):
        # Update Robot 1 battery level
        self.robot1_battery += self.robot1_direction
        if self.robot1_battery >= 100:
            self.robot1_battery = 100
            self.robot1_direction = -1  # Reverse direction
        elif self.robot1_battery <= 0:
            self.robot1_battery = 0
            self.robot1_direction = 1  # Reverse direction

        # Update Robot 2 battery level
        self.robot2_battery += self.robot2_direction
        if self.robot2_battery >= 100:
            self.robot2_battery = 100
            self.robot2_direction = -1  # Reverse direction
        elif self.robot2_battery <= 0:
            self.robot2_battery = 0
            self.robot2_direction = 1  # Reverse direction

        # Publish battery levels as strings
        self.battery_pub1.publish(String(data=str(self.robot1_battery)))
        self.battery_pub2.publish(String(data=str(self.robot2_battery)))

        # Log the published values
        self.get_logger().info(f'[ROBOT1 Battery] {self.robot1_battery}')
        self.get_logger().info(f'[ROBOT2 Battery] {self.robot2_battery}')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleStatusPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
