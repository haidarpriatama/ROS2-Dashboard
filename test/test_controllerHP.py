import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point

class JoystickPositionNode(Node):
    def __init__(self):
        super().__init__('joystick_position_node')
        self.subscription = self.create_subscription(
            String,
            '/ROBOT2/serverRX',
            self.joystick_callback,
            10
        )
        self.publisher_ = self.create_publisher(Point, 'robot_position', 10)
        self.current_position = Point()
        self.scale = 4.0  # skala cepat untuk lapangan besar

        self.get_logger().info('JoystickPositionNode aktif (skala besar)')

    def joystick_callback(self, msg):
        try:
            parts = msg.data.strip().split()
            if len(parts) >= 5 and parts[1] == 'T':
                input_x = float(parts[3])  # joystick kiri/kanan
                input_y = float(parts[2])  # joystick atas/bawah

                # PERBAIKAN: joystick maju gerakkan Y robot
                self.current_position.x += input_x * self.scale  # kanan/kiri
                self.current_position.y += input_y * self.scale  # atas/bawah

                # Format posisi dengan 2 angka di belakang koma
                self.current_position.x = round(self.current_position.x, 2)
                self.current_position.y = round(self.current_position.y, 2)
                self.current_position.z = round(self.current_position.z, 2)

                self.publisher_.publish(self.current_position)
                self.get_logger().info(
                    f'Posisi sekarang: x={self.current_position.x:.2f}, y={self.current_position.y:.2f}')
        except Exception as e:
            self.get_logger().error(f'Gagal parsing joystick data: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = JoystickPositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
