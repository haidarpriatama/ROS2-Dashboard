#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import math
import time

class CircularPositionPublisher(Node):
    def __init__(self):
        super().__init__('circular_position_publisher')
        
        # Parameter lapangan (map)
        self.field_width = 1000  # Lebar lapangan dalam cm
        self.field_height = 600  # Tinggi lapangan dalam cm
        self.radius = 50  # Radius lingkaran dalam cm
        self.center_x = self.field_width / 2  # Titik tengah X
        self.center_y = self.field_height / 2  # Titik tengah Y
        
        # Publisher untuk posisi robot
        self.topic_name = 'coord_muter'
        self.publisher = self.create_publisher(Point, self.topic_name, 10)
        
        # Timer untuk mempublikasikan posisi setiap 0.1 detik
        self.timer_period = 0.1  # 100 ms
        self.timer = self.create_timer(self.timer_period, self.publish_position)
        
        # Variabel untuk menghitung posisi melingkar
        self.angle = 0.0  # Sudut awal dalam radian
        self.angular_speed = 1  # Kecepatan sudut dalam radian per detik
        
        self.get_logger().info(f"Publishing circular positions to topic '{self.topic_name}'")

    def publish_position(self):
        # Hitung posisi berdasarkan sudut
        x = self.center_x + self.radius * math.cos(self.angle)
        y = self.center_y + self.radius * math.sin(self.angle)
        
        # Buat pesan Point
        msg = Point()
        msg.x = x
        msg.y = y
        msg.z = 0.0  # Z tetap 0 karena hanya 2D
        
        # Publikasikan pesan
        self.publisher.publish(msg)
        self.get_logger().info(f"Published position: x={x:.2f}, y={y:.2f}")
        
        # Perbarui sudut untuk iterasi berikutnya
        self.angle += self.angular_speed * self.timer_period
        if self.angle >= 2 * math.pi:  # Reset sudut jika sudah satu putaran penuh
            self.angle -= 2 * math.pi

def main(args=None):
    rclpy.init(args=args)
    node = CircularPositionPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()