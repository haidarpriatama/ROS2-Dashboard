import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading

class LivePlotter(Node):
    def __init__(self):
        super().__init__('live_plotter')
        self.subscription = self.create_subscription(
            String,
            'plot_topic',  # Pastikan nama topic sama
            self.listener_callback,
            10)
        self.subscription

        # Data untuk plot
        self.time_data = []
        self.x_data = []
        self.spx_data = []
        self.y_data = []
        self.spy_data = []

        self.start_time = self.get_clock().now().nanoseconds / 1e9  # Waktu mulai (detik)

        # Setup figure
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1)
        self.line_x, = self.ax1.plot([], [], 'b-', label='x')
        self.line_spx, = self.ax1.plot([], [], 'k-', label='spx')

        self.line_y, = self.ax2.plot([], [], 'r-', label='y')
        self.line_spy, = self.ax2.plot([], [], 'k-', label='spy')

        self.ax1.legend()
        self.ax2.legend()

        # Bikin grid biar enak diliat
        self.ax1.grid(True)
        self.ax2.grid(True)

        # Animasi
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=100)

    def listener_callback(self, msg):
        try:
            parts = msg.data.split()
            if parts[0] == 'L' and len(parts) == 5:
                x = float(parts[1])
                y = float(parts[2])
                spx = float(parts[3])
                spy = float(parts[4])

                now = self.get_clock().now().nanoseconds / 1e9
                elapsed_time = now - self.start_time

                self.time_data.append(elapsed_time)
                self.x_data.append(x)
                self.spx_data.append(spx)
                self.y_data.append(y)
                self.spy_data.append(spy)

                self.get_logger().info(f"Data received: {elapsed_time:.2f}s, x={x}, y={y}, spx={spx}, spy={spy}")
        except Exception as e:
            self.get_logger().error(f"Failed to parse data: {e}")

    def update_plot(self, frame):
        # Update data ke line
        self.line_x.set_data(self.time_data, self.x_data)
        self.line_spx.set_data(self.time_data, self.spx_data)

        self.line_y.set_data(self.time_data, self.y_data)
        self.line_spy.set_data(self.time_data, self.spy_data)

        # Sliding window: tampilkan hanya 10 detik terakhir
        window_size = 10.0  # detik
        if self.time_data:
            current_time = self.time_data[-1]
            min_time = current_time - window_size
            self.ax1.set_xlim(min_time, current_time)
            self.ax2.set_xlim(min_time, current_time)

        # Autoscale Y, jangan autoscale X
        for ax in [self.ax1, self.ax2]:
            ax.relim()
            ax.autoscale_view(scalex=False)

        self.ax1.set_xlabel('Time [s]')
        self.ax1.set_ylabel('X and SPX')
        self.ax2.set_xlabel('Time [s]')
        self.ax2.set_ylabel('Y and SPY')

    def spin_matplotlib(self):
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = LivePlotter()

    # Spin ROS2 di thread terpisah
    thread = threading.Thread(target=rclpy.spin, args=(node,))
    thread.start()

    # Mainloop Matplotlib
    node.spin_matplotlib()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
