import csv
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Wrench

class LQRLogger(Node):
    def __init__(self):
        super().__init__('lqr_performance_logger')
        self.start_time = time.time()
        self.log_file = open('lqr_performance.csv', 'w', newline='')
        self.writer = csv.writer(self.log_file)
        self.writer.writerow(['time', 'x', 'x_dot', 'theta', 'theta_dot', 'force'])
        self.state_pub = self.create_publisher(Float64MultiArray, '/cart_pole_states', 10)

        # Subscribers
        self.state_sub = self.create_subscription(
            Float64MultiArray,
            '/cart_pole_states',
            self.state_callback,
            10
        )
        self.force_sub = self.create_subscription(
            Wrench,
            '/cart_force',
            self.force_callback,
            10
        )

        # Variables
        self.state = [0.0, 0.0, 0.0, 0.0]  # [x, x_dot, theta, theta_dot]
        self.force = 0.0

    def state_callback(self, msg):
        self.state = msg.data

    def force_callback(self, msg):
        self.force = msg.force.x
        self.log_data()

    def log_data(self):
        elapsed_time = time.time() - self.start_time
        self.writer.writerow([elapsed_time] + self.state + [self.force])

    def close(self):
        self.log_file.close()

def main(args=None):
    rclpy.init(args=args)
    logger = LQRLogger()
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        logger.close()
        logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()