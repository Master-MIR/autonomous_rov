import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random
import numpy as np

class TopicPublisher(Node):  # <-- Inherit from Node
    def __init__(self):
        super().__init__('minimal_publisher')

        # self.ns = self.get_namespace()
        # self.get_logger().info("namespace =" + self.ns)

        self.publisher_ = self.create_publisher(Float64, "/bluerov2/global_position/rel_alt", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # Generate time array
        time_init = 0
        time_now = 0
        time_final = 20
        dt = 0.1
        time_values = np.arange(time_init, time_final , dt) 

        z_desired = []
        z_init = 0.0
        z_final = -0.5

        z_dot_desired = []

        a2 = (3.0 * (z_final - z_init) / (time_final**2))
        a3 = (-2.0 * (z_final - z_init) / (time_final**3))

        for i in time_values:

            print("Time: ", time_now)
            # print(i)
            if i < time_final:
                z = z_init + (a2 * time_now**2) + (a3 * time_now**3)
                z_dot = z_init + (2 * a2 * time_now) + (3 * a3 * time_now**2)
                z_dot_desired.append(z_dot)
                z_desired.append(z)

            elif i >= time_final: 
                print("time now >= time final")
                z_desired.append(z_final)
                z_dot_desired.append(0.0)
                # print(z_desired)

            else:
                print("Error")

            time_now += dt

        
        msg = Float64()
        # msg.data = random.uniform(0, 5)
        # for i in z_desired:
        if self.i >= len(z_desired):
            self.i = -1 
        msg.data = z_desired[self.i] 
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%f"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = TopicPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()
