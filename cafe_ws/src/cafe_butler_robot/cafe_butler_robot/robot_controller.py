#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from threading import Timer

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        self.task_sub = self.create_subscription(String, '/robot_task', self.task_callback, 10)
        self.confirmation_pub = self.create_publisher(String, '/confirmation', 10)

        self.simulated_travel_time = 3.0  # seconds

    def task_callback(self, msg):
        task = msg.data
        self.get_logger().info(f"Received task: {task}")

        if task == "return_home":
            self.move_to("home")

        elif task == "go_kitchen":
            self.move_to("kitchen", confirm=True)

        elif task.startswith("go_table"):
            table = task.split("_")[-1]
            self.move_to(f"table{table}", confirm=True)

    def move_to(self, destination, confirm=False):
        self.get_logger().info(f"Moving to {destination}...")
        # Simulate movement
        Timer(self.simulated_travel_time, self.arrive_at, [destination, confirm]).start()

    def arrive_at(self, location, confirm):
        self.get_logger().info(f"Arrived at {location}")
        if confirm:
            self.send_confirmation(location)

    def send_confirmation(self, location):
        msg = String()
        msg.data = location
        self.confirmation_pub.publish(msg)
        self.get_logger().info(f"Published confirmation from: {location}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
