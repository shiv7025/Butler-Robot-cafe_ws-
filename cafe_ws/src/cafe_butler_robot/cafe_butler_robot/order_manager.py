#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from threading import Timer

class OrderManager(Node):
    def __init__(self):
        super().__init__('order_manager')

        self.order_sub = self.create_subscription(String, '/order', self.order_callback, 10)
        self.confirmation_sub = self.create_subscription(String, '/confirmation', self.confirmation_callback, 10)
        self.cancel_order_sub = self.create_subscription(String, '/cancel_order', self.cancel_order_callback, 10)
        self.cancel_table_sub = self.create_subscription(String, '/cancel_table', self.cancel_table_callback, 10)

        self.task_pub = self.create_publisher(String, '/robot_task', 10)

        self.current_orders = []
        self.pending_table = None
        self.waiting_for = None
        self.timeout_timer = None
        self.timeout_seconds = 10  # Adjust as needed

        self.cancel_all = False
        self.cancelled_tables = []

    def order_callback(self, msg):
        self.get_logger().info(f"Order Received: {msg.data}")
        self.current_orders = msg.data.split(',')
        self.cancel_all = False
        self.cancelled_tables = []
        self.start_task_flow()

    def confirmation_callback(self, msg):
        sender = msg.data
        self.get_logger().info(f"Confirmation received from {sender}")
        if self.waiting_for == sender:
            self.stop_timeout()
            self.waiting_for = None
            self.proceed_task_flow()

    def cancel_order_callback(self, msg):
        self.get_logger().info("Order canceled entirely")
        self.cancel_all = True
        self.stop_timeout()
        self.publish_task("return_home")

    def cancel_table_callback(self, msg):
        table = msg.data
        if table in self.current_orders:
            self.cancelled_tables.append(table)
            self.get_logger().info(f"Cancelled table: {table}")

    def publish_task(self, task):
        msg = String()
        msg.data = task
        self.task_pub.publish(msg)
        self.get_logger().info(f"Task Sent: {task}")

    def start_task_flow(self):
        if self.cancel_all or not self.current_orders:
            self.publish_task("return_home")
            return
        self.publish_task("go_kitchen")
        self.wait_for_confirmation("kitchen", self.kitchen_timeout)

    def kitchen_timeout(self):
        self.get_logger().info("Kitchen timeout occurred")
        self.publish_task("return_home")

    def proceed_task_flow(self):
        self.skip_cancelled_tables()
        if not self.current_orders:
            self.publish_task("go_kitchen")  # After table confirmation failure
            self.wait_for_confirmation("kitchen", self.kitchen_timeout_after_failure)
            return

        if self.pending_table:
            self.deliver_to_table(self.pending_table)
        elif self.current_orders:
            next_table = self.current_orders.pop(0)
            self.deliver_to_table(next_table)
        else:
            self.publish_task("go_kitchen")  # Before going home
            self.wait_for_confirmation("kitchen", self.final_kitchen_confirm)

    def deliver_to_table(self, table):
        self.pending_table = table
        self.publish_task(f"go_{table}")
        self.wait_for_confirmation(table, lambda: self.table_timeout(table))

    def table_timeout(self, table):
        self.get_logger().info(f"{table} did not confirm. Skipping to next.")
        self.pending_table = None
        self.proceed_task_flow()

    def kitchen_timeout_after_failure(self):
        self.get_logger().info("After failed table delivery, kitchen didn't confirm. Returning home.")
        self.publish_task("return_home")

    def final_kitchen_confirm(self):
        self.publish_task("return_home")

    def wait_for_confirmation(self, source, timeout_callback):
        self.waiting_for = source
        self.start_timeout(timeout_callback)

    def start_timeout(self, callback):
        if self.timeout_timer:
            self.timeout_timer.cancel()
        self.timeout_timer = Timer(self.timeout_seconds, callback)
        self.timeout_timer.start()

    def stop_timeout(self):
        if self.timeout_timer:
            self.timeout_timer.cancel()
            self.timeout_timer = None

    def skip_cancelled_tables(self):
        self.current_orders = [t for t in self.current_orders if t not in self.cancelled_tables]

def main(args=None):
    rclpy.init(args=args)
    node = OrderManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
