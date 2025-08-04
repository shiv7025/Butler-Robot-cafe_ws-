#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
from tkinter import simpledialog

class CafeGUI(Node):
    def __init__(self):
        super().__init__('cafe_gui')

        self.order_pub = self.create_publisher(String, '/order', 10)
        self.confirm_pub = self.create_publisher(String, '/confirmation', 10)
        self.cancel_order_pub = self.create_publisher(String, '/cancel_order', 10)
        self.cancel_table_pub = self.create_publisher(String, '/cancel_table', 10)

        self.window = tk.Tk()
        self.window.title("Cafe GUI")

        # Order buttons
        tk.Label(self.window, text="Send Order").pack(pady=5)
        tk.Button(self.window, text="Single Table Order", command=self.send_single_order).pack()
        tk.Button(self.window, text="Multiple Table Order", command=self.send_multi_order).pack()

        # Confirmation buttons
        tk.Label(self.window, text="\nSend Confirmation").pack(pady=5)
        tk.Button(self.window, text="Kitchen Confirm", command=lambda: self.send_confirmation("kitchen")).pack()
        tk.Button(self.window, text="Table1 Confirm", command=lambda: self.send_confirmation("table1")).pack()
        tk.Button(self.window, text="Table2 Confirm", command=lambda: self.send_confirmation("table2")).pack()
        tk.Button(self.window, text="Table3 Confirm", command=lambda: self.send_confirmation("table3")).pack()

        # Cancellation
        tk.Label(self.window, text="\nCancel").pack(pady=5)
        tk.Button(self.window, text="Cancel Entire Order", command=self.cancel_order).pack()
        tk.Button(self.window, text="Cancel Table", command=self.cancel_table).pack()

        self.window.protocol("WM_DELETE_WINDOW", self.shutdown)

    def send_single_order(self):
        table = simpledialog.askstring("Input", "Enter table name (e.g., table1):")
        if table:
            self.publish_order([table])

    def send_multi_order(self):
        tables = simpledialog.askstring("Input", "Enter tables separated by commas (e.g., table1,table2):")
        if tables:
            table_list = [t.strip() for t in tables.split(",")]
            self.publish_order(table_list)

    def publish_order(self, tables):
        order_msg = String()
        order_msg.data = ",".join(tables)
        self.order_pub.publish(order_msg)
        self.get_logger().info(f"Published order: {order_msg.data}")

    def send_confirmation(self, location):
        confirm_msg = String()
        confirm_msg.data = location
        self.confirm_pub.publish(confirm_msg)
        self.get_logger().info(f"Sent confirmation from: {location}")

    def cancel_order(self):
        cancel_msg = String()
        cancel_msg.data = "cancel"
        self.cancel_order_pub.publish(cancel_msg)
        self.get_logger().info("Entire order cancelled.")

    def cancel_table(self):
        table = simpledialog.askstring("Input", "Enter table to cancel (e.g., table2):")
        if table:
            cancel_msg = String()
            cancel_msg.data = table
            self.cancel_table_pub.publish(cancel_msg)
            self.get_logger().info(f"Cancelled table: {table}")

    def spin(self):
        self.window.after(100, self.spin_once)
        self.window.mainloop()

    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.1)
        self.window.after(100, self.spin_once)

    def shutdown(self):
        self.get_logger().info("Shutting down Cafe GUI")
        self.window.destroy()
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    gui = CafeGUI()
    gui.spin()

if __name__ == '__main__':
    main()
