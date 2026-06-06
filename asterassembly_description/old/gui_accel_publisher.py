import rclpy
from rclpy.node import Node
from geometry_msgs.msg import AccelStamped
import tkinter as tk
from tkinter import ttk
import threading

class AccelPublisher(Node):
    def __init__(self):
        super().__init__('accel_gui_publisher')
        self.publisher_ = self.create_publisher(AccelStamped, '/accel_control', 10)
        self.current_value = 0.5  # fréquence initiale
        self.timer = self.create_timer(0.1, self.publish_message)

    def set_value(self, val):
        self.current_value = float(val)

    def publish_message(self):
        msg = AccelStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.accel.linear.x = self.current_value
        self.publisher_.publish(msg)

def launch_gui(node):
    window = tk.Tk()
    window.title("Contrôle de la fréquence de marche")

    label = ttk.Label(window, text="Fréquence (Hz)", font=("Arial", 14))
    label.pack(pady=10)

    slider = ttk.Scale(window, from_=0.1, to=2.0, orient='horizontal', command=node.set_value)
    slider.set(0.5)
    slider.pack(padx=20, pady=20, fill='x')

    window.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = AccelPublisher()

    gui_thread = threading.Thread(target=launch_gui, args=(node,), daemon=True)
    gui_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

