import sys
from PyQt5.QtWidgets import QApplication, QWidget, QSlider, QVBoxLayout
from PyQt5.QtCore import Qt

import rclpy
from rclpy.node import Node

class ParamSlider(Node):
    def __init__(self):
        super().__init__("param_slider")
        self.declare_parameter("testpara", 10)

    def set_testpara(self, value):
        self.set_parameters([rclpy.parameter.Parameter(
            "testpara",
            rclpy.Parameter.Type.INTEGER,
            value
        )])
        self.get_logger().info(f"Set testpara = {value}")

class SliderWindow(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(100)
        self.slider.setValue(10)

        self.slider.valueChanged.connect(self.on_value_changed)

        layout = QVBoxLayout()
        layout.addWidget(self.slider)
        self.setLayout(layout)

    def on_value_changed(self, value):
        self.node.set_testpara(value)

def main():
    rclpy.init()
    node = ParamSlider()

    app = QApplication(sys.argv)
    window = SliderWindow(node)
    window.show()

    # Qt と ROS2 を同時に回す
    while app.processEvents():
        rclpy.spin_once(node, timeout_sec=0.01)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()