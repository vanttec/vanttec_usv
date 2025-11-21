#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QSlider, QDoubleSpinBox,
                             QPushButton, QCheckBox)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont

class MPCWeightsTuner(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        
        # Weight definitions: (name, default, min, max, step)
        self.weights_config = [
            ("w_along", 1.0, 0.0, 10000.0, 1.0),
            ("w_cross", 5.0, 0.0, 10000.0, 1.0),
            ("w_heading", 0.5, 0.0, 10000.0, 0.5),
            ("w_input", 0.01, 0.0, 10000.0, 0.01),
            ("w_slack", 10.0, 0.0, 10000.0, 1.0),
            ("w_surge", 0.01, 0.0, 10000.0, 0.01),
            ("w_yaw", 0.01, 0.0, 10000.0, 0.01),
            ("terminal_w", 5.0, 0.0, 100000.0, 1.0),
        ]
        
        self.weights = [config[1] for config in self.weights_config]
        self.init_ui()
        
    def init_ui(self):
        self.setWindowTitle("MPC Weights Tuner")
        self.setStyleSheet("""
            QMainWindow {
                background-color: #1e1e1e;
            }
            QWidget {
                background-color: #1e1e1e;
                color: #e0e0e0;
            }
            QLabel {
                color: #e0e0e0;
                font-size: 13px;
            }
            QSlider::groove:horizontal {
                border: 1px solid #3a3a3a;
                height: 6px;
                background: #2a2a2a;
                margin: 2px 0;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: #4a9eff;
                border: 1px solid #3a8eef;
                width: 16px;
                margin: -6px 0;
                border-radius: 8px;
            }
            QSlider::handle:horizontal:hover {
                background: #5aafff;
            }
            QDoubleSpinBox {
                background-color: #2a2a2a;
                border: 1px solid #3a3a3a;
                border-radius: 4px;
                padding: 4px;
                color: #e0e0e0;
                font-size: 12px;
            }
            QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
                background-color: #3a3a3a;
                border: none;
                width: 16px;
            }
            QDoubleSpinBox::up-button:hover, QDoubleSpinBox::down-button:hover {
                background-color: #4a4a4a;
            }
        """)
        
        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setSpacing(20)
        main_layout.setContentsMargins(30, 30, 30, 30)
        
        # Title
        title = QLabel("MPC Weights Configuration")
        title_font = QFont("Arial", 18, QFont.Bold)
        title.setFont(title_font)
        title.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title)
        
        # MPC Horizon DT
        tf_layout = QHBoxLayout()
        tf_label = QLabel("MPC Horizon tf (s)")
        tf_label.setFont(QFont("Arial", 13, QFont.Medium))
        tf_layout.addWidget(tf_label)
        tf_layout.addStretch()

        self.tf_spinbox = QDoubleSpinBox()
        self.tf_spinbox.setMinimum(0.1)
        self.tf_spinbox.setMaximum(30.0)
        self.tf_spinbox.setSingleStep(0.1)
        self.tf_spinbox.setValue(2.5)
        self.tf_spinbox.setDecimals(2)
        self.tf_spinbox.setMinimumWidth(80)
        self.tf_spinbox.valueChanged.connect(self.on_tf_changed)
        tf_layout.addWidget(self.tf_spinbox)

        main_layout.addLayout(tf_layout)

        # MPC Enable toggle
        toggle_layout = QHBoxLayout()
        toggle_label = QLabel("MPC Enabled")
        toggle_label.setFont(QFont("Arial", 13, QFont.Medium))
        toggle_layout.addWidget(toggle_label)
        toggle_layout.addStretch()

        self.mpc_toggle = QCheckBox()
        self.mpc_toggle.setChecked(True)
        self.mpc_toggle.setStyleSheet("""
            QCheckBox::indicator {
                width: 50px;
                height: 26px;
            }
            QCheckBox::indicator:unchecked {
                background-color: #3a3a3a;
                border: 2px solid #4a4a4a;
                border-radius: 13px;
            }
            QCheckBox::indicator:checked {
                background-color: #4a9eff;
                border: 2px solid #3a8eef;
                border-radius: 13px;
            }
        """)
        self.mpc_toggle.stateChanged.connect(self.on_toggle_changed)
        toggle_layout.addWidget(self.mpc_toggle)

        main_layout.addLayout(toggle_layout)
        
        # Create sliders for each weight
        self.sliders = []
        self.spinboxes = []
        for i, (name, default, min_val, max_val, step) in enumerate(self.weights_config):
            weight_widget = self.create_weight_slider(name, default, min_val, max_val, step, i)
            main_layout.addWidget(weight_widget)
        
        main_layout.addStretch()
        
        # Set window size
        self.setMinimumSize(300, 600)
        self.resize(300, 1000)
        
    def create_weight_slider(self, name, default, min_val, max_val, step, index):
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setSpacing(8)
        
        # Label row
        label_layout = QHBoxLayout()
        name_label = QLabel(name)
        name_font = QFont("Arial", 12, QFont.Medium)
        name_label.setFont(name_font)
        label_layout.addWidget(name_label)
        label_layout.addStretch()
        
        # Value spinbox
        spinbox = QDoubleSpinBox()
        spinbox.setMinimum(min_val)
        spinbox.setMaximum(max_val)
        spinbox.setSingleStep(step)
        spinbox.setValue(default)
        spinbox.setDecimals(1 if step >= 1.0 else 2)
        spinbox.setMinimumWidth(80)
        spinbox.valueChanged.connect(lambda v: self.on_spinbox_changed(index, v))
        self.spinboxes.append(spinbox)
        label_layout.addWidget(spinbox)

        # Reset button
        reset_btn = QPushButton("↻")
        reset_btn.setFixedSize(30, 30)
        reset_btn.setStyleSheet("""
            QPushButton {
                background-color: #3a3a3a;
                border: 1px solid #4a4a4a;
                border-radius: 15px;
                color: #e0e0e0;
                font-size: 16px;
            }
            QPushButton:hover {
                background-color: #4a4a4a;
            }
        """)
        reset_btn.clicked.connect(lambda: self.reset_weight(index, default))
        label_layout.addWidget(reset_btn)
        
        layout.addLayout(label_layout)
        
        # Slider
        slider = QSlider(Qt.Horizontal)
        # Scale slider to integer range for smooth control
        slider_scale = int(1.0 / step)
        slider.setMinimum(int(min_val * slider_scale))
        slider.setMaximum(int(max_val * slider_scale))
        slider.setValue(int(default * slider_scale))
        slider.setTickPosition(QSlider.TicksBelow)
        slider.setTickInterval(int((max_val - min_val) * slider_scale / 10))
        slider.valueChanged.connect(lambda v: self.on_slider_changed(index, v / slider_scale))
        self.sliders.append((slider, slider_scale))
        
        layout.addWidget(slider)
        
        # Add separator line
        separator = QWidget()
        separator.setFixedHeight(1)
        separator.setStyleSheet("background-color: #3a3a3a;")
        layout.addWidget(separator)
        
        return container
    
    def reset_weight(self, index, default_value):
        self.spinboxes[index].setValue(default_value)
    
    def on_toggle_changed(self, state):
        param = Parameter()
        param.name = "mpc_enabled"
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_BOOL
        param.value.bool_value = (state == Qt.Checked)
        
        request = SetParameters.Request()
        request.parameters = [param]
        
        future = self.node.set_params_client.call_async(request)
        future.add_done_callback(self.param_response_callback)

    def on_tf_changed(self, value):
        param = Parameter()
        param.name = "mpc_tf"
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = value
        
        request = SetParameters.Request()
        request.parameters = [param]
        
        future = self.node.set_params_client.call_async(request)
        future.add_done_callback(self.param_response_callback)

    def on_slider_changed(self, index, value):
        # Update spinbox
        self.spinboxes[index].blockSignals(True)
        self.spinboxes[index].setValue(value)
        self.spinboxes[index].blockSignals(False)
        
        # Update weight and publish
        self.weights[index] = value
        self.publish_weights()
    
    def on_spinbox_changed(self, index, value):
        # Update slider
        slider, scale = self.sliders[index]
        slider.blockSignals(True)
        slider.setValue(int(value * scale))
        slider.blockSignals(False)
        
        # Update weight and publish
        self.weights[index] = value
        self.publish_weights()
    
    def publish_weights(self):
        # Create parameter
        param = Parameter()
        param.name = "mpc_weights"
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        param.value.double_array_value = self.weights
        
        # Call set_parameters service
        request = SetParameters.Request()
        request.parameters = [param]
        
        future = self.node.set_params_client.call_async(request)
        future.add_done_callback(self.param_response_callback)
    
    def param_response_callback(self, future):
        try:
            response = future.result()
            if response.results and not response.results[0].successful:
                self.node.get_logger().warn(f"Failed to set parameter: {response.results[0].reason}")
        except Exception as e:
            self.node.get_logger().error(f"Service call failed: {e}")


class MPCWeightsNode(Node):
    def __init__(self):
        super().__init__('mpc_weights_tuner')
        
        
        # Create service client for setting parameters
        self.set_params_client = self.create_client(
            SetParameters,
            '/mpc_node/set_parameters'
        )
        
        # Wait for service
        self.get_logger().info(f"Waiting for parameter service on /mpc_node/set_parameters...")
        while not self.set_params_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.get_logger().info('Connected to parameter service!')


def main():
    rclpy.init()
    
    # Create ROS2 node
    node = MPCWeightsNode()
    
    # Create Qt application
    app = QApplication(sys.argv)
    
    # Create and show GUI
    window = MPCWeightsTuner(node)
    window.show()
    
    # Run Qt event loop with ROS2 spinning
    import threading
    
    def spin_ros():
        rclpy.spin(node)
    
    ros_thread = threading.Thread(target=spin_ros, daemon=True)
    ros_thread.start()
    
    # Execute Qt app
    exit_code = app.exec_()
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()