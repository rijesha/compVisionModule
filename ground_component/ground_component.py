import sys

from PySide2.QtCore import Signal
from PySide2.QtWidgets import QApplication, QMainWindow, QPushButton, QGridLayout
from PySide2.QtWidgets import QPushButton, QLabel, QLineEdit, QWidget
from mavlink_handler import MavlinkHandler
from mavlink_handler import *

# Subclass QMainWindow to customize your application's main window


class MainWindow(QMainWindow):
    got_msg_signal = Signal(object)

    def __init__(self):
        super().__init__()
        self.mav = None
        self.__current_north_label = QLabel("xx")
        self.__current_east_label = QLabel("xx")
        self.__current_down_label = QLabel("xx")
        self.__current_heading_label = QLabel("xx")
        self.__current_angle_in_frame_label = QLabel("xx")

        self.__desired_north_label = QLabel("xx")
        self.__desired_east_label = QLabel("xx")
        self.__desired_down_label = QLabel("xx")
        self.__desired_angle_in_frame_label = QLabel("xx")

        self.__desired_north_label_edit = QLineEdit()
        self.__desired_east_label_edit = QLineEdit()
        self.__desired_down_label_edit = QLineEdit()
        self.__desired_angle_in_frame_label_edit = QLineEdit("0.20")

        self.__position_pgain_label = QLabel("xx")
        self.__position_pgain_label_edit = QLineEdit()
        self.__position_pgain_z_label = QLabel("xx")
        self.__position_pgain_z_label_edit = QLineEdit()

        self.__velocity_pgain_label = QLabel("xx")
        self.__velocity_pgain_label_edit = QLineEdit()
        self.__velocity_pgain_z_label = QLabel("xx")
        self.__velocity_pgain_z_label_edit = QLineEdit()

        self.__velocity_igain_label = QLabel("xx")
        self.__velocity_igain_label_edit = QLineEdit()
        self.__velocity_igain_z_label = QLabel("xx")
        self.__velocity_igain_z_label_edit = QLineEdit()

        self.__yaw_pgain_label_label = QLabel("xx")
        self.__yaw_pgain_label_edit = QLineEdit()

        self.__target_velocity_n_label = QLabel("xx")
        self.__target_velocity_e_label = QLabel("xx")
        self.__target_velocity_d_label = QLabel("xx")
        self.__velocity_n_label = QLabel("xx")
        self.__velocity_e_label = QLabel("xx")
        self.__velocity_d_label = QLabel("xx")
        self.__target_acceleration_n_label = QLabel("xx")
        self.__target_acceleration_e_label = QLabel("xx")
        self.__target_acceleration_d_label = QLabel("xx")

        self.__target_roll_label = QLabel("xx")
        self.__target_pitch_label = QLabel("xx")

        self.__update_desired_pose = QPushButton("Update Desired Position")
        self.__update_control_gains = QPushButton("Update Control Gains")
        self.__enable_control = QPushButton("Enable Control")
        self.__disable_control = QPushButton("Disable Control")

        self.__update_desired_pose.clicked.connect(
            self.__update_desired_pose_func)
        self.__update_control_gains.clicked.connect(
            self.__update_desired_control_gains_func)
        self.__enable_control.clicked.connect(self.__enable_control_func)
        self.__disable_control.clicked.connect(self.__disable_control_func)

        self.setWindowTitle("UAV Controller")
        self.got_msg_signal.connect(self.message_handler)

        layout = QGridLayout()

        layout.addWidget(self.create_ned_layout(), 0, 0)
        layout.addWidget(self.create_control_gains_layout(), 0, 1)
        layout.addWidget(self.create_control_targets_layout(), 1, 0)
        layout.addWidget(self.__enable_control, 2, 0)
        layout.addWidget(self.__disable_control, 2, 1)

        # Set the central widget of the Window.
        widget = QWidget()
        widget.setLayout(layout)
        self.setCentralWidget(widget)

    def bind_msg_sender(self, sender):
        self.mav = sender

    def __update_desired_pose_func(self):
        msg = MAVLink_set_target_ned_message(float(self.__desired_north_label_edit.text()), float(self.__desired_east_label_edit.text()), float(
            self.__desired_down_label_edit.text()), float(self.__desired_angle_in_frame_label_edit.text()))
        self.mav.send(msg)

    def __update_desired_control_gains_func(self):
        msg = MAVLink_set_control_gains_message(float(self.__position_pgain_label_edit.text()), float(self.__position_pgain_z_label_edit.text()), float(self.__velocity_pgain_label_edit.text()), float(
            self.__velocity_igain_label_edit.text()), float(self.__velocity_pgain_z_label_edit.text()), float(self.__velocity_igain_z_label_edit.text()), float(self.__yaw_pgain_label_edit.text()))
        self.mav.send(msg)

    def __enable_control_func(self):
        pass

    def __disable_control_func(self):
        pass

    def create_ned_layout(self):
        widget = QWidget()
        layout = QGridLayout()

        layout.addWidget(QLabel("Field"), 0, 0)
        layout.addWidget(QLabel("Current Value"), 0, 1)
        layout.addWidget(QLabel("Desired Value Edit"), 0, 2)
        layout.addWidget(QLabel("Desired Value"), 0, 3)

        layout.addWidget(QLabel("N:"), 1, 0)
        layout.addWidget(self.__current_north_label, 1, 1)
        layout.addWidget(self.__desired_north_label_edit, 1, 2)
        layout.addWidget(self.__desired_north_label, 1, 3)

        layout.addWidget(QLabel("E:"), 2, 0)
        layout.addWidget(self.__current_east_label, 2, 1)
        layout.addWidget(self.__desired_east_label_edit, 2, 2)
        layout.addWidget(self.__desired_east_label, 2, 3)

        layout.addWidget(QLabel("D:"), 3, 0)
        layout.addWidget(self.__current_down_label, 3, 1)
        layout.addWidget(self.__desired_down_label_edit, 3, 2)
        layout.addWidget(self.__desired_down_label, 3, 3)

        layout.addWidget(QLabel("Heading:"), 4, 0)
        layout.addWidget(self.__current_heading_label, 4, 1)

        layout.addWidget(QLabel("Angle_in_frame:"), 5, 0)
        layout.addWidget(self.__current_angle_in_frame_label, 5, 1)
        layout.addWidget(self.__desired_angle_in_frame_label_edit, 5, 2)
        layout.addWidget(self.__desired_angle_in_frame_label, 5, 3)

        layout.addWidget(self.__update_desired_pose, 6, 0, 1, 4)

        widget.setLayout(layout)
        return widget

    def create_control_gains_layout(self):
        widget = QWidget()
        layout = QGridLayout()

        layout.addWidget(QLabel("Field"), 0, 0)
        layout.addWidget(QLabel("Current Value"), 0, 1)
        layout.addWidget(QLabel("Desired Value Edit"), 0, 2)

        layout.addWidget(QLabel("Pos pgain:"), 1, 0)
        layout.addWidget(self.__position_pgain_label, 1, 1)
        layout.addWidget(self.__position_pgain_label_edit, 1, 2)

        layout.addWidget(QLabel("Pos pgain Z:"), 2, 0)
        layout.addWidget(self.__position_pgain_z_label, 2, 1)
        layout.addWidget(self.__position_pgain_z_label_edit, 2, 2)

        layout.addWidget(QLabel("Vel pgain:"), 3, 0)
        layout.addWidget(self.__velocity_pgain_label, 3, 1)
        layout.addWidget(self.__velocity_pgain_label_edit, 3, 2)

        layout.addWidget(QLabel("Vel igain:"), 4, 0)
        layout.addWidget(self.__velocity_igain_label, 4, 1)
        layout.addWidget(self.__velocity_igain_label_edit, 4, 2)

        layout.addWidget(QLabel("Vel pgain Z:"), 5, 0)
        layout.addWidget(self.__velocity_pgain_z_label, 5, 1)
        layout.addWidget(self.__velocity_pgain_z_label_edit, 5, 2)

        layout.addWidget(QLabel("Vel igain Z:"), 6, 0)
        layout.addWidget(self.__velocity_igain_z_label, 6, 1)
        layout.addWidget(self.__velocity_igain_z_label_edit, 6, 2)

        layout.addWidget(QLabel("Heading pgain:"), 7, 0)
        layout.addWidget(self.__yaw_pgain_label_label, 7, 1)
        layout.addWidget(self.__yaw_pgain_label_edit, 7, 2)

        layout.addWidget(self.__update_control_gains, 8, 0, 1, 3)

        widget.setLayout(layout)
        return widget

    def create_control_targets_layout(self):
        widget = QWidget()
        layout = QGridLayout()

        layout.addWidget(QLabel("Field"), 0, 0)
        layout.addWidget(QLabel("Desired Value"), 0, 1)
        layout.addWidget(QLabel("Current Value"), 0, 2)

        layout.addWidget(QLabel("Velocity N:"), 1, 0)
        layout.addWidget(self.__target_velocity_n_label, 1, 1)
        layout.addWidget(self.__velocity_n_label, 1, 2)

        layout.addWidget(QLabel("Velocity E:"), 2, 0)
        layout.addWidget(self.__target_velocity_e_label, 2, 1)
        layout.addWidget(self.__velocity_e_label, 2, 2)

        layout.addWidget(QLabel("Velocity D:"), 3, 0)
        layout.addWidget(self.__target_velocity_d_label, 3, 1)
        layout.addWidget(self.__velocity_d_label, 3, 2)

        layout.addWidget(QLabel("Accel N:"), 4, 0)
        layout.addWidget(self.__target_acceleration_n_label, 4, 1)

        layout.addWidget(QLabel("Accel E:"), 5, 0)
        layout.addWidget(self.__target_acceleration_e_label, 5, 1)

        layout.addWidget(QLabel("Accel D:"), 6, 0)
        layout.addWidget(self.__target_acceleration_d_label, 6, 1)

        layout.addWidget(QLabel("Roll:"), 7, 0)
        layout.addWidget(self.__target_roll_label, 7, 1)

        layout.addWidget(QLabel("Pitch:"), 8, 0)
        layout.addWidget(self.__target_pitch_label, 8, 1)

        widget.setLayout(layout)
        return widget

    def update_with_target_ned(self, msg):
        self.__current_north_label.setText('%.4f m' % msg.north)
        self.__current_east_label.setText('%.4f m' % msg.east)
        self.__current_down_label.setText('%.4f m' % msg.down)
        self.__current_heading_label.setText('%.4f deg' % msg.yaw)
        self.__current_angle_in_frame_label.setText(
            '%.4f deg' % msg.angle_in_frame)

        self.__desired_north_label.setText('%.4f m' % msg.desired_north)
        self.__desired_east_label.setText('%.4f m' % msg.desired_east)
        self.__desired_down_label.setText('%.4f m' % msg.desired_down)
        self.__desired_angle_in_frame_label.setText(
            '%.4f deg' % msg.desired_angle_in_frame)

    def update_control_gains(self, msg):
        self.__position_pgain_label.setText('%.2f' % msg.pos_pgain)
        self.__position_pgain_z_label.setText('%.2f' % msg.pos_pgain_z)

        self.__velocity_pgain_label.setText('%.2f' % msg.vel_pgain)
        self.__velocity_pgain_z_label.setText('%.2f' % msg.vel_pgain_z)

        self.__velocity_igain_label.setText('%.2f' % msg.vel_igain)
        self.__velocity_igain_z_label.setText('%.2f' % msg.vel_igain_z)

        self.__yaw_pgain_label_label.setText('%.2f' % msg.yaw_pgain)

    def update_control_targets(self, msg):
        self.__target_velocity_n_label.setText('%.2f' % msg.vel_n)
        self.__target_velocity_e_label.setText('%.2f' % msg.vel_e)
        self.__target_velocity_d_label.setText('%.2f' % msg.vel_d)
        self.__velocity_n_label.setText('%.2f' % msg.current_vel_n)
        self.__velocity_e_label.setText('%.2f' % msg.current_vel_e)
        self.__velocity_d_label.setText('%.2f' % msg.current_vel_d)
        self.__target_acceleration_n_label.setText('%.2f' % msg.acc_n)
        self.__target_acceleration_e_label.setText('%.2f' % msg.acc_e)
        self.__target_acceleration_d_label.setText('%.2f' % msg.acc_d)

        self.__target_roll_label.setText('%.2f deg' % msg.roll)
        self.__target_pitch_label.setText('%.2f deg' % msg.pitch)

    def message_handler(self, msg):

        if msg.name == "TARGET_NED":
            self.update_with_target_ned(msg)
        elif msg.name == "CONTROL_GAINS":
            self.update_control_gains(msg)
        elif msg.name == "CONTROL_TARGETS":
            self.update_control_targets(msg)

    def on_message_cb(self, msg):
        self.got_msg_signal.emit(msg)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    m = MavlinkHandler('udpin:localhost:14563', window.on_message_cb)
    window.bind_msg_sender(m)
    window.show()
    app.exec_()
    m.shutdown()
