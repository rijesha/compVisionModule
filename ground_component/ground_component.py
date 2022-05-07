import sys

from PySide2.QtCore import QSize, Qt
from PySide2.QtWidgets import QApplication, QMainWindow, QPushButton, QGridLayout, QVBoxLayout 
from PySide2.QtWidgets import QFrame, QPushButton, QLabel, QLineEdit, QWidget

# Subclass QMainWindow to customize your application's main window
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
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
        self.__target_acceleration_n_label = QLabel("xx")
        self.__target_acceleration_e_label = QLabel("xx")
        self.__target_acceleration_d_label = QLabel("xx")

        self.__target_roll_label = QLabel("xx")
        self.__target_pitch_label = QLabel("xx")

        self.__update_desired_pose = QPushButton("Update Desired Position")
        self.__update_control_gains = QPushButton("Update Control Gains")
        self.__enable_control_gains = QPushButton("Enable Control")
        self.__disable_control_gains = QPushButton("Disable Control")
        self.setWindowTitle("UAV Controller")

        layout = QGridLayout()

        layout.addWidget(self.create_ned_layout(),0,0)
        layout.addWidget(self.create_control_gains_layout(),0,1)
        layout.addWidget(self.create_control_targets_layout(),1,0)
        layout.addWidget(self.__enable_control_gains,2,0)
        layout.addWidget(self.__disable_control_gains,2,1)

        # Set the central widget of the Window.
        widget = QWidget()
        widget.setLayout(layout)
        self.setCentralWidget(widget)

    
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
        
        layout.addWidget(self.__update_desired_pose,6,0,1,4)

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
       
        layout.addWidget(self.__update_control_gains,8,0,1,3)

        widget.setLayout(layout)
        return widget

    def create_control_targets_layout(self):
        widget = QWidget()
        layout = QGridLayout()

        layout.addWidget(QLabel("Field"), 0, 0)
        layout.addWidget(QLabel("Current Value"), 0, 1)
        layout.addWidget(QLabel("Desired Value Edit"), 0, 2)

        layout.addWidget(QLabel("Velocity N:"), 1, 0)
        layout.addWidget(self.__target_velocity_n_label, 1, 1)

        layout.addWidget(QLabel("Velocity E:"), 2, 0)
        layout.addWidget(self.__target_velocity_e_label, 2, 1)
        
        layout.addWidget(QLabel("Velocity D:"), 3, 0)
        layout.addWidget(self.__target_velocity_d_label, 3, 1)
        
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

    def update_with_current_ned(self, msg):
        pass

    def update_with_desired_ned(self, msg):
        pass

    def update_set_control_gains(self, msg):
        pass

    def update_control_loop_intermediaries(self,msg):
        pass

app = QApplication(sys.argv)

window = MainWindow()
window.show()

app.exec_()