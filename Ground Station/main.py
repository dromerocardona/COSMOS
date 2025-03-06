import sys
import threading
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QLabel, QPushButton, \
    QSpacerItem, QGridLayout, QProgressBar, QGroupBox, QComboBox, QFileDialog, QGraphicsOpacityEffect
from PyQt5.QtCore import Qt, pyqtSignal, QObject, QTimer, QPropertyAnimation
from PyQt5.QtGui import QFont, QPixmap, QIcon
from communication import Communication
from autogyroRotationGraph import AutoGyroRotationGraph
from temperatureGraph import TemperatureGraph
from altitudeGraph import AltitudeGraph
from rotationGraph import RotationGraph
from voltageGraph import VoltageGraph
from GPS import GPSMap
import time
from playsound3 import playsound
import datetime
import serial

def get_available_serial_ports():
    ports = [f"COM{i}" for i in range(1, 20)]
    available_ports = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            available_ports.append(port)
        except (OSError, serial.SerialException):
            pass
    return available_ports

# Loading screen
class LoadingScreen(QWidget):
    def center_window(self):
        screen_geometry = QApplication.desktop().screenGeometry()
        x = (screen_geometry.width() - self.width()) // 2
        y = (screen_geometry.height() - self.height()) // 2
        self.move(x, y)

    def __init__(self):
        super().__init__()

        # Set title, size, icon
        self.setWindowTitle("Loading...")
        self.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)
        self.setGeometry(100, 100, 1200, 800)
        self.setWindowIcon(QIcon('COSMOS_logo.png'))
        threading.Thread(target=playsound, args=('gcs_startup.mp3',), daemon=True).start()

        # Set background image and color
        self.setStyleSheet("""
            QWidget {
                background-image: url('loading_background.png');
                background-repeat: no-repeat;
                background-position: center;
                background size: cover;
            }
        """)

        layout = QVBoxLayout(self)
        layout.setSpacing(0)
        layout.setContentsMargins(0, 0, 0, 0)

        # Add loading image
        central_widget = QWidget(self)
        central_layout = QVBoxLayout(central_widget)
        central_layout.setSpacing(0)
        central_layout.setContentsMargins(0, 0, 0, 0)

        # Add loading image
        loading_pixmap = QPixmap('COSMOS_logo.png')  # Update with the correct path to your image
        self.image_width, image_height = 605, 500
        loading_pixmap = loading_pixmap.scaled(self.image_width, image_height)
        self.image_label = QLabel()
        self.image_label.setPixmap(loading_pixmap)
        self.image_label.setAlignment(Qt.AlignCenter)
        central_layout.addWidget(self.image_label)

        layout.addWidget(central_widget)

        # Add progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setValue(0)
        self.progress_bar.setStyleSheet("""
            QProgressBar {
                border: 2px solid #8f8f91;
                border-radius: 5px;
                text-align: center;
            }
            QProgressBar::chunk {
                background-color: #b0aee7;
                width: 20px;
            }
        """)
        layout.addWidget(self.progress_bar)

        # Start timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_progress)
        self.timer.start(100)

        # Add fade-in effect
        self.opacity_effect = QGraphicsOpacityEffect()
        self.setGraphicsEffect(self.opacity_effect)
        self.opacity_animation = QPropertyAnimation(self.opacity_effect, b"opacity")
        self.opacity_animation.setDuration(2000)
        self.opacity_animation.setStartValue(0)
        self.opacity_animation.setEndValue(1)
        self.opacity_animation.start()

        self.center_window()

    # Update progress bar
    def update_progress(self):
        current_value = self.progress_bar.value()
        if current_value < 100:
            self.progress_bar.setValue(current_value + 5)
        else:
            self.timer.stop()
            self.close()

# Used for updating graphs, telemetry, etc.
class SignalEmitter(QObject):
    update_signal = pyqtSignal()

    def emit_signal(self) -> None:
        self.update_signal.emit()

# Main GCS window
class GroundStation(QMainWindow):
    def __init__(self):
        super().__init__()
        # Set title, size, icon
        self.release = None
        self.blade_cam = None
        self.ground_cam = None
        self.sim = None
        self.setWindowTitle("COSMOS GS")
        self.setGeometry(100, 100, 1200, 800)
        self.setWindowIcon(QIcon('COSMOS_logo.png'))
        self.setStyleSheet("background-color: #b0aee7;")

        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)

        main_layout = QVBoxLayout(self.central_widget)

        header_layout = QHBoxLayout()

        self.sats = QLabel("N/A")

        # Left side of the header
        left_header_layout = QHBoxLayout()
        logo_label = QLabel()
        logo_pixmap = QPixmap('COSMOS_logo.png')
        logo_pixmap = logo_pixmap.scaled(60, 50, Qt.KeepAspectRatio)
        logo_label.setPixmap(logo_pixmap)
        team_label = QLabel("TEAM #3195")
        team_label.setFont(QFont("Arial", 16, QFont.Bold))
        team_label.setStyleSheet("color: white;")
        left_header_layout.addWidget(logo_label)
        left_header_layout.addWidget(team_label)

        # Centered header text
        header_text = QLabel("COSMOS GROUND STATION")
        header_text.setAlignment(Qt.AlignCenter)
        header_text.setFont(QFont("Arial", 24, QFont.Bold))
        header_text.setStyleSheet("color: white;")

        # Add left layout and header text to the main header layout
        header_layout.addLayout(left_header_layout)
        header_layout.addStretch()
        header_layout.addWidget(header_text)
        header_layout.addStretch()

        # Right side of the header
        right_header_layout = QHBoxLayout()
        right_header_layout.setAlignment(Qt.AlignRight)
        right_header_layout.setSpacing(10)

        # Add serial port selection dropdown
        self.serial_port_dropdown = QComboBox()
        self.serial_port_dropdown.addItems(get_available_serial_ports())
        self.serial_port_dropdown.currentIndexChanged.connect(self.change_serial_port)
        right_header_layout.addWidget(self.serial_port_dropdown)

        # Enclose sats in a white box
        sats_box = QGroupBox()
        sats_box.setStyleSheet("background-color: white;")
        sats_box_layout = QVBoxLayout()
        sats_box.setLayout(sats_box_layout)

        # Enclose sats in a white box
        sats_box = QGroupBox()
        sats_box.setStyleSheet("background-color: white;")
        sats_box_layout = QVBoxLayout()
        sats_box.setLayout(sats_box_layout)

        self.sats = QLabel("N/A")
        self.sats.setAlignment(Qt.AlignCenter)
        sats_icon = QPixmap('sats_icon.png')
        sats_icon = sats_icon.scaled(25, 25, Qt.KeepAspectRatio)
        sats_icon_label = QLabel()
        sats_icon_label.setPixmap(sats_icon)

        sats_box_layout.addWidget(sats_icon_label)
        sats_box_layout.addWidget(self.sats)

        right_header_layout.addWidget(sats_box)
        header_layout.addLayout(right_header_layout)

        header_widget = QWidget()
        header_widget.setLayout(header_layout)
        header_widget.setStyleSheet("background-color: #545454;")  # Set header background color
        main_layout.addWidget(header_widget)

        # Add content elements
        content_layout = QHBoxLayout()

        # Add sidebar
        sidebar_layout = QVBoxLayout()
        sidebar_layout.setAlignment(Qt.AlignTop)
        sidebar_layout.setSpacing(10)
        sidebar_layout.addItem(QSpacerItem(10, 10))

        # Create a QGroupBox for the sidebar elements
        sidebar_groupbox = QGroupBox("")
        sidebar_groupbox.setStyleSheet("background-color: #d1d1f0;")  # Set the background color of the group box
        sidebar_groupbox_layout = QVBoxLayout()
        sidebar_groupbox.setLayout(sidebar_groupbox_layout)

        # Create a grid layout for the command buttons
        buttons_grid = QGridLayout()
        buttons_grid.setAlignment(Qt.AlignCenter)

        # Add sidebar elements to the group box layout
        self.liveMode = QLabel("Mode: N/A")
        self.liveMode.setFont(QFont("Arial", 11, QFont.Bold))
        self.liveMode.setStyleSheet("color: black; font-weight: bold;")
        self.liveState = QLabel("State: N/A")
        self.liveState.setFont(QFont("Arial", 11, QFont.Bold))
        self.liveState.setStyleSheet("color: black; font-weight: bold;")
        self.liveMissionTime = QLabel("Mission Time: N/A")
        self.liveMissionTime.setFont(QFont("Arial", 11, QFont.Bold))
        self.liveMissionTime.setStyleSheet("color: black; font-weight: bold;")
        self.liveGPSTime = QLabel("GPS Time: N/A")
        self.liveGPSTime.setFont(QFont("Arial", 11, QFont.Bold))
        self.liveGPSTime.setStyleSheet("color: black; font-weight: bold;")
        self.livePacketCount = QLabel("Packet Count: N/A")
        self.livePacketCount.setFont(QFont("Arial", 11, QFont.Bold))
        self.livePacketCount.setStyleSheet("color: black; font-weight: bold;")
        self.liveReceivedPackets = QLabel("Received Packets: N/A")
        self.liveReceivedPackets.setFont(QFont("Arial", 11, QFont.Bold))
        self.liveReceivedPackets.setStyleSheet("color: black; font-weight: bold;")
        self.liveGPSAltitude = QLabel("GPS Altitude: N/A")
        self.liveGPSAltitude.setFont(QFont("Arial", 11, QFont.Bold))
        self.liveGPSAltitude.setStyleSheet("color: black; font-weight: bold;")
        self.gyroRotation = QLabel("Gyro Rotation: N/A")
        self.gyroRotation.setFont(QFont("Arial", 11, QFont.Bold))
        self.gyroRotation.setStyleSheet("color: black; font-weight: bold;")
        self.liveCMDEcho = QLabel("CMD Echo: N/A")
        self.liveCMDEcho.setFont(QFont("Arial", 11, QFont.Bold))
        self.liveCMDEcho.setStyleSheet("color: black; font-weight: bold;")
        sidebar_groupbox_layout.addWidget(self.liveMode)
        sidebar_groupbox_layout.addWidget(self.liveState)
        sidebar_groupbox_layout.addWidget(self.liveMissionTime)
        sidebar_groupbox_layout.addWidget(self.liveGPSTime)
        sidebar_groupbox_layout.addWidget(self.livePacketCount)
        sidebar_groupbox_layout.addWidget(self.liveReceivedPackets)
        sidebar_groupbox_layout.addWidget(self.liveGPSAltitude)
        sidebar_groupbox_layout.addWidget(self.gyroRotation)
        sidebar_groupbox_layout.addWidget(self.liveCMDEcho)

        # Add the QGroupBox to the sidebar layout
        sidebar_groupbox.setStyleSheet("QGroupBox { background-color: #d1d1f0; border: 1px solid black; }")
        sidebar_layout.addWidget(sidebar_groupbox)

        # Add command buttons to sidebar layout
        button_width = 100
        button_height = 80
        self.reset_graphs_button = QPushButton("Reset\nGraphs")
        self.reset_graphs_button.clicked.connect(self.reset_graphs)
        self.reset_graphs_button.setStyleSheet("color: black; border: 1px solid black; font-weight: bold; border-radius: 5px; background-color: #a7cbf5;")
        self.reset_graphs_button.setFixedWidth(button_width)
        self.reset_graphs_button.setFixedHeight(button_height)
        self.set_UTC_time_button = QPushButton("Set\nUTC Time")
        self.set_UTC_time_button.setStyleSheet("color: black; border: 1px solid black; font-weight: bold; border-radius: 5px; background-color: #a7cbf5;")
        self.set_UTC_time_button.clicked.connect(self.set_utc_time)
        self.set_UTC_time_button.setFixedHeight(button_height)
        self.set_UTC_time_button.setFixedWidth(button_width)
        self.set_GPS_time_button = QPushButton("Set\nGPS Time")
        self.set_GPS_time_button.clicked.connect(self.set_gps_time)
        self.set_GPS_time_button.setStyleSheet("color: black; border: 1px solid black; font-weight: bold; border-radius: 5px; background-color: #a7cbf5;")
        self.set_GPS_time_button.setFixedHeight(button_height)
        self.set_GPS_time_button.setFixedWidth(button_width)
        self.SIM_toggle_button = QPushButton("SIM\nEnable")
        self.SIM_toggle_button.clicked.connect(self.toggle_sim)
        self.SIM_toggle_button.setStyleSheet("color: black; border: 1px solid black; font-weight: bold; border-radius: 5px; background-color: #a7cbf5;")
        self.SIM_toggle_button.setFixedHeight(button_height)
        self.SIM_activate_button = QPushButton("SIM\nActivate")
        self.SIM_activate_button.setFixedHeight(button_height)
        self.SIM_activate_button.clicked.connect(self.sim_activate)
        self.SIM_activate_button.setStyleSheet("color: black; border: 1px solid black; font-weight: bold; border-radius: 5px; background-color: #a7cbf5;")
        self.SIM_activate_button.setFixedHeight(button_height)
        self.CAL_button = QPushButton("CAL")
        self.CAL_button.clicked.connect(self.cal)
        self.CAL_button.setStyleSheet("color: black; border: 1px solid black; font-weight: bold; padding: 10px 0; border-radius: 5px; background-color: #a7cbf5;")
        self.CAL_button.setFixedHeight(button_height)
        self.RELEASE_toggle = QPushButton("CANISTER\nRELEASE ON")
        self.RELEASE_toggle.clicked.connect(self.toggle_release)
        self.RELEASE_toggle.setStyleSheet("color: black; border: 1px solid black; font-weight: bold; border-radius: 5px; background-color: #a7cbf5;")
        self.RELEASE_toggle.setFixedHeight(button_height)
        self.BLADE_CAM_toggle = QPushButton("BLADE CAM\nON")
        self.BLADE_CAM_toggle.clicked.connect(self.toggle_blade_cam)
        self.BLADE_CAM_toggle.setStyleSheet("color: black; border: 1px solid black; font-weight: bold; border-radius: 5px; background-color: #a7cbf5;")
        self.BLADE_CAM_toggle.setFixedHeight(button_height)
        self.GROUND_CAM_toggle = QPushButton("GROUND CAM\nON")
        self.GROUND_CAM_toggle.clicked.connect(self.toggle_ground_cam)
        self.GROUND_CAM_toggle.setStyleSheet("color: black; border: 1px solid black; font-weight: bold; border-radius: 5px; background-color: #a7cbf5;")
        self.GROUND_CAM_toggle.setFixedHeight(button_height)
        self.start_stop_button = QPushButton("CXON")
        self.start_stop_button.clicked.connect(self.toggle_data_transmission)
        self.start_stop_button.setStyleSheet("color: black; border: 1px solid black; font-weight: bold; padding: 10px 0; border-radius: 5px; background-color: #a7cbf5;")
        self.start_stop_button.setFixedHeight(button_height)
        self.calCamStabilization = QPushButton("Cal\nCam Stable")
        self.calCamStabilization.clicked.connect(self.cal_camera_stabilization)
        self.calCamStabilization.setStyleSheet("color: black; border: 1px solid black; font-weight: bold; border-radius: 5px; background-color: #a7cbf5;")
        self.calCamStabilization.setFixedHeight(button_height)

        buttons_grid.addWidget(self.reset_graphs_button, 0, 0)
        buttons_grid.addWidget(self.set_UTC_time_button, 0, 1)
        buttons_grid.addWidget(self.set_GPS_time_button, 0, 2)
        buttons_grid.addWidget(self.SIM_toggle_button, 1, 0)
        buttons_grid.addWidget(self.SIM_activate_button, 1, 1)
        buttons_grid.addWidget(self.CAL_button, 1, 2)
        buttons_grid.addWidget(self.RELEASE_toggle, 2, 0)
        buttons_grid.addWidget(self.BLADE_CAM_toggle, 3, 1)
        buttons_grid.addWidget(self.GROUND_CAM_toggle, 3, 2)
        buttons_grid.addWidget(self.start_stop_button, 2, 2)
        buttons_grid.addWidget(self.calCamStabilization, 2, 1)

        # Create a widget for the buttons grid and add it to the sidebar layout
        buttons_widget = QWidget()
        buttons_widget.setLayout(buttons_grid)
        sidebar_layout.addWidget(buttons_widget)

        # Add the sidebar widget to the content layout
        sidebar_widget = QWidget()
        sidebar_widget.setLayout(sidebar_layout)
        sidebar_widget.setStyleSheet("background-color: #e9eeff;")

        # Add footer
        footer_layout = QVBoxLayout()
        footer_widget = QWidget()
        footer_widget.setLayout(footer_layout)
        footer_widget.setStyleSheet("background-color: #a7cbf5;")  # Set footer background color

        # Footer data
        self.accelerationRPY = QLabel("Acceleration R,P,Y: N/A")
        self.accelerationRPY.setStyleSheet("color: black; font-weight: bold;")
        self.magnetometerRPY = QLabel("Magnetometer R,P,Y: N/A")
        self.magnetometerRPY.setStyleSheet("color: black; font-weight: bold;")
        self.GPS_LATITUDE = QLabel("GPS Latitude: N/A")
        self.GPS_LATITUDE.setStyleSheet("color: black; font-weight: bold;")
        self.GPS_LONGITUDE = QLabel("GPS Longitude: N/A")
        self.GPS_LONGITUDE.setStyleSheet("color: black; font-weight: bold;")
        self.telemetry = QLabel("Telemetry: N/A")
        self.telemetry.setStyleSheet("color: black; font-weight: bold;")
        footer_layout.addWidget(self.telemetry)

        # Graphs/GPS layout
        graphs_layout = QVBoxLayout()
        graphs_widget = QWidget()
        graphs_widget.setLayout(graphs_layout)
        graphs_widget.setStyleSheet("background-color: #e6e6e6;")  # Set graphs background color

        graphs_grid = QGridLayout()
        self.altitudeGraph = AltitudeGraph()
        self.autoGyroRotationGraph = AutoGyroRotationGraph()
        self.temperatureGraph = TemperatureGraph()
        self.rotationGraph = RotationGraph()
        self.voltageGraph = VoltageGraph()
        self.GPS = GPSMap()
        self.GPS.location_updated.connect(self.GPS.update_map)
        self.GPS.update_gui()
        graphs_grid.addWidget(self.altitudeGraph.win, 0, 0)
        graphs_grid.addWidget(self.autoGyroRotationGraph.win, 0, 1)
        graphs_grid.addWidget(self.temperatureGraph.win, 0, 2)
        graphs_grid.addWidget(self.rotationGraph.win, 1, 0)
        graphs_grid.addWidget(self.voltageGraph.win, 1, 1)
        graphs_grid.addWidget(self.GPS.win, 1, 2)
        graphs_grid.addWidget(self.accelerationRPY, 3, 0)
        latitude_longitude_layout = QHBoxLayout()
        latitude_longitude_layout.addWidget(self.GPS_LATITUDE)
        latitude_longitude_layout.addWidget(self.GPS_LONGITUDE)
        graphs_grid.addLayout(latitude_longitude_layout, 3, 2, alignment=Qt.AlignCenter)
        graphs_grid.addWidget(self.magnetometerRPY, 3, 1)

        # Set the style for the labels to be in a colored box
        self.accelerationRPY.setStyleSheet("background-color: #d1d1f0; padding: 5px; border-radius: 5px; border: 1px solid black;")
        self.GPS_LATITUDE.setStyleSheet("background-color: #d1d1f0; padding: 5px; border-radius: 5px; border: 1px solid black;")
        self.magnetometerRPY.setStyleSheet("background-color: #d1d1f0; padding: 5px; border-radius: 5px; border: 1px solid black;")
        self.GPS_LONGITUDE.setStyleSheet("background-color: #d1d1f0; padding: 5px; border-radius: 5px; border: 1px solid black;")

        graphs_layout.addLayout(graphs_grid)

        # Add all elements to main layout
        content_layout.addWidget(sidebar_widget)
        content_layout.addWidget(graphs_widget)
        main_layout.addLayout(content_layout)
        main_layout.addWidget(footer_widget)

        self.showMaximized()

        # Communication setup
        self.comm = Communication(serial_port=self.serial_port_dropdown.currentText())  # Initialize with selected serial port
        self.reader_thread = None
        self.reading_data = False

        self.signal_emitter = SignalEmitter()
        self.signal_emitter.update_signal.connect(self.update_graphs)

        # Start timer for updating live data
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_live_data)
        self.timer.start(1000)

    #enables/disables data transmission
    def toggle_data_transmission(self):
        if self.reading_data:
            self.stop_data_transmission()
        else:
            self.start_data_transmission()

    #start data transmission
    def start_data_transmission(self):
        self.reading_data = True
        self.start_stop_button.setText("CXOFF")
        self.comm.send_command("CMD,3195,CX,ON")
        self.comm.start_communication(self.signal_emitter)

    #stop data transmission
    def stop_data_transmission(self):
        self.reading_data = False
        self.start_stop_button.setText("CXON")
        self.comm.send_command("CMD,3195,CX,OFF")
        self.comm.stop_communication()

    #send/execute commands
    def set_utc_time(self):
        utc_time = datetime.datetime.now().strftime("%H:%M:%S")
        self.comm.send_command(f"CMD,3195,ST,{utc_time}")
    def set_gps_time(self):
        self.comm.send_command("CMD,3195,ST,GPS_TIME")
    def toggle_sim(self):
        if self.sim:
            self.sim_disable()
            self.sim = False
        elif not self.sim:
            self.sim_enable()
            self.sim = True
    def sim_enable(self):
        self.comm.simEnabled = True
        self.SIM_toggle_button.setText("SIM\nDisable")
        self.comm.send_command("CMD,3195,SIM,ENABLE")
    def sim_activate(self):
        if self.comm.simEnabled:
            self.comm.send_command("CMD,3195,SIM,ACTIVATE")
            csv_filename, _ = QFileDialog.getOpenFileName(self, "Select CSV file", "", "CSV Files (*.csv)")
            if csv_filename:
                self.comm.simulation_mode(csv_filename)
    def sim_disable(self):
        self.comm.simulation = False
        self.SIM_toggle_button.setText("SIM\nEnable")
        self.comm.send_command("CMD,3195,SIM,DISABLE")
    def cal(self):
        self.comm.send_command("CMD,3195,CAL")
    def toggle_release(self):
        if self.release:
            self.release_off()
            self.release = False
        elif not self.release:
            self.release_on()
            self.release = True
    def release_on(self):
        self.RELEASE_toggle.setText("CANISTER\nRELEASE OFF")
        self.comm.send_command("CMD,3195,MEC,RELEASE,ON")
    def release_off(self):
        self.RELEASE_toggle.setText("CANISTER\nRELEASE ON")
        self.comm.send_command("CMD,3195,MEC,RELEASE,OFF")
    def toggle_blade_cam(self):
        if self.blade_cam:
            self.blade_cam_off()
            self.blade_cam = False
        elif not self.blade_cam:
            self.blade_cam_on()
            self.blade_cam = True
    def blade_cam_on(self):
        self.BLADE_CAM_toggle.setText("BLADE CAM\nOFF")
        self.comm.send_command("CMD,3195,MEC,CAM,BLADE,ON")
    def blade_cam_off(self):
        self.BLADE_CAM_toggle.setText("BLADE CAM\nON")
        self.comm.send_command("CMD,3195,MEC,CAM,BLADE,OFF")
    def toggle_ground_cam(self):
        if self.ground_cam:
            self.ground_cam_off()
            self.ground_cam = False
        elif not self.ground_cam:
            self.ground_cam_on()
            self.ground_cam = True
    def ground_cam_on(self):
        self.GROUND_CAM_toggle.setText("GROUND CAM\nOFF")
        self.comm.send_command("CMD,3195,MEC,CAM,GROUND,ON")
    def ground_cam_off(self):
        self.GROUND_CAM_toggle.setText("GROUND CAM\nON")
        self.comm.send_command("CMD,3195,MEC,CAM,GROUND,OFF")
    def cal_camera_stabilization(self):
        self.comm.send_command("CMD,3195,MEC,CAM_STABLE")

    #update telemetry on GCS
    def update_live_data(self):
        self.liveMode.setText(f"Mode: {self.comm.get_MODE() or 'N/A'}")
        self.liveState.setText(f"State: {self.comm.get_STATE() or 'N/A'}")
        self.liveMissionTime.setText(f"Mission Time: {self.comm.get_MISSION_TIME() or 'N/A'}")
        self.liveGPSTime.setText(f"GPS Time: {self.comm.get_GPS_TIME() or 'N/A'}")
        self.livePacketCount.setText(f"Packet Count: {self.comm.get_PACKET_COUNT() or 'N/A'}")
        self.liveReceivedPackets.setText(f"Received Packets: {self.comm.receivedPacketCount or 'N/A'}")
        self.liveGPSAltitude.setText(f"GPS Altitude: {self.comm.get_GPS_ALTITUDE() or 'N/A'} m")
        self.gyroRotation.setStyleSheet(f"Gyro Rate: {self.comm.get_AUTO_GYRO_ROTATION_RATE() or 'N/A'} °/s")
        self.liveCMDEcho.setText(f"CMD Echo: {self.comm.get_CMD_ECHO() or 'N/A'}")

        self.accelerationRPY.setText(f"Acceleration R,P,Y: {f"{self.comm.get_ACCEL_R()}, {self.comm.get_ACCEL_P()}, {self.comm.get_ACCEL_Y()}" or 'N/A'}")
        self.magnetometerRPY.setText(f"Magnetometer R,P,Y: {f"{self.comm.get_MAG_R()}, {self.comm.get_MAG_P()}, {self.comm.get_MAG_Y()}" or 'N/A'}")
        self.GPS_LATITUDE.setText(f"GPS Latitude: {self.comm.get_GPS_LATITUDE() or 'N/A'}")
        self.GPS_LONGITUDE.setText(f"GPS Longitude: {self.comm.get_GPS_LONGITUDE() or 'N/A'}")

        self.telemetry.setText(f"Telemetry: {self.comm.lastPacket or 'N/A'}")

        #update GPS data
        latitude = self.comm.get_GPS_LATITUDE()
        longitude = self.comm.get_GPS_LONGITUDE()
        if latitude is not None and longitude is not None:
            self.GPS.location_updated.connect(self.GPS.update_map)
            self.GPS.location_updated.emit(latitude, longitude)

    #update graphs
    def update_graphs(self):
        current_time = time.time()

        altitude = self.comm.get_ALTITUDE()
        if altitude is not None:
            self.altitudeGraph.update_graph(altitude, current_time)
        autogyro_rotation_rate = self.comm.get_AUTO_GYRO_ROTATION_RATE()
        if autogyro_rotation_rate is not None:
            self.autoGyroRotationGraph.update_graph(autogyro_rotation_rate, current_time)
        temperature = self.comm.get_TEMPERATURE()
        if temperature is not None:
            self.temperatureGraph.update_graph(temperature, current_time)
        gyro_r = self.comm.get_GYRO_R()
        gyro_p = self.comm.get_GYRO_P()
        gyro_y = self.comm.get_GYRO_Y()
        if gyro_r is not None and gyro_p is not None and gyro_y is not None:
            self.rotationGraph.update_graph(gyro_r, gyro_p, gyro_y, current_time)
        voltage = self.comm.get_VOLTAGE()
        if voltage is not None:
            self.voltageGraph.update_graph(voltage, current_time)

        latitude = self.comm.get_GPS_LATITUDE()
        longitude = self.comm.get_GPS_LONGITUDE()
        if latitude is not None and longitude is not None:
            self.GPS.location_updated.emit(latitude, longitude)
            self.GPS.update_map(latitude, longitude)
            self.GPS.update_gui()

    #reset graphs
    def reset_graphs(self):
        self.altitudeGraph.reset_graph()
        self.autoGyroRotationGraph.reset_graph()
        self.temperatureGraph.reset_graph()
        self.rotationGraph.reset_graph()
        self.voltageGraph.reset_graph()

    def change_serial_port(self):
        selected_port = self.serial_port_dropdown.currentText()
        self.comm.serial_port = selected_port
        print(f"Serial port changed to {selected_port}")

    #close window/program
    def closeEvent(self, event):
        self.stop_data_transmission()
        event.accept()

#Run the application
if __name__ == "__main__":
    app = QApplication(sys.argv)

    loading_screen = LoadingScreen()
    loading_screen.show()

    for i in range(100):
        app.processEvents()
        time.sleep(0.01)

    main_window = GroundStation()
    main_window.show()

    sys.exit(app.exec_())
