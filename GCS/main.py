import sys
import threading
import serial
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QLabel, QPushButton, \
    QSpacerItem, QGridLayout, QProgressBar, QGroupBox, QComboBox, QFileDialog, QGraphicsOpacityEffect, QShortcut, \
    QInputDialog
from PyQt5.QtCore import Qt, pyqtSignal, QObject, QTimer, QPropertyAnimation
from PyQt5.QtGui import QFont, QPixmap, QIcon, QKeySequence, QMovie, QFontDatabase
from communication import Communication
from autogyroRotationGraph import AutoGyroRotationGraph
from temperatureGraph import TemperatureGraph
from altitudeGraph import AltitudeGraph
from rotationGraph import RotationGraph
from voltageGraph import VoltageGraph
from accelerationGraph import AccelerationGraph
from magnetometerGraph import MagnetometerGraph
from GPS import GPSMap
import time
from playsound3 import playsound
import datetime
from serial.tools import list_ports
import shutil
from typing import Iterable

def get_available_serial_ports() -> Iterable[str]:
    return map(lambda c: c.device, list_ports.comports())

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
        loading_pixmap = QPixmap('COSMOS_logo.png')
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
        self.timer.start(50)

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

        self.gif_label = QLabel(self.central_widget)
        self.gif_label.setAlignment(Qt.AlignCenter)
        self.gif_label.setStyleSheet("background: transparent;")
        self.gif_label.hide()

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

        # Load the custom font
        font_id = QFontDatabase.addApplicationFont("SpeedyRegular-7BLoE.ttf")
        if font_id != -1:
            font_family = QFontDatabase.applicationFontFamilies(font_id)[0]
            custom_font = QFont(font_family, 24, QFont.Bold)
            header_text.setFont(custom_font)
        else:
            print("Failed to load Speedy Regular font.")

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
        self.serial_port_dropdown.setStyleSheet("background-color: white;")
        right_header_layout.addWidget(self.serial_port_dropdown)

        # Add baud rate selection dropdown
        self.baud_rate_dropdown = QComboBox()
        self.baud_rate_dropdown.addItems(["110", "300", "600", "1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200"])
        self.baud_rate_dropdown.setCurrentText("115200")  # default baud rate
        self.baud_rate_dropdown.setStyleSheet("background-color: white;")
        self.baud_rate_dropdown.currentIndexChanged.connect(self.change_baud_rate)
        right_header_layout.addWidget(self.baud_rate_dropdown)

        # Add a button to refresh serial ports
        self.refresh_ports_button = QPushButton("Refresh")
        self.refresh_ports_button.clicked.connect(self.update_serial_ports)
        self.refresh_ports_button.setStyleSheet("background-color: white; font-size: 10px;")
        self.refresh_ports_button.setFixedSize(55, 20)
        right_header_layout.addWidget(self.refresh_ports_button)

        # initialize connection to communication
        self.comm = Communication(
            serial_port=self.serial_port_dropdown.currentText())  # Initialize with selected serial port

        # Enclose sats in a white box
        sats_box = QGroupBox()
        sats_box.setStyleSheet("background-color: white;")
        sats_box_layout = QVBoxLayout()
        sats_box.setLayout(sats_box_layout)

        # Create QLabel for text
        self.sats = QLabel("N/A")
        self.sats.setAlignment(Qt.AlignCenter)

        # Create QLabel for the image
        sats_icon = QPixmap('sats_icon.png')
        sats_icon = sats_icon.scaled(25, 25, Qt.KeepAspectRatio)
        sats_icon_label = QLabel()
        sats_icon_label.setPixmap(sats_icon)

        # Create a horizontal layout
        h_layout = QHBoxLayout()
        h_layout.addWidget(sats_icon_label)
        h_layout.addWidget(self.sats)
        sats_box_layout.addLayout(h_layout)

        # Add the sats box to the right header layout
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
        screen_geometry = QApplication.desktop().screenGeometry()
        button_width = screen_geometry.width() // 18
        button_height = screen_geometry.height() // 15
        self.reset_csv_button = QPushButton("Reset\nCSV")
        self.reset_csv_button.clicked.connect(self.comm.reset_csv)
        self.reset_csv_button.clicked.connect(lambda: self.on_button_click(self.reset_csv_button))
        self.reset_csv_button.setStyleSheet("color: black; border: 1px solid black; font-weight: bold; border-radius: 5px; background-color: #a7cbf5;")
        self.reset_csv_button.setFixedWidth(button_width)
        self.reset_csv_button.setFixedHeight(button_height)
        self.set_UTC_time_button = QPushButton("Set\nUTC Time")
        self.set_UTC_time_button.setStyleSheet("color: black; border: 1px solid black; font-weight: bold; border-radius: 5px; background-color: #a7cbf5;")
        self.set_UTC_time_button.clicked.connect(self.set_utc_time)
        self.set_UTC_time_button.clicked.connect(lambda: self.on_button_click(self.set_UTC_time_button))
        self.set_UTC_time_button.setFixedHeight(button_height)
        self.set_UTC_time_button.setFixedWidth(button_width)
        self.set_GPS_time_button = QPushButton("Set\nGPS Time")
        self.set_GPS_time_button.clicked.connect(self.set_gps_time)
        self.set_GPS_time_button.clicked.connect(lambda: self.on_button_click(self.set_GPS_time_button))
        self.set_GPS_time_button.setStyleSheet("color: black; border: 1px solid black; font-weight: bold; border-radius: 5px; background-color: #a7cbf5;")
        self.set_GPS_time_button.setFixedHeight(button_height)
        self.set_GPS_time_button.setFixedWidth(button_width)
        self.SIM_toggle_button = QPushButton("SIM\nEnable")
        self.SIM_toggle_button.clicked.connect(self.toggle_sim)
        self.SIM_toggle_button.clicked.connect(lambda: self.on_button_click(self.SIM_toggle_button))
        self.SIM_toggle_button.setStyleSheet("color: black; border: 1px solid black; font-weight: bold; border-radius: 5px; background-color: #a7cbf5;")
        self.SIM_toggle_button.setFixedHeight(button_height)
        self.SIM_activate_button = QPushButton("SIM\nActivate")
        self.SIM_activate_button.setFixedHeight(button_height)
        self.SIM_activate_button.clicked.connect(self.sim_activate)
        self.SIM_activate_button.clicked.connect(lambda: self.on_button_click(self.SIM_activate_button))
        self.SIM_activate_button.setStyleSheet("color: black; border: 1px solid black; font-weight: bold; border-radius: 5px; background-color: #a7cbf5;")
        self.SIM_activate_button.setFixedHeight(button_height)
        self.CAL_button = QPushButton("CAL")
        self.CAL_button.clicked.connect(self.cal)
        self.CAL_button.clicked.connect(lambda: self.on_button_click(self.CAL_button))
        self.CAL_button.setStyleSheet("color: black; border: 1px solid black; font-weight: bold; padding: 10px 0; border-radius: 5px; background-color: #a7cbf5;")
        self.CAL_button.setFixedHeight(button_height)
        self.RELEASE_on_button = QPushButton("CANISTER\nRELEASE")
        self.RELEASE_on_button.clicked.connect(self.release_on)
        self.RELEASE_on_button.clicked.connect(lambda: self.on_button_click(self.RELEASE_on_button))
        self.RELEASE_on_button.setStyleSheet("color: black; border: 1px solid black; font-weight: bold; border-radius: 5px; background-color: #a7cbf5;")
        self.RELEASE_on_button.setFixedHeight(button_height)
        self.RELEASE_off_button = QPushButton("CANISTER\nLOCK")
        self.RELEASE_off_button.clicked.connect(self.release_off)
        self.RELEASE_off_button.clicked.connect(lambda: self.on_button_click(self.RELEASE_off_button))
        self.RELEASE_off_button.setStyleSheet("color: black; border: 1px solid black; font-weight: bold; border-radius: 5px; background-color: #a7cbf5;")
        self.RELEASE_off_button.setFixedHeight(button_height)
        self.BLADE_CAM_on_button = QPushButton("BLADE CAM\nON")
        self.BLADE_CAM_on_button.clicked.connect(self.blade_cam_on)
        self.BLADE_CAM_on_button.clicked.connect(lambda: self.on_button_click(self.BLADE_CAM_on_button))
        self.BLADE_CAM_on_button.setStyleSheet("color: black; border: 1px solid black; font-weight: bold; border-radius: 5px; background-color: #a7cbf5;")
        self.BLADE_CAM_on_button.setFixedHeight(button_height)
        self.BLADE_CAM_off_button = QPushButton("BLADE CAM\nOFF")
        self.BLADE_CAM_off_button.clicked.connect(self.blade_cam_off)
        self.BLADE_CAM_off_button.clicked.connect(lambda: self.on_button_click(self.BLADE_CAM_off_button))
        self.BLADE_CAM_off_button.setStyleSheet("color: black; border: 1px solid black; font-weight: bold; border-radius: 5px; background-color: #a7cbf5;")
        self.BLADE_CAM_off_button.setFixedHeight(button_height)
        self.GROUND_CAM_on_button = QPushButton("GROUND CAM\nON")
        self.GROUND_CAM_on_button.clicked.connect(self.ground_cam_on)
        self.GROUND_CAM_on_button.clicked.connect(lambda: self.on_button_click(self.GROUND_CAM_on_button))
        self.GROUND_CAM_on_button.setStyleSheet("color: black; border: 1px solid black; font-weight: bold; border-radius: 5px; background-color: #a7cbf5;")
        self.GROUND_CAM_on_button.setFixedHeight(button_height)
        self.GROUND_CAM_off_button = QPushButton("GROUND CAM\nOFF")
        self.GROUND_CAM_off_button.clicked.connect(self.ground_cam_off)
        self.GROUND_CAM_off_button.clicked.connect(lambda: self.on_button_click(self.GROUND_CAM_off_button))
        self.GROUND_CAM_off_button.setStyleSheet("color: black; border: 1px solid black; font-weight: bold; border-radius: 5px; background-color: #a7cbf5;")
        self.GROUND_CAM_off_button.setFixedHeight(button_height)
        self.start_stop_button = QPushButton("CXON")
        self.start_stop_button.clicked.connect(self.toggle_data_transmission)
        self.start_stop_button.clicked.connect(lambda: self.on_button_click(self.start_stop_button))
        self.start_stop_button.setStyleSheet("color: black; border: 1px solid black; font-weight: bold; padding: 10px 0; border-radius: 5px; background-color: #a7cbf5;")
        self.start_stop_button.setFixedHeight(button_height)
        self.calCamStabilization = QPushButton("Cal\nCam Stable")
        self.calCamStabilization.clicked.connect(self.cal_camera_stabilization)
        self.calCamStabilization.clicked.connect(lambda: self.on_button_click(self.calCamStabilization))
        self.calCamStabilization.setStyleSheet("color: black; border: 1px solid black; font-weight: bold; border-radius: 5px; background-color: #a7cbf5;")
        self.calCamStabilization.setFixedHeight(button_height)
        self.copyCSV = QPushButton("Download\nCSV")
        self.copyCSV.clicked.connect(self.copy_csv)
        self.copyCSV.clicked.connect(lambda: self.on_button_click(self.copyCSV))
        self.copyCSV.setStyleSheet("color: black; border: 1px solid black; font-weight: bold; border-radius: 5px; background-color: #a7cbf5;")
        self.copyCSV.setFixedHeight(button_height)

        buttons_grid.addWidget(self.reset_csv_button, 0, 0)
        buttons_grid.addWidget(self.set_UTC_time_button, 0, 1)
        buttons_grid.addWidget(self.set_GPS_time_button, 0, 2)
        buttons_grid.addWidget(self.SIM_toggle_button, 1, 0)
        buttons_grid.addWidget(self.SIM_activate_button, 1, 1)
        buttons_grid.addWidget(self.CAL_button, 1, 2)
        buttons_grid.addWidget(self.RELEASE_on_button, 2, 0)
        buttons_grid.addWidget(self.RELEASE_off_button, 2, 1)
        buttons_grid.addWidget(self.calCamStabilization, 2, 2)
        buttons_grid.addWidget(self.GROUND_CAM_on_button, 3, 0)
        buttons_grid.addWidget(self.GROUND_CAM_off_button, 3, 1)
        buttons_grid.addWidget(self.start_stop_button, 3, 2)
        buttons_grid.addWidget(self.BLADE_CAM_on_button, 4, 0)
        buttons_grid.addWidget(self.BLADE_CAM_off_button, 4, 1)
        buttons_grid.addWidget(self.copyCSV, 4, 2)

        # Create a QGroupBox for GPS coordinates, styled like other telemetry boxes
        self.gps_coords_box = QGroupBox("")
        self.gps_coords_box.setStyleSheet("background-color: #d1d1f0;")
        gps_coords_layout = QVBoxLayout()
        self.gps_coords_box.setLayout(gps_coords_layout)

        self.GPS_LATITUDE = QLabel("GPS Latitude: N/A")
        self.GPS_LATITUDE.setAlignment(Qt.AlignLeft)
        self.GPS_LATITUDE.setStyleSheet("color: black; font-weight: bold;")
        self.GPS_LONGITUDE = QLabel("GPS Longitude: N/A")
        self.GPS_LONGITUDE.setAlignment(Qt.AlignLeft)
        self.GPS_LONGITUDE.setStyleSheet("color: black; font-weight: bold;")

        self.gps_coords_box.setStyleSheet("QGroupBox { background-color: #d1d1f0; border: 1px solid black; }")

        gps_coords_layout.addWidget(self.GPS_LATITUDE)
        gps_coords_layout.addWidget(self.GPS_LONGITUDE)

        # Add the button grid widget to the sidebar
        buttons_widget = QWidget()
        buttons_widget.setLayout(buttons_grid)
        sidebar_layout.addWidget(buttons_widget)

        # Add the GPS coordinates box under the buttons
        sidebar_layout.addWidget(self.gps_coords_box)

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
        self.telemetry = QLabel("Telemetry: N/A")
        self.telemetry.setStyleSheet("color: black; font-weight: bold;")
        footer_layout.addWidget(self.telemetry)

        # Graphs/GPS layout
        graphs_layout = QVBoxLayout()
        graphs_widget = QWidget()
        graphs_widget.setLayout(graphs_layout)
        graphs_widget.setStyleSheet("background-color: #e6e6e6;")  # Set graphs background color

        graphs_grid = QGridLayout()
        screen_geometry = QApplication.desktop().screenGeometry()
        screen_width = screen_geometry.width()
        graph_width = int(screen_width * 0.175)
        self.altitudeGraph = AltitudeGraph()
        self.autoGyroRotationGraph = AutoGyroRotationGraph()
        self.autoGyroRotationGraph.win.setFixedWidth(graph_width)
        self.temperatureGraph = TemperatureGraph()
        self.temperatureGraph.win.setFixedWidth(graph_width)
        self.rotationGraph = RotationGraph()
        self.rotationGraph.win.setFixedWidth(graph_width)
        self.voltageGraph = VoltageGraph()
        self.voltageGraph.win.setFixedWidth(graph_width)
        self.accelerationGraph = AccelerationGraph()
        self.accelerationGraph.win.setFixedWidth(graph_width)
        self.magnetometerGraph = MagnetometerGraph()
        self.magnetometerGraph.win.setFixedWidth(graph_width)

        self.GPS = GPSMap()
        self.GPS.location_updated.connect(self.GPS.update_map)
        self.GPS.update_gui()

        graphs_grid.addWidget(self.altitudeGraph.win, 0, 0)
        graphs_grid.addWidget(self.autoGyroRotationGraph.win, 0, 1)
        graphs_grid.addWidget(self.temperatureGraph.win, 0, 2)
        graphs_grid.addWidget(self.accelerationGraph.win, 0, 3)
        graphs_grid.addWidget(self.rotationGraph.win, 1, 2)
        graphs_grid.addWidget(self.voltageGraph.win, 1, 1)
        graphs_grid.addWidget(self.GPS.win, 1, 0)
        graphs_grid.addWidget(self.magnetometerGraph.win, 1, 3)

        graphs_layout.addLayout(graphs_grid)

        # Add all elements to the main layout
        content_layout.addWidget(sidebar_widget)
        content_layout.addWidget(graphs_widget)
        main_layout.addLayout(content_layout)
        main_layout.addWidget(footer_widget)

        self.showMaximized()
        self.setup_shortcuts()

        # Communication setup
        self.reader_thread = None
        self.reading_data = False

        self.signal_emitter = SignalEmitter()
        self.signal_emitter.update_signal.connect(self.update_graphs)

        # Start timer for updating live data
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_live_data)
        self.timer.start(100)

        # Timer for updating serial ports
        #self.serial_timer = QTimer(self)
        #self.serial_timer.timeout.connect(self.update_serial_ports)
        #self.serial_timer.start(1000)

    def disable_button_temporarily(self, button):
        button.setEnabled(False)
        QTimer.singleShot(1500, lambda: button.setEnabled(True))

    #enables/disables data transmission
    def toggle_data_transmission(self):
        if self.reading_data:
            self.stop_data_transmission()
        else:
            self.start_data_transmission()

    #start data transmission
    def start_data_transmission(self):
        self.disable_button_temporarily(self.start_stop_button)
        self.reading_data = True
        self.start_stop_button.setText("CXOFF")
        self.comm.start_communication(self.signal_emitter)
        self.comm.send_command("CMD,3195,CX,ON")
        threading.Thread(target=playsound, args=('connect.mp3',), daemon=True).start()

    #stop data transmission
    def stop_data_transmission(self):
        self.disable_button_temporarily(self.start_stop_button)
        self.reading_data = False
        self.start_stop_button.setText("CXON")
        self.comm.stop_communication()
        self.comm.send_command("CMD,3195,CX,OFF")
        threading.Thread(target=playsound, args=('disconnect.mp3',), daemon=True).start()

    #send/execute commands
    def setup_shortcuts(self):
        """Setup keyboard shortcuts."""
        release_on_shortcut = QShortcut(QKeySequence("Ctrl+R"), self)
        release_on_shortcut.activated.connect(self.release_on)
        release_off_shortcut = QShortcut(QKeySequence("Ctrl+SHIFT+R"), self)
        release_off_shortcut.activated.connect(self.release_off)
        party_on_shortcut = QShortcut(QKeySequence("Ctrl+P"), self)
        party_on_shortcut.activated.connect(self.party_on)
        party_off_shortcut = QShortcut(QKeySequence("Ctrl+SHIFT+P"), self)
        party_off_shortcut.activated.connect(self.party_off)
        command_shortcut = QShortcut(QKeySequence("Ctrl+/"), self)
        command_shortcut.activated.connect(self.open_command_input)

    def open_command_input(self):
        command, ok = QInputDialog.getText(self, "Command Input", "Enter Command:")
        if ok and command:
            self.comm.send_command(command)

    def set_utc_time(self):
        self.disable_button_temporarily(self.set_UTC_time_button)
        utc_time = datetime.datetime.now().strftime("%H:%M:%S")
        self.comm.send_command(f"CMD,3195,ST,{utc_time}")
    def set_gps_time(self):
        self.disable_button_temporarily(self.set_GPS_time_button)
        self.comm.send_command("CMD,3195,ST,GPS_TIME")
    def toggle_sim(self):
        if self.sim:
            self.sim_disable()
            self.sim = False
        elif not self.sim:
            self.sim_enable()
            self.sim = True
    def sim_enable(self):
        self.disable_button_temporarily(self.SIM_toggle_button)
        self.comm.simEnabled = True
        self.SIM_toggle_button.setText("SIM\nDisable")
        self.comm.send_command("CMD,3195,SIM,ENABLE")
    def sim_activate(self):
        self.disable_button_temporarily(self.SIM_activate_button)
        if self.comm.simEnabled and self.comm.receivedPacketCount:
            self.comm.send_command("CMD,3195,SIM,ACTIVATE")
            csv_filename, _ = QFileDialog.getOpenFileName(self, "Select CSV file", "", "CSV and Text Files (*.csv *.txt)")
            if csv_filename:
                self.comm.simulation_mode(csv_filename)
    def sim_disable(self):
        self.disable_button_temporarily(self.SIM_toggle_button)
        self.comm.simulation = False
        self.comm.stop_simulation()
        self.SIM_toggle_button.setText("SIM\nEnable")
        self.comm.send_command("CMD,3195,SIM,DISABLE")
    def cal(self):
        self.disable_button_temporarily(self.CAL_button)
        self.comm.send_command("CMD,3195,CAL")
    def release_on(self):
        self.disable_button_temporarily(self.RELEASE_on_button)
        self.comm.send_command("CMD,3195,MEC,RELEASE,ON")
    def release_off(self):
        self.disable_button_temporarily(self.RELEASE_off_button)
        self.comm.send_command("CMD,3195,MEC,RELEASE,OFF")
    def blade_cam_on(self):
        self.disable_button_temporarily(self.BLADE_CAM_on_button)
        self.comm.send_command("CMD,3195,MEC,CAMERA,BLADE,ON")
    def blade_cam_off(self):
        self.disable_button_temporarily(self.BLADE_CAM_off_button)
        self.comm.send_command("CMD,3195,MEC,CAMERA,BLADE,OFF")
    def ground_cam_on(self):
        self.disable_button_temporarily(self.GROUND_CAM_on_button)
        self.comm.send_command("CMD,3195,MEC,CAMERA,GROUND,ON")
    def ground_cam_off(self):
        self.disable_button_temporarily(self.GROUND_CAM_off_button)
        self.comm.send_command("CMD,3195,MEC,CAMERA,GROUND,OFF")
    def cal_camera_stabilization(self):
        self.disable_button_temporarily(self.calCamStabilization)
        self.comm.send_command("CMD,3195,MEC,CAMERA,STABLE")

    def party_on(self):
            self.comm.send_command("CMD,3195,PARTY,ON")
            threading.Thread(target=playsound, args=('celebrate.mp3',), daemon=True).start()
            gif = QMovie("party.gif")
            self.gif_label.setMovie(gif)
            self.gif_label.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint | Qt.Tool)
            self.gif_label.setAttribute(Qt.WA_TranslucentBackground)
            self.gif_label.setGeometry(QApplication.desktop().screenGeometry())  # Set to fullscreen geometry
            self.gif_label.show()
            gif.start()
            QTimer.singleShot(10000, self.gif_label.hide)  # Hide the GIF after 10 seconds
    def party_off(self):
        self.comm.send_command("CMD,3195,PARTY,OFF")

    def copy_csv(self):
        destination_folder = QFileDialog.getExistingDirectory(self, "Select Destination Folder")
        if destination_folder:
            source_file = 'Flight_3195.csv'
            destination_file = f"{destination_folder}/Flight_3195.csv"
            try:
                shutil.copy(source_file, destination_file)
                print(f"File copied to {destination_file}")
            except Exception as e:
                print(f"Error copying file: {e}")

    #update telemetry on GCS
    def update_live_data(self):
        if self.comm.receivedPacketCount == 0:
            return

        self.liveMode.setText(f"Mode: {self.comm.get_MODE() or 'N/A'}")
        self.liveState.setText(f"State: {self.comm.get_STATE() or 'N/A'}")
        self.liveMissionTime.setText(f"Mission Time: {self.comm.get_MISSION_TIME() or 'N/A'}")
        self.liveGPSTime.setText(f"GPS Time: {self.comm.get_GPS_TIME() or 'N/A'}")
        self.livePacketCount.setText(f"Packet Count: {self.comm.get_PACKET_COUNT() or 'N/A'}")
        self.liveReceivedPackets.setText(f"Received Packets: {self.comm.receivedPacketCount or 'N/A'}")
        self.liveGPSAltitude.setText(f"GPS Altitude: {self.comm.get_GPS_ALTITUDE() or 'N/A'} m")
        self.gyroRotation.setText(f"Gyro Rate: {self.comm.get_AUTO_GYRO_ROTATION_RATE() or 'N/A'} °/s²")
        self.liveCMDEcho.setText(f"CMD Echo: {self.comm.get_CMD_ECHO() or 'N/A'}")

        self.GPS_LATITUDE.setText(f"GPS Latitude: {self.comm.get_GPS_LATITUDE() or 'N/A'}")
        self.GPS_LONGITUDE.setText(f"GPS Longitude: {self.comm.get_GPS_LONGITUDE() or 'N/A'}")

        self.telemetry.setText(f"Telemetry: {self.comm.lastPacket or 'N/A'}")
        self.sats.setText(f"{self.comm.get_GPS_SATS() or 'N/A'}")

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
        accel_r = self.comm.get_ACCEL_R()
        accel_p = self.comm.get_ACCEL_P()
        accel_y = self.comm.get_ACCEL_Y()
        if accel_r is not None and accel_p is not None and accel_y is not None:
            self.accelerationGraph.update_graph(accel_r, accel_p, accel_y, current_time)
        mag_r = self.comm.get_MAG_R()
        mag_p = self.comm.get_MAG_P()
        mag_y = self.comm.get_MAG_Y()
        if mag_r is not None and mag_p is not None and mag_y is not None:
            self.magnetometerGraph.update_graph(mag_r, mag_p, mag_y, current_time)

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
        if selected_port != self.comm.serial_port:
            self.comm.stop_communication()
            self.comm.serial_port = selected_port
            try:
                self.comm.ser = serial.Serial(selected_port, self.comm.baud_rate, timeout=self.comm.timeout)
                print(f"Serial port changed to {selected_port}")
                if self.reading_data:
                    self.comm.start_communication(self.signal_emitter)
            except serial.SerialException as e:
                print(f"Failed to open serial port {selected_port}: {e}")
                self.comm.ser = None

    def update_serial_ports(self):
        current_port = self.serial_port_dropdown.currentText()
        available_ports = set(get_available_serial_ports())
        self.serial_port_dropdown.clear()
        self.serial_port_dropdown.addItems(available_ports)
        if current_port in available_ports:
            self.serial_port_dropdown.setCurrentText(current_port)
        else:
            self.comm.stop_communication()
            self.reading_data = False
            self.start_stop_button.setText("CXON")
        print("Serial ports updated.")

    def change_baud_rate(self):
        selected_baud_rate = int(self.baud_rate_dropdown.currentText())
        self.comm.change_baud_rate(selected_baud_rate)
        print(f"Baud rate changed to {selected_baud_rate}")

    def on_button_click(self, button):
        self.change_button_color(button, "#d1d1f0", 500)

    def change_button_color(self, button, color, duration):
        original_style = button.styleSheet()

        if not hasattr(button, "_color_timer"):
            button._color_timer = QTimer()

        button._color_timer.stop()  # Stop any active timer
        button.setStyleSheet(original_style)  # Reset to the original style

        button.setStyleSheet(f"color: black; border: 1px solid black; font-weight: bold; border-radius: 5px; background-color: {color};")
        button._color_timer.singleShot(duration, lambda: button.setStyleSheet(original_style))

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
