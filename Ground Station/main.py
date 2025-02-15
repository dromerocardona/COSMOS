import sys
import threading
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QLabel, QPushButton, \
    QSpacerItem, QSizePolicy, QGridLayout, QProgressBar
from PyQt5.QtCore import Qt, pyqtSignal, QObject, QTimer
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

# Loading screen
class LoadingScreen(QWidget):
    def __init__(self):
        super().__init__()

        #Set title, size, icon
        self.setWindowTitle("Loading...")
        self.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)
        self.setGeometry(100, 100, 1200, 800)
        self.setWindowIcon(QIcon('COSMOS_logo.png'))

        layout = QVBoxLayout()

        #Add loading image
        loading_pixmap = QPixmap('COSMOS_logo.png')  # Update with the correct path to your image
        self.image_width,image_height = 300,300
        loading_pixmap = loading_pixmap.scaled(self.image_width, image_height)
        self.image_label = QLabel()
        self.image_label.setPixmap(loading_pixmap)
        self.image_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.image_label)

        #Add loading text
        self.label = QLabel("Loading...")
        self.label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.label)

        #Add progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setValue(0)
        layout.addWidget(self.progress_bar)

        self.setLayout(layout)

        #Start timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_progress)
        self.timer.start(100)

    #Update progress bar
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
        #Set title, size, icon
        self.setWindowTitle("COSMOS GS")
        self.setGeometry(100, 100, 1200, 800)
        self.setWindowIcon(QIcon('COSMOS_logo.png'))
        self.setStyleSheet("background-color: white;")

        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)

        main_layout = QVBoxLayout(self.central_widget)

        header_layout = QHBoxLayout()

        #add header elements
        header_text = QLabel("COSMOS GROUND STATION")
        header_text.setAlignment(Qt.AlignCenter)
        header_text.setFont(QFont("Arial", 24, QFont.Bold))
        header_text.setStyleSheet("color: black;")
        header_layout.addWidget(header_text)

        header_widget = QWidget()
        header_widget.setLayout(header_layout)
        main_layout.addWidget(header_widget)

        #add content elements
        content_layout = QHBoxLayout()

        #add sidebar
        sidebar_layout = QVBoxLayout()
        sidebar_layout.setAlignment(Qt.AlignTop)
        sidebar_layout.setSpacing(10)
        sidebar_layout.addItem(QSpacerItem(10,10))

        #add sidebar elements
        self.liveMode = QLabel("Mode: N/A")
        self.liveMode.setStyleSheet("color: black; font-weight: bold;")
        self.liveState = QLabel("State: N/A")
        self.liveState.setStyleSheet("color: black; font-weight: bold;")
        self.liveMissionTime = QLabel("Mission Time: N/A")
        self.liveMissionTime.setStyleSheet("color: black; font-weight: bold;")
        self.liveGPSTime = QLabel("GPS Time: N/A")
        self.liveGPSTime.setStyleSheet("color: black; font-weight: bold;")
        self.livePacketCount = QLabel("Packet Count: N/A")
        self.livePacketCount.setStyleSheet("color: black; font-weight: bold;")
        self.liveReceivedPackets = QLabel("Received Packets: N/A")
        self.liveReceivedPackets.setStyleSheet("color: black; font-weight: bold;")
        self.liveGPSAltitude = QLabel("GPS Altitude: N/A")
        self.liveGPSAltitude.setStyleSheet("color: black; font-weight: bold;")
        self.liveCMDEcho = QLabel("CMD Echo: N/A")
        self.liveCMDEcho.setStyleSheet("color: black; font-weight: bold;")
        sidebar_layout.addWidget(self.liveMode)
        sidebar_layout.addWidget(self.liveState)
        sidebar_layout.addWidget(self.liveMissionTime)
        sidebar_layout.addWidget(self.liveGPSTime)
        sidebar_layout.addWidget(self.livePacketCount)
        sidebar_layout.addWidget(self.liveReceivedPackets)
        sidebar_layout.addWidget(self.liveGPSAltitude)
        sidebar_layout.addWidget(self.liveCMDEcho)

        #primary command reset
        self.sim = False
        self.release = False
        self.cam = False

        #add command buttons
        self.reset_graphs_button = QPushButton("Reset Graphs")
        self.reset_graphs_button.setStyleSheet("color: black; border: 1px solid black; font-weight: bold;")
        self.reset_graphs_button.clicked.connect(self.reset_graphs)
        self.set_UTC_time_button = QPushButton("Set UTC Time")
        self.set_UTC_time_button.setStyleSheet("color: black; border: 1px solid black; font-weight: bold;")
        self.set_UTC_time_button.clicked.connect(self.set_utc_time)
        self.set_GPS_time_button = QPushButton("Set GPS Time")
        self.set_GPS_time_button.setStyleSheet("color: black; border: 1px solid black; font-weight: bold;")
        self.set_GPS_time_button.clicked.connect(self.set_gps_time)
        self.SIM_toggle_button = QPushButton("SIM Enable")
        self.SIM_toggle_button.setStyleSheet("color: black; border: 1px solid black; font-weight: bold;")
        self.SIM_toggle_button.clicked.connect(self.toggle_sim)
        self.SIM_activate_button = QPushButton("SIM Activate")
        self.SIM_activate_button.setStyleSheet("color: black; border: 1px solid black; font-weight: bold;")
        self.SIM_activate_button.clicked.connect(self.sim_activate)
        self.CAL_button = QPushButton("CAL")
        self.CAL_button.setStyleSheet("color: black; border: 1px solid black; font-weight: bold;")
        self.CAL_button.clicked.connect(self.cal)
        self.RELEASE_toggle = QPushButton("CANISTER RELEASE ON")
        self.RELEASE_toggle.setStyleSheet("color: black; border: 1px solid black; font-weight: bold;")
        self.RELEASE_toggle.clicked.connect(self.toggle_release)
        self.CAM_toggle = QPushButton("CAM ON")
        self.CAM_toggle.setStyleSheet("color: black; border: 1px solid black; font-weight: bold;")
        self.CAM_toggle.clicked.connect(self.toggle_cam)
        self.start_stop_button = QPushButton("CXON")
        self.start_stop_button.setStyleSheet("color: black; border: 1px solid black; font-weight: bold;")
        self.start_stop_button.clicked.connect(self.toggle_data_transmission)
        self.calCamStabilization = QPushButton("Cal Cam Stable")
        self.calCamStabilization.setStyleSheet("color: black; border: 1px solid black; font-weight: bold;")
        self.calCamStabilization.clicked.connect(self.cal_camera_stabilization)

        #add command buttons to sidebar
        sidebar_layout.addWidget(self.reset_graphs_button)
        sidebar_layout.addWidget(self.set_UTC_time_button)
        sidebar_layout.addWidget(self.set_GPS_time_button)
        sidebar_layout.addWidget(self.SIM_toggle_button)
        sidebar_layout.addWidget(self.SIM_activate_button)
        sidebar_layout.addWidget(self.CAL_button)
        sidebar_layout.addWidget(self.RELEASE_toggle)
        sidebar_layout.addWidget(self.CAM_toggle)
        sidebar_layout.addWidget(self.start_stop_button)
        sidebar_layout.addWidget(self.calCamStabilization)

        #add footer
        footer_layout = QVBoxLayout()

        #footer data
        self.accelerationRPY = QLabel("Acceleration R,P,Y: N/A")
        self.accelerationRPY.setStyleSheet("color: black; font-weight: bold;")
        self.magnetometerRPY = QLabel("Magnetometer R,P,Y: N/A")
        self.magnetometerRPY.setStyleSheet("color: black; font-weight: bold;")
        self.telemetry = QLabel("Telemetry: N/A")
        self.telemetry.setStyleSheet("color: black; font-weight: bold;")
        footer_layout.addWidget(self.telemetry)

        #graphs/GPS layout
        graphs_layout = QVBoxLayout()
        graphs_grid = QGridLayout()
        self.altitudeGraph = AltitudeGraph()
        self.autoGyroRotationGraph = AutoGyroRotationGraph()
        self.temperatureGraph = TemperatureGraph()
        self.rotationGraph = RotationGraph()
        self.voltageGraph = VoltageGraph()
        self.GPS = GPSMap()
        graphs_grid.addWidget(self.altitudeGraph.win, 0, 0)
        graphs_grid.addWidget(self.autoGyroRotationGraph.win, 0, 1)
        graphs_grid.addWidget(self.temperatureGraph.win, 1, 0)
        graphs_grid.addWidget(self.rotationGraph.win, 1, 1)
        graphs_grid.addWidget(self.voltageGraph.win, 2, 0)
        graphs_grid.addWidget(self.GPS.win, 2, 1)
        graphs_grid.addWidget(self.accelerationRPY, 3, 0)
        graphs_grid.addWidget(self.magnetometerRPY, 4, 0)

        graphs_layout.addLayout(graphs_grid)

        #add all elements to main layout
        content_layout.addLayout(sidebar_layout)
        content_layout.addLayout(graphs_layout)
        main_layout.addLayout(content_layout)
        main_layout.addLayout(footer_layout)

        #communcation setup
        self.comm = Communication(serial_port='COM5') #update serial port
        self.reader_thread = None
        self.reading_data = False

        self.signal_emitter = SignalEmitter()
        self.signal_emitter.update_signal.connect(self.update_graphs)

        #start timer for updating live data
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
        self.reader_thread = threading.Thread(target=self.comm.read, args=(self.signal_emitter,))
        self.reader_thread.start()

    #stop data transmission
    def stop_data_transmission(self):
        self.reading_data = False
        self.start_stop_button.setText("CXON")
        self.comm.send_command("CMD,3195,CX,OFF")
        if self.reader_thread and self.reader_thread.is_alive():
            self.comm.stop_reading()
            self.reader_thread.join()
            self.reader_thread = None

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
        self.SIM_toggle_button.setText("SIM Disable")
        self.comm.send_command("CMD,3195,SIM,ENABLE")
    def sim_activate(self):
        self.comm.send_command("CMD,3195,SIM,ACTIVATE")
        self.comm.simulation_mode()
    def sim_disable(self):
        self.comm.simulation = False
        self.SIM_toggle_button.setText("SIM Enable")
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
        self.RELEASE_toggle.setText("RELEASE OFF")
        self.comm.send_command("CMD,3195,MEC,RELEASE,ON")
    def release_off(self):
        self.RELEASE_toggle.setText("RELEASE ON")
        self.comm.send_command("CMD,3195,MEC,RELEASE,OFF")
    def toggle_cam(self):
        if self.cam:
            self.cam_off()
            self.cam = False
        elif not self.cam:
            self.cam_on()
            self.cam = True
    def cam_on(self):
        self.CAM_toggle.setText("CAM OFF")
        self.comm.send_command("CMD,3195,MEC,CAM,ON")
    def cam_off(self):
        self.CAM_toggle.setText("CAM ON")
        self.comm.send_command("CMD,3195,MEC,CAM,OFF")
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
        self.liveGPSAltitude.setText(f"GPS Altitude: {self.comm.get_GPS_ALTITUDE() or 'N/A'}")
        self.liveCMDEcho.setText(f"CMD Echo: {self.comm.get_CMD_ECHO() or 'N/A'}")
        self.accelerationRPY.setText(f"Acceleration R,P,Y: {f"{self.comm.get_ACCEL_R()}, {self.comm.get_ACCEL_P()}, {self.comm.get_ACCEL_Y()}" or 'N/A'}")
        self.magnetometerRPY.setText(f"Magnetometer R,P,Y: {f"{self.comm.get_MAG_R()}, {self.comm.get_MAG_P()}, {self.comm.get_MAG_Y()}" or 'N/A'}")
        self.telemetry.setText(f"Telemetry: {self.comm.lastPacket or 'N/A'}")

        #update GPS data
        latitude = self.comm.get_GPS_LATITUDE()
        longitude = self.comm.get_GPS_LONGITUDE()
        if latitude is not None and longitude is not None:
            self.GPS.location_updated.emit(latitude, longitude)

    #update graphs
    def update_graphs(self):
        current_time = time()

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

    #reset graphs
    def reset_graphs(self):
        self.altitudeGraph.reset_graph()
        self.autoGyroRotationGraph.reset_graph()
        self.temperatureGraph.reset_graph()
        self.rotationGraph.reset_graph()
        self.voltageGraph.reset_graph()

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
