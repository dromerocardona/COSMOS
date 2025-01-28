import sys
import threading
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QLabel, QPushButton, \
    QSpacerItem, QSizePolicy, QGridLayout, QProgressBar
from PyQt5.QtCore import Qt, pyqtSignal, QObject, QTimer
from PyQt5.QtGui import QFont, QPixmap, QIcon
from communication import Communication
from pressureGraph import PressureGraph
from temperatureGraph import TemperatureGraph
from altitudeGraph import AltitudeGraph
from rotationGraph import RotationGraph
from voltageGraph import VoltageGraph
from GPS import GPSMap
import time

class LoadingScreen(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Loading...")
        self.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)
        self.setGeometry(100, 100, 1200, 800)
        self.setWindowIcon(QIcon('COSMOS_logo.png'))

        layout = QVBoxLayout()

        loading_pixmap = QPixmap('COSMOS_logo.png')  # Update with the correct path to your image
        self.image_width,image_height = 100,100
        loading_pixmap = loading_pixmap.scaled(self.image_width, image_height)
        self.image_label = QLabel()
        self.image_label.setPixmap(loading_pixmap)
        self.image_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.image_label)

        self.label = QLabel("Loading...")
        self.label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.label)

        self.progress_bar = QProgressBar()
        self.progress_bar.setValue(0)
        layout.addWidget(self.progress_bar)

        self.setLayout(layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_progress)
        self.timer.start(100)

    def update_progress(self):
        current_value = self.progress_bar.value()
        if current_value < 100:
            self.progress_bar.setValue(current_value + 5)
        else:
            self.timer.stop()
            self.close()

class SignalEmitter(QObject):
    update_signal = pyqtSignal()

    def emit_signal(self):
        self.update_signal.emit()

class GroundStation(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("COSMOS GS")
        self.setGeometry(100, 100, 1200, 800)
        self.setWindowIcon(QIcon('COSMOS_logo.png'))

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

        content_layout = QHBoxLayout()

        sidebar_layout = QVBoxLayout()
        sidebar_layout.setAlignment(Qt.AlignTop)
        sidebar_layout.setSpacing(10)
        sidebar_layout.addItem(QSpacerItem(10,10))

        #add sidebar elements
        self.liveMode = QLabel("Mode: N/A")
        self.liveState = QLabel("State: N/A")
        self.liveMissionTime = QLabel("Mission Time: N/A")
        self.liveGPSTime = QLabel("GPS Time: N/A")
        self.livePacketCount = QLabel("Packet Count: N/A")
        self.liveReceivedPackets = QLabel("Received Packets: N/A")
        self.liveGPSAltitude = QLabel("GPS Altitude: N/A")
        self.liveAutogyroRotationRate = QLabel("Autogyro Rotation Rate: N/A")
        self.liveCMDEcho = QLabel("CMD Echo: N/A")
        sidebar_layout.addWidget(self.liveMode)
        sidebar_layout.addWidget(self.liveState)
        sidebar_layout.addWidget(self.liveMissionTime)
        sidebar_layout.addWidget(self.liveGPSTime)
        sidebar_layout.addWidget(self.livePacketCount)
        sidebar_layout.addWidget(self.liveReceivedPackets)
        sidebar_layout.addWidget(self.liveGPSAltitude)
        sidebar_layout.addWidget(self.liveAutogyroRotationRate)
        sidebar_layout.addWidget(self.liveCMDEcho)

        self.start_stop_button = QPushButton("CXON")
        self.start_stop_button.clicked.connect(self.toggle_data_transmission)
        sidebar_layout.addWidget(self.start_stop_button)

        self.accelerationRPY = QLabel("Acceleration R,P,Y: N/A")
        self.magnetometerRPY = QLabel("Magnetometer R,P,Y: N/A")
        self.telemetry = QLabel("Telemetry: N/A")

        graphs_layout = QVBoxLayout()
        graphs_grid = QGridLayout()
        self.altitudeGraph = AltitudeGraph()
        self.pressureGraph = PressureGraph()
        self.temperatureGraph = TemperatureGraph()
        self.rotationGraph = RotationGraph()
        self.voltageGraph = VoltageGraph()
        self.GPS = GPSMap()
        graphs_grid.addWidget(self.altitudeGraph.win, 0, 0)
        graphs_grid.addWidget(self.pressureGraph.win, 0, 1)
        graphs_grid.addWidget(self.temperatureGraph.win, 1, 0)
        graphs_grid.addWidget(self.rotationGraph.win, 1, 1)
        graphs_grid.addWidget(self.voltageGraph.win, 2, 0)
        graphs_grid.addWidget(self.GPS.win, 2, 1)

        graphs_layout.addLayout(graphs_grid)

        content_layout.addLayout(sidebar_layout)
        content_layout.addLayout(graphs_layout)
        main_layout.addLayout(content_layout)

        self.comm = Communication(serial_port='COM8') #update serial port

        self.reader_thread = None
        self.reading_data = False

        self.signal_emitter = SignalEmitter()
        self.signal_emitter.update_signal.connect(self.update_graphs)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_live_data)
        self.timer.start(1000)

    def toggle_data_transmission(self):
        if self.reading_data:
            self.stop_data_transmission()
        else:
            self.start_data_transmission()

    def start_data_transmission(self):
        self.reading_data = True
        self.start_stop_button.setText("CXOFF")
        self.reader_thread = threading.Thread(target=self.comm.read, args=(self.signal_emitter,))
        self.reader_thread.start()

    def stop_data_transmission(self):
        self.reading_data = False
        self.start_stop_button.setText("CXON")
        if self.reader_thread and self.reader_thread.is_alive():
            self.comm.stop_reading()
            self.reader_thread.join()
            self.reader_thread = None

    def update_live_data(self):
        self.liveMode.setText(f"Mode: {self.comm.get_MODE() or 'N/A'}")
        self.liveState.setText(f"State: {self.comm.get_STATE() or 'N/A'}")
        self.liveMissionTime.setText(f"Mission Time: {self.comm.get_MISSION_TIME() or 'N/A'}")
        self.liveGPSTime.setText(f"GPS Time: {self.comm.get_GPS_TIME() or 'N/A'}")
        self.livePacketCount.setText(f"Packet Count: {self.comm.get_PACKET_COUNT() or 'N/A'}")
        self.liveReceivedPackets.setText(f"Received Packets: {self.comm.receivedPacketCount or 'N/A'}")
        self.liveGPSAltitude.setText(f"GPS Altitude: {self.comm.get_GPS_ALTITUDE() or 'N/A'}")
        self.liveAutogyroRotationRate.setText(f"Autogyro Rotation Rate: {self.comm.get_AUTO_GYRO_ROTATION_RATE() or 'N/A'}")
        self.liveCMDEcho.setText(f"CMD Echo: {self.comm.get_CMD_ECHO() or 'N/A'}")
        self.accelerationRPY.setText(f"Acceleration R,P,Y: {f"{self.comm.get_ACCEL_R()}, {self.comm.get_ACCEL_P()},{self.comm.get_ACCEL_Y()}" or 'N/A'}")
        self.magnetometerRPY.setText(f"Magnetometer R,P,Y: {f"{self.comm.get_MAG_R()}, {self.comm.get_MAG_P()}, {self.comm.get_MAG_Y()}" or 'N/A'}")
        self.telemetry.setText(f"Telemetry: {self.comm.lastPacket or 'N/A'}")

    def update_graphs(self):
        current_time = time()

        altitude = self.comm.get_ALTITUDE()
        if altitude is not None:
            self.altitudeGraph.update_graph(altitude, current_time)
        pressure = self.comm.get_PRESSURE()
        if pressure is not None:
            self.pressureGraph.update_graph(pressure, current_time)
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

    def reset_graphs(self):
        self.altitudeGraph.reset_graph()
        self.pressureGraph.reset_graph()
        self.temperatureGraph.reset_graph()
        self.rotationGraph.reset_graph()
        self.voltageGraph.reset_graph()

    def closeEvent(self, event):
        self.stop_data_transmission()
        event.accept()

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
