import sys
from offline_folium import offline
import folium
import os
from PyQt5 import QtWidgets, QtCore, QtWebEngineWidgets
from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import pyqtSignal, pyqtSlot, QObject
import shutil

class GPSMap(QWidget, QObject):
    location_updated = QtCore.pyqtSignal(float, float)

    def __init__(self):
        super().__init__()
        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
        self.setMinimumSize(600, 600)

        # Create a QWidget as the window
        self.win = QtWidgets.QWidget()
        self.win.setWindowTitle("GPS Map")
        self.layout = QtWidgets.QVBoxLayout()
        self.win.setLayout(self.layout)

        # Create a QWebEngineView to display the map
        self.browser = QtWebEngineWidgets.QWebEngineView()
        self.layout.addWidget(self.browser)

        # Placeholder for telemetry data
        self.latitude = 0.0
        self.longitude = 0.0
        shutil.copyfile("template_map.html", "live_gps_map.html")
        self.map_file = "live_gps_map.html"
        self.location_updated.connect(self.update_map)

        # Timer for periodic updates
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(1000)

        # **Initial Map Display**
        self.create_initial_map()

    @pyqtSlot(float, float)
    def update_map(self, latitude, longitude):
        """Update the map HTML file with new GPS coordinates."""
        self.latitude = latitude
        self.longitude = longitude

        folium_map = folium.Map(location=[latitude, longitude], zoom_start=15)
        folium.Marker([latitude, longitude], tooltip="Current Position").add_to(folium_map)

        # Save map to file
        folium_map.save(self.map_file)

    def update_gui(self):
        """Fetch live telemetry and update the map view."""
        if self.latitude and self.longitude:
            # Update the folium map file
            self.update_map(self.latitude, self.longitude)

            # Load the updated map in the web view
            self.browser.setUrl(QtCore.QUrl.fromLocalFile(os.path.abspath(self.map_file)))

    def create_initial_map(self):
        """Create an initial map with a default location."""
        # Replace with desired default location
        initial_latitude = 34.722719
        initial_longitude = -86.638421

        folium_map = folium.Map(location=[initial_latitude, initial_longitude], zoom_start=10)
        folium_map.save(self.map_file)
        self.browser.setUrl(QtCore.QUrl.fromLocalFile(os.path.abspath(self.map_file)))

    def start(self):
        """Start the application."""
        self.win.show()
        sys.exit(self.app.exec_())