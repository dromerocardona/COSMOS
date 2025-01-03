import sys
#from offline_folium import offline
import folium
import os
from PyQt5 import QtWidgets, QtCore, QtWebEngineWidgets
import time

class GPSMap:
    def __init__(self):
        # Initializing PyQt Application
        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)

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
        self.map_file = "live_gps_map.html"

        # Timer for periodic updates
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(1000)

    def start_tracking(self):
        """Initialize tracking if needed."""
        pass  # Tracking state management can be added here if required

    def update_map(self, latitude, longitude):
        """Update the map HTML file with new GPS coordinates."""
        # Create a new folium map centered at the given latitude and longitude
        folium_map = folium.Map(location=[latitude, longitude], zoom_start=15)
        folium.Marker([latitude, longitude], tooltip="Current Position").add_to(folium_map)

        # Save map to file
        folium_map.save(self.map_file)

    def update_gui(self):
        """Fetch live telemetry and update the map view."""
        # Simulate fetching telemetry data (Replace with actual live telemetry fetching logic)
        telemetry = self.get_live_telemetry()
        if telemetry:
            self.latitude = telemetry.get("latitude", self.latitude)
            self.longitude = telemetry.get("longitude", self.longitude)

            # Update the folium map file
            self.update_map(self.latitude, self.longitude)

            # Load the updated map in the web view
            self.browser.setUrl(QtCore.QUrl.fromLocalFile(os.path.abspath(self.map_file)))

    def start(self):
        """Start the application."""
        self.win.show()
        sys.exit(self.app.exec_())

    def reset_map(self):
        """Reset the map to initial state."""
        self.latitude = 0.0
        self.longitude = 0.0
        self.update_map(self.latitude, self.longitude)

    def get_live_telemetry(self):
        """Simulate fetching live telemetry data."""
        # Replace this dummy implementation with live telemetry fetch
        # Example of the data format:
        # return {"latitude": 40.7128, "longitude": -74.0060}  # Example: New York coordinates
        return {"latitude": self.latitude + 0.001, "longitude": self.longitude + 0.001}
