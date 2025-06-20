import sys
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore
import time

class AltitudeGraph:
    def __init__(self):
        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')
        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
        self.win = pg.GraphicsLayoutWidget(show=True, title="Altitude")
        self.win.setStyleSheet("border: 1px solid black;")
        self.plot = self.win.addPlot(title="<b>Altitude</b>")
        self.curve = self.plot.plot(pen=pg.mkPen(color='#1e8d12', width=3), symbol='o', symbolSize=8, symbolBrush='#1e8d12')
        self.data = []
        self.timestamps = []
        self.start_time = None

        self.plot.setLabel('left', 'Altitude', 'm')
        self.plot.setLabel('bottom', 'Time (s)')
        self.plot.setRange(yRange=[0, 600])

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(1000)

    def start_tracking(self):
        if self.start_time is None:
            self.start_time = time.time()

    def update_graph(self, altitude, timestamp):
        self.start_tracking()
        elapsed_time = timestamp - self.start_time

        self.data.append(altitude)
        self.timestamps.append(elapsed_time)
        self.data = self.data[-20:]
        self.timestamps = self.timestamps[-20:]

    def update_gui(self):
        self.curve.setData(self.timestamps, self.data)
        if len(self.timestamps) > 1:
            self.plot.setXRange(self.timestamps[0], self.timestamps[-1])

    def start(self):
        self.win.show()
        sys.exit(self.app.exec_())

    def reset_graph(self):
        self.data.clear()
        self.timestamps.clear()
        self.start_time = None

    def toggle_dark_mode(self, dark_mode: bool):
        if dark_mode:
            self.win.setBackground('#4f5052')  # Set black background
            pg.setConfigOption('foreground', 'w')  # White text
            title_color = 'w'
        else:
            self.win.setBackground('w')  # Set white background
            pg.setConfigOption('foreground', 'k')  # Black text
            title_color = 'k'

        # Update axis colors
        self.plot.getAxis('left').setPen(title_color)
        self.plot.getAxis('bottom').setPen(title_color)

        # Update title color
        self.plot.setTitle("<b>Altitude</b>", color=title_color, size="12pt")

        # Update tick label colors
        self.plot.getAxis('left').setTextPen(title_color)
        self.plot.getAxis('bottom').setTextPen(title_color)