import sys
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore
import time

class TemperatureGraph:
    def __init__(self):
        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')
        self.app = QtWidgets.QApplication(sys.argv)
        self.win = pg.GraphicsLayoutWidget(show=True, title="Temperature")
        self.win.setStyleSheet("border: 1px solid black;")
        self.plot = self.win.addPlot(title="Temperature")
        self.curve = self.plot.plot(pen='r')
        self.data = []
        self.timestamps = []
        self.start_time = None

        self.plot.setLabel('left', 'Temperature (°C)')
        self.plot.setLabel('bottom', 'Time', 's')
        self.plot.setRange(yRange=[0, 50])

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(1000)  # Update every 1000 ms

    def start_tracking(self):
        if self.start_time is None:
            self.start_time = time.time()

    def update_graph(self, temperature, timestamp):
        self.start_tracking()
        elapsed_time = timestamp - self.start_time

        self.data.append(temperature)
        self.timestamps.append(elapsed_time)
        #self.data = self.data[-20:]
        #self.timestamps = self.timestamps[-20:]

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