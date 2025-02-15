import sys
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore
import time

class RotationGraph:
    def __init__(self):
        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')
        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
        self.win = pg.GraphicsLayoutWidget(show=True, title="Rotation")
        self.win.setStyleSheet("border: 1px solid black;")
        self.plot = self.win.addPlot(title="Rotation")

        self.plot.addLegend()

        self.curve_r = self.plot.plot(pen='r', name="GYRO_R")
        self.curve_p = self.plot.plot(pen='g', name="GYRO_P")
        self.curve_y = self.plot.plot(pen='b', name="GYRO_Y")

        self.data_r = []
        self.data_p = []
        self.data_y = []
        self.timestamps = []
        self.start_time = None

        self.plot.setLabel('left', 'Rotation', '°')
        self.plot.setLabel('bottom', 'Time', 's')
        self.plot.setRange(yRange=[-360, 360])
        self.plot.addLegend()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(1000)

    def start_tracking(self):
        if self.start_time is None:
            self.start_time = time.time()

    def update_graph(self, gyro_r, gyro_p, gyro_y, timestamp):
        self.start_tracking()
        elapsed_time = timestamp - self.start_time

        self.data_r.append(gyro_r)
        self.data_p.append(gyro_p)
        self.data_y.append(gyro_y)
        self.timestamps.append(elapsed_time)

        #self.data_r = self.data_r[-20:]
        #self.data_p = self.data_p[-20:]
        #self.data_y = self.data_y[-20:]
        #self.timestamps = self.timestamps[-20:]

    def update_gui(self):
        self.curve_r.setData(self.timestamps, self.data_r)
        self.curve_p.setData(self.timestamps, self.data_p)
        self.curve_y.setData(self.timestamps, self.data_y)
        if len(self.timestamps) > 1:
            self.plot.setXRange(self.timestamps[0], self.timestamps[-1])

    def start(self):
        self.win.show()
        sys.exit(self.app.exec_())

    def reset_graph(self):
        self.data_r.clear()
        self.data_p.clear()
        self.data_y.clear()
        self.timestamps.clear()
        self.start_time = None