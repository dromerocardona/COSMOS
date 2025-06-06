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
        self.plot = self.win.addPlot(title="<b>Rotation</b>")

        self.plot.addLegend()

        self.curve_r = self.plot.plot(pen=pg.mkPen(color='r', width=3), name="GYRO_R", symbol='o', symbolSize=8, symbolBrush='r')
        self.curve_p = self.plot.plot(pen=pg.mkPen(color='#1e8d12', width=3), name="GYRO_P", symbol='o', symbolSize=8, symbolBrush='#1e8d12')
        self.curve_y = self.plot.plot(pen=pg.mkPen(color='b', width=3), name="GYRO_Y", symbol='o', symbolSize=8, symbolBrush='b')

        self.data_r = []
        self.data_p = []
        self.data_y = []
        self.timestamps = []
        self.start_time = None

        self.plot.setLabel('left', 'Rotation (°/s)')
        self.plot.setLabel('bottom', 'Time (s)')
        self.plot.setRange(yRange=[-1000, 1000])
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

        self.data_r = self.data_r[-20:]
        self.data_p = self.data_p[-20:]
        self.data_y = self.data_y[-20:]
        self.timestamps = self.timestamps[-20:]

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
        self.plot.setTitle("<b>Rotation</b>", color=title_color, size="12pt")

        # Update tick label colors
        self.plot.getAxis('left').setTextPen(title_color)
        self.plot.getAxis('bottom').setTextPen(title_color)