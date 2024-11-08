import sys
import threading
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QLabel, QPushButton, \
    QSpacerItem, QSizePolicy, QGridLayout
from PyQt5.QtCore import Qt, pyqtSignal, QObject, QTimer
from PyQt5.QtGui import QFont, QPixmap, QIcon
from communication import Communication

class SignalEmitter(QObject):
    update_signal = pyqtSignal()

    def emit_signal(self):
        self.update_signal.emit()

class GroundStation(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Cosmos GS")
        self.setGeometry(100, 100, 1200, 800)
        self.setWindowIcon(QIcon('icon.png'))

        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)

        main_layout = QVBoxLayout(self.central_widget)

        header_layout = QHBoxLayout()
        #add header elements
        header_widget = QWidget()
        header_widget.setLayout(header_layout)
        main_layout.addWidget(header_widget)

        content_layout = QHBoxLayout()

        sidebar_layout = QVBoxLayout()
        sidebar_layout.setAlignment(Qt.AlignTop)
        sidebar_layout.setSpacing(10)
        sidebar_layout.addItem(QSpacerItem(10,10))
        #add sidebar elements

        graphs_layout = QVBoxLayout()
        graphs_grid = QGridLayout()
        #add graphs

        graphs_layout.addLayout(graphs_grid)

        content_layout.addLayout(sidebar_layout)
        content_layout.addLayout(graphs_layout)
        main_layout.addLayout(content_layout)

        self.comm = Communication(serial_port='COM8')

        self.timer = QTimer(self)
        #self.timer.timeout.connect(self.update_live_data)
        self.timer.start(1000)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = GroundStation()
    window.show()
    sys.exit(app.exec_())