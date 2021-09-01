from PyQt5.QtWidgets import QLineEdit, QLabel, QVBoxLayout, QWidget, QApplication
from PyQt5.QtCore import Qt, pyqtSignal, QPoint, QPointF, QRectF
from PyQt5.QtGui import QRadialGradient, QLinearGradient, QColor, QBrush, QPainter, QPen, QPalette
import sys

class DisplayLine(QLineEdit):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.radius = 2000

        self.setDisabled(True)
        self.ready = False
        self.drawBackground()

        palette = QPalette()
        palette.setColor(QPalette.Text, Qt.darkGray)
        self.setPalette(palette)
        self.setAlignment(Qt.AlignCenter)
        # self.setStyleSheet('background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0  # E1E1E1, stop: 0.4 #DDDDDD, stop: 0.5  # D8D8D8, stop: 1.0 #D3D3D3);')
        # self.setStyleSheet('*{background: qradialgradient(cx:0.5, cy:0.5, fx:0.5, fy:0.5, radius: 2, stop:0 white, stop:1 green);}')
        # self.setStyleSheet(" *{ background: QLinearGradient( x1: 0, y1: 0, x2: 1, y2: 0, stop: 0 white, stop: 1 green );}")
        # self.setStyleSheet("* {background: qlineargradient( x1:0 y1:0, x2:1 y2:0, stop:0 cyan, stop:1 blue);}")

    # def paintEvent(self, event):
    #
    #     painter = QPainter(self)
    #
    #     painter.setRenderHint(QPainter.Antialiasing)
    #     painter.setBackgroundMode(Qt.TransparentMode)
    #     rect = QRectF(self.rect())
    #
    #     gradient = QRadialGradient(rect.center(), 2000)
    #
    #     gradient.setColorAt(0, QColor(255, 255, 255))
    #     if self.ready:
    #         gradient.setColorAt(1, QColor(0, 255, 0))
    #     else:
    #         gradient.setColorAt(1, QColor(255, 0, 0))
    #
    #     painter.setBrush(QBrush(gradient))
    #     painter.drawRect(rect)
    #
    #     super().paintEvent(event)
    def drawBackground(self):
        if self.ready:
            self.setStyleSheet('* {border: 1px solid lightgrey; background-color: qradialgradient(cx:0.5, cy:0.5, fx:0.5, fy:0.5, radius: 5, stop:0 white, stop:1 green);}')
        else:
            self.setStyleSheet('* {border: 1px solid lightgrey; background-color: qradialgradient(cx:0.5, cy:0.5, fx:0.5, fy:0.5, radius: 5, stop:0 white, stop:1 red);}')

    def setReady(self, flag):
        self.ready = flag
        self.drawBackground()

if __name__ == '__main__':

    app = QApplication(sys.argv)
    gui = DisplayLine()
    gui.setText('daniele')
    gui.show()
    sys.exit(app.exec_())
