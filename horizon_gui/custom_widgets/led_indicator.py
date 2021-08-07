from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import sys


class LedIndicator(QWidget):
    scaledSize = 1000.0

    def __init__(self, parent=None):
        QWidget.__init__(self, parent)

        self.setMinimumSize(5, 5)
        # self.setCheckable(True)
        self.is_ready = False

        # Grey
        disabled_color_1 = QColor(107, 107, 107)
        disabled_color_2 = QColor(93, 93, 93)
        self.color_disabled = [disabled_color_1, disabled_color_2]

        # # Green
        # self.on_color_1 = QColor(0, 255, 0)
        # self.on_color_2 = QColor(0, 192, 0)
        ready_color_1 = QColor(0, 28, 0)
        ready_color_2 = QColor(0, 128, 0)
        self.color_ready = [ready_color_1, ready_color_2]

        # Red
        not_ready_color_1 = QColor(255, 0, 0)
        not_ready_color_2 = QColor(192, 0, 0)
        # self.off_color_1 = QColor(28, 0, 0)
        # self.off_color_2 = QColor(128, 0, 0)
        self.color_not_ready = [not_ready_color_1, not_ready_color_2]

    def resizeEvent(self, QResizeEvent):
        self.update()

    def paintEvent(self, QPaintEvent):
        realSize = min(self.width(), self.height())

        painter = QPainter(self)
        pen = QPen(Qt.black)
        pen.setWidth(1)

        painter.setRenderHint(QPainter.Antialiasing)
        painter.translate(self.width() / 2, self.height() / 2)
        painter.scale(realSize / self.scaledSize, realSize / self.scaledSize)

        gradient = QRadialGradient(QPointF(-500, -500), 1500, QPointF(-500, -500))
        gradient.setColorAt(0, QColor(224, 224, 224))
        gradient.setColorAt(1, QColor(28, 28, 28))
        painter.setPen(pen)
        painter.setBrush(QBrush(gradient))
        painter.drawEllipse(QPointF(0, 0), 500, 500)

        gradient = QRadialGradient(QPointF(500, 500), 1500, QPointF(500, 500))
        gradient.setColorAt(0, QColor(224, 224, 224))
        gradient.setColorAt(1, QColor(28, 28, 28))
        painter.setPen(pen)
        painter.setBrush(QBrush(gradient))
        painter.drawEllipse(QPointF(0, 0), 490, 490)

        painter.setPen(pen)
        if not self.isEnabled():
            gradient = QRadialGradient(QPointF(-500, -500), 1500, QPointF(-500, -500))
            gradient.setColorAt(0, self.color_disabled[0])
            gradient.setColorAt(1, self.color_disabled[1])
        else:

            if self.is_ready:
                gradient = QRadialGradient(QPointF(-500, -500), 1500, QPointF(-500, -500))
                gradient.setColorAt(0, self.color_ready[0])
                gradient.setColorAt(1, self.color_ready[1])
            else:
                gradient = QRadialGradient(QPointF(500, 500), 1500, QPointF(500, 500))
                gradient.setColorAt(0, self.color_not_ready[0])
                gradient.setColorAt(1, self.color_not_ready[1])

        painter.setBrush(gradient)
        painter.drawEllipse(QPointF(0, 0), 480, 480)

    def setReady(self, flag):
        self.is_ready = flag
        self.repaint()

    # @pyqtProperty(QColor)
    # def onColor1(self):
    #     return self.on_color_1
    #
    # @onColor1.setter
    # def onColor1(self, color):
    #     self.on_color_1 = color
    #
    # @pyqtProperty(QColor)
    # def onColor2(self):
    #     return self.on_color_2
    #
    # @onColor2.setter
    # def onColor2(self, color):
    #     self.on_color_2 = color
    #
    # @pyqtProperty(QColor)
    # def offColor1(self):
    #     return self.off_color_1
    #
    # @offColor1.setter
    # def offColor1(self, color):
    #     self.off_color_1 = color
    #
    # @pyqtProperty(QColor)
    # def offColor2(self):
    #     return self.off_color_2
    #
    # @offColor2.setter
    # def offColor2(self, color):
    #     self.off_color_2 = color



class ExampleWindow(QWidget):
    def __init__(self):
        super(ExampleWindow, self).__init__()

        self.setWindowTitle("Checkbox")

        main_layout = QGridLayout(self)

        self.led = LedIndicator()
        self.led.setEnabled(False)



        main_layout.addWidget(self.led)



if __name__ == '__main__':

    app = QApplication(sys.argv)
    gui = ExampleWindow()

    gui.show()
    sys.exit(app.exec_())
