#!/usr/bin/env python
"""
Qt Widget Range slider widget.
Hazen 06/13
"""

from PyQt5 import QtCore, QtGui, QtWidgets


class Slice(QtWidgets.QWidget):
    def __init__(self, min, max):
        self.min = min
        self.max = max

        self.single_step = 0.0
        self.moving = "none"
        self.old_scale_min = 0.0
        self.old_scale_max = 0.0
        self.scale = 0

    def emitRange(self):
        if (self.old_scale_min != self.scale_min) or (self.old_scale_max != self.scale_max):
            self.rangeChanged.emit(self.scale_min, self.scale_max)
            self.old_scale_min = self.scale_min
            self.old_scale_max = self.scale_max
            if 0:
                print("Range change:", self.scale_min, self.scale_max)

    def setValues(self, values):
        self.scale_min = values[0]
        self.scale_max = values[1]
        self.emitRange()
        self.updateDisplayValues()
        self.update()

    def getValues(self):
        return [self.scale_min, self.scale_max]


    def enterEvent(self, event):
        print("Mouse Entered")
        return super(Slice, self).enterEvent(event)

    def leaveEvent(self, event):
        print("Mouse Left")
        return super(Slice, self).enterEvent(event)

class QRangeSlider(QtWidgets.QWidget):
    """
    Range Slider super class.
    """

    doubleClick = QtCore.pyqtSignal()
    rangeChanged = QtCore.pyqtSignal(float, float)

    def __init__(self, slider_range, values, parent=None):
        QtWidgets.QWidget.__init__(self, parent)
        self.bar_width = 10
        self.emit_while_moving = 0
        self.slices = []
        self.setMouseTracking(False)

        if slider_range:
            self.setRange(slider_range)
        else:
            self.setRange([0.0, 1.0, 0.01])

        if values:
            # add a slice of the given range for each list
            # todo add more slices in constructor
            self.slices.append(Slice(values[0], values[1]))
        else:
            self.slices.append(Slice(0.3, 0.6))

    #
    # def resizeEvent(self, event):
        # self.updateDisplayValues()
    #
    # def setRange(self, slider_range):
    #     self.start = slider_range[0]
    #     self.scale = slider_range[1] - slider_range[0]
    #     self.single_step = slider_range[2]
    #
    # def setEmitWhileMoving(self, flag):
    #     if flag:
    #         self.emit_while_moving = 1
    #     else:
    #         self.emit_while_moving = 0

    def setRange(self, slider_range):
        self.start = slider_range[0]
        self.scale = slider_range[1] - slider_range[0]
        self.single_step = slider_range[2]

    def updateDisplayValues(self, slider):
        size = float(self.width() - 2 * self.bar_width - 1)
        # these are the sliders max/min
        self.display_min = int(size * (slider.getValues()[0] - self.start) / self.scale) + self.bar_width
        self.display_max = int(size * (slider.getValues()[1] - self.start) / self.scale) + self.bar_width
    #
    # def updateScaleValues(self, slider):
    #     size = float(self.width() - 2 * self.bar_width - 1)
    #     if (self.moving == "min") or (self.moving == "bar"):
    #         self.scale_min = self.start + (slider.getValues()[0] - self.bar_width) / float(size) * self.scale
    #         self.scale_min = float(round(slider.getValues()[0] / self.single_step)) * self.single_step
    #     if (self.moving == "max") or (self.moving == "bar"):
    #         self.scale_max = self.start + (slider.getValues()[1] - self.bar_width) / float(size) * self.scale
    #         self.scale_max = float(round(slider.getValues()[1] / self.single_step)) * self.single_step
    #     self.updateDisplayValues()
    #     self.update()

    def getPos(self, event):
        return event.x()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        w = self.width()
        h = self.height()

        # background
        painter.setPen(QtCore.Qt.gray)
        painter.setBrush(QtCore.Qt.lightGray)
        painter.drawRect(2, 2, w - 4, h - 4)

        for slice in self.slices:
            self.addSlice(slice)

    def addSlice(self, slice):
        h = self.height()

        size = float(self.width() - 2 * self.bar_width - 1)
        # these are the sliders max/min
        self.slices
        display_min = int(size * (slice.getValues()[0] - self.start) / self.scale) + self.bar_width
        display_max = int(size * (slice.getValues()[1] - self.start) / self.scale) + self.bar_width

        painter = QtGui.QPainter(self)
        # range bar
        painter.setPen(QtCore.Qt.darkGray)
        painter.setBrush(QtCore.Qt.darkGray)
        painter.drawRect(display_min - 1, 5, display_max - display_min + 2, h - 10)

        # min & max tabs
        painter.setPen(QtCore.Qt.black)
        painter.setBrush(QtCore.Qt.gray)
        painter.drawRect(display_min - self.bar_width, 1, self.bar_width, h - 2)

        painter.setPen(QtCore.Qt.black)
        painter.setBrush(QtCore.Qt.gray)
        painter.drawRect(display_max, 1, self.bar_width, h - 2)


if (__name__ == "__main__"):
    import sys


    class Parameters:
        def __init__(self):
            self.x_pixels = 200
            self.y_pixels = 200


    app = QtWidgets.QApplication(sys.argv)

    hslider = QRangeSlider(slider_range=[-5.0, 5.0, 0.5], values=[-4, -3])
    # hslider.setEmitWhileMoving(True)
    hslider.show()
    sys.exit(app.exec_())