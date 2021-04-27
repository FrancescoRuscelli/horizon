#!/usr/bin/env python
"""
Qt Widget Range slider widget.
Hazen 06/13
"""

from PyQt5 import QtCore, QtGui, QtWidgets


class Slice():
    def __init__(self, min, max):
        self.min = min
        self.max = max

        self.single_step = 0.0
        self.moving = "none"
        self.old_min = 0.0
        self.old_max = 0.0
        self.scale = 0

    def emitRange(self):
        if (self.old_min != self.min) or (self.old_max != self.max):
            self.rangeChanged.emit(self.min, self.max)
            self.old_min = self.min
            self.old_max = self.max
            if 0:
                print("Range change:", self.min, self.max)

    def setValues(self, values):
        self.min = values[0]
        self.max = values[1]

    def getValues(self):
        return [self.min, self.max]

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

        self.add_slice_button = QtWidgets.QPushButton('Add Slice', self)

        self.resize(500, 400)

        if slider_range:
            self.setRange(slider_range)
        else:
            self.setRange([0.0, 1.0, 0.01])

        if values:
            # add a slice of the given range for each list
            # todo add more slices in constructor
            intial_slice = Slice(values[0], values[1])
            self.slices.append(intial_slice)
        else:
            self.slices.append(Slice(0.3, 0.6))

        self._connectActions()


    def _connectActions(self):
        self.add_slice_button.clicked.connect(self.addSliceAction)

    def addSliceAction(self):
        self.slices.append(Slice(3, 4))
        self.update()

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
        display_min = int(size * (slider.getValues()[0] - self.start) / self.scale) + self.bar_width
        display_max = int(size * (slider.getValues()[1] - self.start) / self.scale) + self.bar_width

        return [display_min, display_max]
    #
    def updateScaleValues(self, slider):
        size = float(self.width() - 2 * self.bar_width - 1)
        if (self.moving == "min") or (self.moving == "bar"):
            self.scale_min = self.start + (slider.getValues()[0] - self.bar_width) / float(size) * self.scale
            self.scale_min = float(round(slider.getValues()[0] / self.single_step)) * self.single_step
        if (self.moving == "max") or (self.moving == "bar"):
            self.scale_max = self.start + (slider.getValues()[1] - self.bar_width) / float(size) * self.scale
            self.scale_max = float(round(slider.getValues()[1] / self.single_step)) * self.single_step
        self.updateDisplayValues()
        self.update()

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

        for elem in self.slices:
            self.addSlice(elem)

    def addSlice(self, slice):
        painter = QtGui.QPainter(self)

        h = self.height()

        display_min, display_max = self.updateDisplayValues(slice)

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

    def getPos(self, event):
        return event.x()

    def mousePressEvent(self, event):
        pos = self.getPos(event)

        for slice in self.slices:
            if abs(self.updateDisplayValues(slice)[0] - 0.5 * self.bar_width - pos) < (0.5 * self.bar_width):
                self.moving = "min"
                print('slice', slice, 'min')
                self.current_slice = slice
                self.current_display_min = self.updateDisplayValues(self.current_slice)[0]
                self.current_display_max = self.updateDisplayValues(self.current_slice)[1]
            elif abs(self.updateDisplayValues(slice)[1] + 0.5 * self.bar_width - pos) < (0.5 * self.bar_width):
                self.moving = "max"
                print('slice', slice, 'max')
                self.current_slice = slice
                self.current_display_min = self.updateDisplayValues(self.current_slice)[0]
                self.current_display_max = self.updateDisplayValues(self.current_slice)[1]
            elif (pos > self.updateDisplayValues(slice)[0]) and (pos < self.updateDisplayValues(slice)[1]):
                self.moving = "bar"
                self.current_slice = slice
                self.current_display_min = self.updateDisplayValues(self.current_slice)[0]
                self.current_display_max = self.updateDisplayValues(self.current_slice)[1]


            self.start_pos = pos

    def mouseMoveEvent(self, event):
        size = self.rangeSliderSize()
        diff = self.start_pos - self.getPos(event)
        if self.moving == "min":
            temp = self.current_display_min - diff
            if (temp >= self.bar_width) and (temp < size - self.bar_width):
                self.display_min = temp
                if self.display_max < self.display_min:
                    self.display_max = self.display_min
                self.updateScaleValues()
                if self.emit_while_moving:
                    self.emitRange()
        elif self.moving == "max":
            temp = self.current_display_max - diff
            if (temp >= self.bar_width) and (temp < size - self.bar_width):
                self.display_max = temp
                if self.display_max < self.display_min:
                    self.display_min = self.display_max
                self.updateScaleValues()
                if self.emit_while_moving:
                    self.emitRange()
        elif self.moving == "bar":
            temp = self.current_display_min - diff
            if (temp >= self.bar_width) and (
                    temp < size - self.bar_width - (self.current_display_max - self.current_display_min)):
                self.display_min = temp
                self.display_max = self.current_display_max - diff
                self.updateScaleValues()
                if self.emit_while_moving:
                    self.emitRange()
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