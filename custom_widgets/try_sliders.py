#!/usr/bin/env python
"""
Qt Widget Range slider widget.
Hazen 06/13
"""

from PyQt5 import QtCore, QtGui, QtWidgets


class Slice(QtWidgets.QWidget):
    rangeChanged = QtCore.pyqtSignal(float, float)

    def __init__(self, min, max):
        QtWidgets.QWidget.__init__(self)
        self.min = min
        self.max = max

        self.single_step = 0.0
        self.old_min = 0.0
        self.old_max = 0.0
        self.scale = 0

    def emitRange(self):
        if (self.old_min != self.min) or (self.old_max != self.max):
            self.rangeChanged.emit(self.min, self.max)
            self.old_min = self.min
            self.old_max = self.max
            if 1:
                print("Range change:", self.min, self.max)

    def setValues(self, values):
        self.min = values[0]
        self.max = values[1]
        self.checkMinMax()

    def setMin(self, min):
        self.min = min
        # self.checkMinMax()

    def setMax(self, max):
        self.max = max
        # self.checkMinMax()

    def getValues(self):
        return [self.min, self.max]

    def checkMinMax(self):
        if self.min > self.max:
            temp = self.min
            self.min = self.max
            self.max = temp


    # def enterEvent(self, event):
    #     print("Mouse Entered")
    #     return super(Slice, self).enterEvent(event)
    #
    # def leaveEvent(self, event):
    #     print("Mouse Left")
    #     return super(Slice, self).enterEvent(event)


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
        self.setMouseTracking(True)
        self.moving = "none"

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
    def setEmitWhileMoving(self, flag):
        if flag:
            self.emit_while_moving = 1
        else:
            self.emit_while_moving = 0

    def setRange(self, slider_range):
        self.start = slider_range[0]
        self.scale = slider_range[1] - slider_range[0]
        self.single_step = slider_range[2]

    def updateDisplayValues(self, slider):
        size = float(self.width() - 2 * self.bar_width - 1) # bar width is the width of the two sliders (the edges)


        # these are the visual sliders max/min
        display_min = int(size * (slider.getValues()[0] - self.start) / self.scale) + self.bar_width
        display_max = int(size * (slider.getValues()[1] - self.start) / self.scale) + self.bar_width

        print('(slider.getValues()[0] - self.start)', (slider.getValues()[0] - self.start))
        print('int(size * (slider.getValues()[0] - self.start) / self.scale)', int(size * (slider.getValues()[0] - self.start) / self.scale)  )
        print('self.width()', self.width())
        print('size', size)

        print('display_min', display_min)
        print('display_max', display_max)

        return [display_min, display_max]
    #
    def updateCurrentSlice(self):
        size = float(self.width() - 2 * self.bar_width - 1)
        if (self.moving == "min") or (self.moving == "bar"):
            min_val = self.start + (self.current_slice_visual_min - self.bar_width) / float(size) * self.scale
            min_val = float(round(min_val / self.single_step)) * self.single_step
            self.current_slice.setMin(min_val)
        if (self.moving == "max") or (self.moving == "bar"):
            max_val = self.start + (self.current_slice_visual_max - self.bar_width) / float(size) * self.scale
            max_val = float(round(max_val / self.single_step)) * self.single_step
            self.current_slice.setMax(max_val)

        self.mergeSlider()
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

    def mergeSlider(self):

        current_display_min, current_display_max = self.updateDisplayValues(self.current_slice)

        for slice in self.slices:
            if slice is not self.current_slice:
                display_min, display_max = self.updateDisplayValues(slice)

                if current_display_min < display_max and current_display_max > display_max:
                    self.slices.remove(slice)
                    self.slices.remove(self.current_slice)
                    self.slices.append(Slice(slice.getValues()[0], self.current_slice.getValues()[1]))
                    self.moving = "none"
                    QtWidgets.QApplication.restoreOverrideCursor()
                elif current_display_max > display_min and current_display_min < display_min:
                    self.slices.remove(slice)
                    self.slices.remove(self.current_slice)
                    self.slices.append(Slice(self.current_slice.getValues()[0], slice.getValues()[1]))
                    self.moving = "none"
                    QtWidgets.QApplication.restoreOverrideCursor()

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

        # this is to switch cursor
        if not event.buttons():
            for slice in self.slices:
                if abs(self.updateDisplayValues(slice)[0] - 0.5 * self.bar_width - self.getPos(event)) < (0.5 * self.bar_width):
                    QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.SplitHCursor)
                elif abs(self.updateDisplayValues(slice)[1] + 0.5 * self.bar_width - self.getPos(event)) < (0.5 * self.bar_width):
                    QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.SplitHCursor)
                else:
                    QtWidgets.QApplication.restoreOverrideCursor()
        else:

            size = self.width() # the length of the bar
            # the length of the bar
            # start pos the position clicked
            # getPos the current position
            diff = self.start_pos - self.getPos(event)

            if self.moving == "min":
                # todo add logic to switch max with min
                temp = self.current_display_min - diff
                if (temp >= self.bar_width) and (temp < size - self.bar_width):
                    self.current_slice_visual_min = temp


                    self.updateCurrentSlice()

                    # if self.current_slice.getValues()[0] - self.current_slice.getValues()[1] > self.single_step:
                    #     pisello = self.current_slice.getValues()[0]
                    #     print('min:', self.current_slice.getValues()[0])
                    #     print('max:', self.current_slice.getValues()[1])
                    #     self.current_slice.setMin(self.current_slice.getValues()[1])
                    #     self.current_slice.setMax(pisello) # ??
                    #     print('min-max:', self.current_slice.getValues())
                    #     print('--------------------------------------')

                    if self.emit_while_moving:
                        self.current_slice.emitRange()
            elif self.moving == "max":
                temp = self.current_display_max - diff
                if (temp >= self.bar_width) and (temp < size - self.bar_width):
                    self.current_slice_visual_max = temp
                    self.updateCurrentSlice()
                    if self.emit_while_moving:
                        self.current_slice.emitRange()
            elif self.moving == "bar":
                temp = self.current_display_min - diff
                if (temp >= self.bar_width) and (temp < size - self.bar_width - (self.current_display_max - self.current_display_min)):
                    self.current_slice_visual_min = temp
                    self.current_slice_visual_max = self.current_display_max - diff
                    self.updateCurrentSlice()
                    if self.emit_while_moving:
                        self.current_slice.emitRange()



    def mouseReleaseEvent(self, event):
        if not (self.moving == "none"):
            self.current_slice.emitRange()
        self.moving = "none"

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