#!/usr/bin/env python
"""
Qt Widget Multiple Slider widget.
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

    def setMax(self, max):
        self.max = max

    def getValues(self):
        return [self.min, self.max]

    def checkMinMax(self):
        if self.min > self.max:
            temp = self.min
            self.min = self.max
            self.max = temp

class QMultiSlider(QtWidgets.QWidget):
    """
    Range Slider super class.
    """
    doubleClick = QtCore.pyqtSignal()
    rangeChanged = QtCore.pyqtSignal(float, float)

    def __init__(self, slider_range, values, parent=None):
        QtWidgets.QWidget.__init__(self, parent)
        self.bar_width = 1
        self.bar_area_selection = 10
        self.emit_while_moving = 0
        self.slices = []
        self.setMouseTracking(True)
        self.moving = "none"

        self.current_display_min = 0
        self.current_display_max = 0

        # self.add_slice_button = QtWidgets.QPushButton('Add Slice', self)

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
        return 1
    #     self.add_slice_button.clicked.connect(self.addSliceAction)

    def addSliceAction(self):
        self.slices.append(Slice(3, 4))
        self.update()

    # def resizeEvent(self, event):
    #     for slice in self.slices:
    #         self.updateDisplayValues(slice)

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
        size = float(self.width() - 2 * self.bar_width - 1) # 'bar_width' is the width of the two sliders (the edges)

        # these are the visual sliders max/min (from value to visual)
        # rescale normalized value (slider.getValues()[0] - self.start) / self.scale) to visual length
        display_min = int(size * (slider.getValues()[0] - self.start) / self.scale) + self.bar_width
        display_max = int(size * (slider.getValues()[1] - self.start) / self.scale) + self.bar_width

        return [display_min, display_max]

    def updateCurrentSlice(self):
        size = float(self.width() - 2 * self.bar_width - 1)
        if (self.moving == "min") or (self.moving == "bar"):
            # inverse of updateCurrentSlice (from visual to value) .. and round it
            min_val = self.start + (self.current_display_min - self.bar_width) / float(size) * self.scale
            min_val = float(round(min_val / self.single_step)) * self.single_step
            self.active_slice.setMin(min_val)
        if (self.moving == "max") or (self.moving == "bar"):
            max_val = self.start + (self.current_display_max - self.bar_width) / float(size) * self.scale
            max_val = float(round(max_val / self.single_step)) * self.single_step
            self.active_slice.setMax(max_val)

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
        painter.setBrush(QtCore.Qt.darkGreen)
        # int x, int y, int width, int height
        # (background)
        painter.drawRect(0, 10, w, h - 4)

        # todo I don't need a resizeEvent because every time I change size paintEvent gets called which call addSlice which call updateDisplayValues
        for elem in self.slices:
            self.addSlice(elem)

    def addSlice(self, slice):
        painter = QtGui.QPainter(self)

        h = self.height()

        display_min, display_max = self.updateDisplayValues(slice)

        # range bar
        painter.setPen(QtCore.Qt.darkYellow)
        painter.setBrush(QtCore.Qt.green)
        # int x, int y, int width, int height
        painter.drawRect(display_min - 1, 10, display_max - display_min + 2, h - 10)

        # min & max tabs
        painter.setPen(QtCore.Qt.darkRed)
        # painter.setBrush(QtCore.Qt.black)
        painter.drawRect(display_min - self.bar_width, 1, self.bar_width, h - 2)

        painter.setPen(QtCore.Qt.darkRed)
        # painter.setBrush(QtCore.Qt.gray)
        painter.drawRect(display_max, 1, self.bar_width, h - 2)

        # tickz
        painter.setPen(QtCore.Qt.darkCyan)
        # painter.setBrush(QtCore.Qt.darkGreen)

        size = float(self.width() - 2 * self.bar_width)
        display_unit = size / self.scale * self.single_step
        display_ticks = [x * display_unit for x in list(range(int(self.scale / self.single_step)+1))]
        for tick_pos in display_ticks:
            painter.drawRect(tick_pos, 1, self.bar_width, 10)
        # rect = painter.drawText(QtCore.QRect(), QtCore.Qt.TextDontPrint, 'something')
        # left = self.width() // 2
        # bottom = self.rect().bottom()
        # pos = QtCore.QPoint(left, bottom)
        # painter.drawText(pos, 'porcacciodedio')

        # enlarge margins if clipping
        # if v == self.sl.minimum():
        #     if left <= 0:
        #         self.left_margin = rect.width() // 2 - x_loc
        #     if self.bottom_margin <= rect.height():
        #         self.bottom_margin = rect.height()
        #
        #     self.layout.setContentsMargins(self.left_margin,
        #                                    self.top_margin, self.right_margin,
        #                                    self.bottom_margin)
        #
        # if v == self.sl.maximum() and rect.width() // 2 >= self.right_margin:
        #     self.right_margin = rect.width() // 2
        #     self.layout.setContentsMargins(self.left_margin,
        #                                    self.top_margin, self.right_margin,
        #                                    self.bottom_margin)




    def mergeSlider(self):
        active_display_min, active_display_max = self.updateDisplayValues(self.active_slice)

        for slice in self.slices:
            if slice is not self.active_slice:
                display_min, display_max = self.updateDisplayValues(slice)

                if active_display_min <= display_max and active_display_max > display_max:
                    self.slices.remove(slice)
                    self.slices.remove(self.active_slice)
                    self.slices.append(Slice(slice.getValues()[0], self.active_slice.getValues()[1]))
                    self.moving = "none"
                    QtWidgets.QApplication.restoreOverrideCursor()
                elif active_display_max >= display_min and active_display_min < display_min:
                    self.slices.remove(slice)
                    self.slices.remove(self.active_slice)
                    self.slices.append(Slice(self.active_slice.getValues()[0], slice.getValues()[1]))
                    self.moving = "none"
                    QtWidgets.QApplication.restoreOverrideCursor()

    def getPos(self, event):
        return event.x()

    def mousePressEvent(self, event):
        pos = self.getPos(event)

        for slice in self.slices:
            # if abs(self.updateDisplayValues(slice)[0] - 0.5 * self.bar_area_selection - pos) < (0.5 * self.bar_area_selection):
            if self.getPos(event) >= (self.updateDisplayValues(slice)[0] - 0.5 * self.bar_area_selection) and \
                    self.getPos(event) <= (self.updateDisplayValues(slice)[0] + 0.5 * self.bar_area_selection):
                self.moving = "min"
                # print('slice', slice, 'min')
                self.active_slice = slice
                self.display_min = self.updateDisplayValues(self.active_slice)[0]
                self.display_max = self.updateDisplayValues(self.active_slice)[1]
            elif self.getPos(event) >= (self.updateDisplayValues(slice)[1] - 0.5 * self.bar_area_selection) and \
                    self.getPos(event) <= (self.updateDisplayValues(slice)[1] + 0.5 * self.bar_area_selection):
                self.moving = "max"
                # print('slice', slice, 'max')
                self.active_slice = slice
                self.display_min = self.updateDisplayValues(self.active_slice)[0]
                self.display_max = self.updateDisplayValues(self.active_slice)[1]
            elif (pos > self.updateDisplayValues(slice)[0]) and (pos < self.updateDisplayValues(slice)[1]):
                self.moving = "bar"
                self.active_slice = slice
                self.display_min = self.updateDisplayValues(self.active_slice)[0]
                self.display_max = self.updateDisplayValues(self.active_slice)[1]

            self.start_pos = pos

    def mouseMoveEvent(self, event):
        # this is to switch cursor when on slider
        # todo something is wrong when adding a slice, it does NOT change the cursor over the old slices
        if not event.buttons():
            for segment in self.slices:
                if self.getPos(event) >= (self.updateDisplayValues(segment)[0] - 0.5 * self.bar_area_selection) and \
                        self.getPos(event) <= (self.updateDisplayValues(segment)[0] + 0.5 * self.bar_area_selection):
                    QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.SplitHCursor)
                    break
                elif self.getPos(event) >= (self.updateDisplayValues(segment)[1] - 0.5 * self.bar_area_selection) and \
                        self.getPos(event) <= (self.updateDisplayValues(segment)[1] + 0.5 * self.bar_area_selection):
                    QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.SplitHCursor)
                    break
                else:
                    QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.ArrowCursor)

        else:

            size = self.width() # the length of the bar
            # the length of the bar
            # start pos the position clicked
            # getPos the current position
            # compute 'diff', which is the distance I moved from the point that i clicked
            diff = self.start_pos - self.getPos(event)
            if self.moving == "min":
                # compute 'temp', I apply diff on the display value corresponding to the slider value
                # point where I am now with the mouse w.r.t the display_min
                temp = self.display_min - diff

                if (temp >= self.bar_width) and (temp < size - self.bar_width):
                    # updated min position after dragging the slider
                    self.current_display_min = temp
                    self.updateCurrentSlice()
                    # logic to switch max with min:
                    if self.active_slice.getValues()[0] > self.active_slice.getValues()[1]:
                        # switch the min/max values of the current slice
                        temp_val = self.active_slice.getValues()[1]
                        self.active_slice.setMax(self.active_slice.getValues()[0])
                        self.active_slice.setMin(temp_val)
                        # reset start position 'start_pos'
                        self.start_pos = self.getPos(event)
                        # go to scenario 'max'
                        self.moving = 'max'
                        # make 'display_max' the point where I am now with the mouse w.r.t the display_min
                        self.display_max = temp

                    if self.emit_while_moving:
                        self.active_slice.emitRange()

            elif self.moving == "max":
                temp = self.display_max - diff
                if (temp >= self.bar_width) and (temp < size - self.bar_width):
                    self.current_display_max = temp
                    self.updateCurrentSlice()
                    if self.active_slice.getValues()[1] < self.active_slice.getValues()[0]:
                        temp_val = self.active_slice.getValues()[1]
                        self.active_slice.setMax(self.active_slice.getValues()[0])
                        self.active_slice.setMin(temp_val)
                        self.start_pos = self.getPos(event)
                        self.moving = 'min'
                        self.display_min = temp

                    if self.emit_while_moving:
                        self.active_slice.emitRange()
            elif self.moving == "bar":
                temp = self.display_min - diff
                if (temp >= self.bar_width) and (temp < size - self.bar_width - (self.display_max - self.display_min)):
                    self.current_display_min = temp
                    self.current_display_max = self.display_max - diff
                    self.updateCurrentSlice()

                    if self.emit_while_moving:
                        self.active_slice.emitRange()

    def mouseReleaseEvent(self, event):
        if not (self.moving == "none"):
            self.active_slice.emitRange()
        self.moving = "none"

if (__name__ == "__main__"):
    import sys

    app = QtWidgets.QApplication(sys.argv)

    hslider = QMultiSlider(slider_range=[-5.0, 5.0, 0.5], values=[-4, -3])
    # hslider.setEmitWhileMoving(True)
    hslider.show()
    sys.exit(app.exec_())