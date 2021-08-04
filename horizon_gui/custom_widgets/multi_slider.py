"""
Qt Widget Multiple Slider widget.
"""

from PyQt5 import QtCore, QtGui, QtWidgets
from functools import partial

class TicksLine(QtWidgets.QWidget):
    def __init__(self, scale, single_step, bar_width, parent=None):
        super().__init__(parent)


        self.bar_width = bar_width
        self.scale = scale
        self.single_step = single_step
        self.bar_tickz_heigth = 10
        self.numbers_bar_heigth = 30
        self.numbers_border_bar = 15
        self.side_border = 10
        self.setMaximumHeight(self.numbers_bar_heigth)
        # self.setContentsMargins(0, 0, 0, 0)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        w = self.width()

        painter.setBrush(QtCore.Qt.blue)
        painter.drawRect(0, 0, w, self.numbers_bar_heigth)

        # tickz
        painter.setPen(QtCore.Qt.darkCyan)

        size = float(self.width() - 2 * self.bar_width) - self.side_border *2
        display_unit = size / self.scale * self.single_step
        display_ticks = [self.side_border + x * display_unit for x in list(range(int(self.scale / self.single_step) + 1))]

        for tick_pos in display_ticks:
            painter.drawRect(int(tick_pos), self.numbers_bar_heigth-self.bar_tickz_heigth, self.bar_width, self.bar_tickz_heigth)

        i = 0
        for tick_pos in display_ticks:
            painter.setPen(QtCore.Qt.red)
            painter.drawText(QtCore.QPoint(int(tick_pos) - 3*len(str(i)), self.numbers_bar_heigth - self.numbers_border_bar), str(i))
            i = i + 1


class AbstractSlice(QtWidgets.QWidget):
    def __init__(self, min, max):
        QtWidgets.QWidget.__init__(self)
        self.min = min
        self.max = max

    def getValues(self):
        int_min = self.min
        int_max = self.max
        return [int_min, int_max]

    def checkMinMax(self):
        if self.min > self.max:
            temp = self.min
            self.min = self.max
            self.max = temp

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

    # def emitRange(self):
    #     if (self.old_min != self.min) or (self.old_max != self.max):
    #         self.rangeChanged.emit(self.min, self.max)
    #         self.old_min = self.min
    #         self.old_max = self.max

    def setValues(self, values):
        self.min = values[0]
        self.max = values[1]
        self.checkMinMax()

    def setMin(self, min):
        self.min = min

    def setMax(self, max):
        self.max = max

    def getValues(self):
        int_min = self.min
        int_max = self.max
        return [int_min, int_max]

    def checkMinMax(self):
        if self.min > self.max:
            temp = self.min
            self.min = self.max
            self.max = temp


class QMultiSlider(QtWidgets.QWidget):
    """
    Range Slider super class.
    """
    slicesChanged = QtCore.pyqtSignal(list)

    def __init__(self, slider_range, values=None, parent=None, options=None, number_bar=False):
        super().__init__(parent)

        # todo experimental to add number
        if number_bar:
            self.numbers_bar_heigth = 20
        else:
            self.numbers_bar_heigth = 0

        self.options = dict()
        self.options['background_color'] = QtCore.Qt.darkGray
        self.options['slice_color'] = QtCore.Qt.gray
        self.options['minmax_color'] = QtCore.Qt.darkRed
        self.options['ticks_color'] = QtCore.Qt.darkCyan
        self.options['number_bar_color'] = QtCore.Qt.darkYellow
        self.options['number_color'] = QtCore.Qt.black
        self.options['disabled_slice_color'] = QtCore.Qt.green


        if options is not None:
            self.options.update(options)

        QtWidgets.QWidget.__init__(self, parent)

        self.setMinimumSize(500, 100)
        self.bar_width = 1
        self.bar_area_selection = 10
        self.emit_while_moving = 0
        self.slices = []
        self.setMouseTracking(True)
        self.moving = "none"
        self.bar_tickz_heigth = 10
        self.current_display_min = 0
        self.current_display_max = 0

        if slider_range:
            self.setRange(slider_range)
        else:
            self.setRange([0.0, 1.0, 0.01])

        if values:
            # add a slice of the given range for each list
            # todo add more slices in constructor
            if values[0] <= slider_range[0]:
                values[0] = slider_range[0]
                if values[1] <= slider_range[0]:
                    values[1] = slider_range[0]

            if values[1] >= slider_range[1]:
                values[1] = slider_range[1]
                if values[0] >= slider_range[1]:
                    values[0] = slider_range[1]

            initial_slice = Slice(values[0], values[1])
            self._addSlice(initial_slice)
        else:
            self._addSlice()

        self._connectActions()

    def _connectActions(self):
        return 1

    def disableValues(self, min, max):
        disabled_value = AbstractSlice(min, max)
        self._addSlice(disabled_value)

    def contextMenuEvent(self, event):

        # todo reset all slices??
        # self.resetAction = QtWidgets.QAction()
        # self.resetAction.setText('&Reset')
        # self.resetAction.triggered.connect(self.reset)
        # menu.addAction(self.resetAction)


        flag_disabled = False
        for slice in self.slices:
            if isinstance(slice, AbstractSlice):
                if event.pos().x() > self._fromValueToDisplay(slice)[0] and event.pos().x() < self._fromValueToDisplay(slice)[1]:
                    flag_disabled = True

        if not flag_disabled:
            menu = QtWidgets.QMenu(self)
            self.newSliceAction = QtWidgets.QAction()
            self.newSliceAction.setText("&New Slice")
            self.newSliceAction.setIcon(QtGui.QIcon("../resources/slice.png"))

            val = self._fromDisplayToRange(event.pos().x(), floor=True)

            new_slice = Slice(val, val+ self.single_step)
            self.newSliceAction.triggered.connect(partial(self._addSlice, new_slice))

            menu.addAction(self.newSliceAction)

            for slice in self.slices:
                if (event.pos().x() > self._fromValueToDisplay(slice)[0]) and (event.pos().x() < self._fromValueToDisplay(slice)[1]):
                    self.removeSliceAction = QtWidgets.QAction()
                    self.removeSliceAction.setText("&Remove Slice")
                    self.removeSliceAction.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_TrashIcon))
                    self.removeSliceAction.triggered.connect(partial(self._removeSlice, slice))
                    menu.addAction(self.removeSliceAction)

        menu.exec_(event.globalPos())

    def _reset(self):
        self.slices.clear()
        self.update()

    def _addSlice(self, slice=None):
        if slice is None:
            slice = Slice(0, self.scale + self.start)

        self.slices.append(slice)
        # set as active slice the one created
        self.active_slice = slice

        self.display_min = self._fromValueToDisplay(self.active_slice)[0]
        self.display_max = self._fromValueToDisplay(self.active_slice)[1]

        # run merge slider if new slider is over one slider already created

        self._emitRanges()
        self._mergeSlider()
        self.update()

    def _removeSlice(self, item):
        self.slices.remove(item)
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

        self._checkScale(slider_range)

    def _fromValueToDisplay(self, slider):
        size = float(self.width() - 2 * self.bar_width - 1) # 'bar_width' is the width of the two sliders (the edges)

        # these are the visual sliders max/min (from value to visual)
        # rescale normalized value (slider.getValues()[0] - self.start) / self.scale) to visual length

        display_min = int(size * (slider.getValues()[0] - self.start) / self.scale) + self.bar_width
        display_max = int(size * (slider.getValues()[1] - self.start) / self.scale) + self.bar_width

        return [display_min, display_max]

    def _fromDisplayToRange(self, display_val, floor=False):

        size = float(self.width() - 2 * self.bar_width - 1)
        range_val = self.start + (display_val - self.bar_width) / float(size) * self.scale
        if floor:
            range_val = float(range_val // self.single_step) * self.single_step
        else:
            range_val = float(round(range_val / self.single_step)) * self.single_step

        return range_val


    def _updateCurrentSlice(self):

        if not isinstance(self.active_slice, AbstractSlice):
            if (self.moving == "min") or (self.moving == "bar"):
                # inverse of updateCurrentSlice (from visual to value) .. and round it
                min_val = self._fromDisplayToRange(self.current_display_min)
                self.active_slice.setMin(min_val)
            if (self.moving == "max") or (self.moving == "bar"):
                max_val = self._fromDisplayToRange(self.current_display_max)
                self.active_slice.setMax(max_val)

            self._mergeSlider()
        self.update()

    def _getPos(self, event):
        return event.x()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        w = self.width()
        h = self.height()

        # background
        # painter.setPen(QtCore.Qt.gray)
        # int x, int y, int width, int height
        # (background)
        num_border = 5
        if self.numbers_bar_heigth != 0:
            painter.setBrush(self.options['number_bar_color'])
            painter.drawRect(0, 0, w, self.numbers_bar_heigth)

        painter.setBrush(self.options['background_color'])
        painter.drawRect(0, self.bar_tickz_heigth + self.numbers_bar_heigth, w, h)

        # tickz
        painter.setPen(self.options['ticks_color'])
        size = float(self.width() - 2 * self.bar_width)
        display_unit = size / self.scale * self.single_step
        display_ticks = [x * display_unit for x in list(range(int(self.scale / self.single_step) + 1))]

        for tick_pos in display_ticks:
            painter.drawRect(int(tick_pos), self.numbers_bar_heigth, self.bar_width, self.bar_tickz_heigth)

        i = 0
        for tick_pos in display_ticks:
            painter.setPen(self.options['number_color'])
            if i == 0:
                tick_pos = 5
            if i == len(display_ticks)-1:
                tick_pos = tick_pos-10
            painter.drawText(QtCore.QPoint(int(tick_pos)-3, self.numbers_bar_heigth-num_border), str(i))
            i = i+1



        # todo I don't need a resizeEvent because every time I change size paintEvent gets called which call addSlice which call updateDisplayValues
        for elem in self.slices:
            self._paintSlice(elem)

    def _paintSlice(self, slice):
        painter = QtGui.QPainter(self)

        h = self.height()
        display_min, display_max = self._fromValueToDisplay(slice)

        # range bar
        # painter.setPen(QtCore.Qt.darkYellow)
        if isinstance(slice, AbstractSlice):
            painter.setBrush(self.options['disabled_slice_color'])
        else:
            painter.setBrush(self.options['slice_color'])
        # int x, int y, int width, int height
        painter.drawRect(display_min - 1, self.bar_tickz_heigth + self.numbers_bar_heigth, display_max - display_min + 2, h)

        # min & max tabs
        if isinstance(slice, AbstractSlice):
            painter.setBrush(self.options['disabled_slice_color'])
        else:
            painter.setPen(self.options['minmax_color'])

        painter.drawRect(display_min - self.bar_width, 1+self.numbers_bar_heigth, self.bar_width, h - 2)
        painter.drawRect(display_max, 1+self.numbers_bar_heigth, self.bar_width, h - 2)

    def _mergeSlider(self):
        active_display_min, active_display_max = self._fromValueToDisplay(self.active_slice)
        # print('entered merge')
        # print('all slices', self.slices)
        # print('current slice', self.active_slice)
        for slice in self.slices:
            if slice is not self.active_slice and not isinstance(slice, AbstractSlice):
                display_min, display_max = self._fromValueToDisplay(slice)

                if active_display_min <= display_max < active_display_max:
                    self._removeSlice(slice)
                    self._removeSlice(self.active_slice)
                    # self.slices.remove(slice)
                    # self.slices.remove(self.active_slice)
                    new_slice = Slice(slice.getValues()[0], self.active_slice.getValues()[1])
                    self._addSlice(new_slice)
                    # self.slices.append(new_slice)
                    self.moving = "none"
                    QtWidgets.QApplication.restoreOverrideCursor()

                    # self.active_slice = new_slice
                elif active_display_max >= display_min > active_display_min:
                    self._removeSlice(slice)
                    self._removeSlice(self.active_slice)
                    # self.slices.remove(slice)
                    # self.slices.remove(self.active_slice)
                    new_slice = Slice(self.active_slice.getValues()[0], slice.getValues()[1])
                    self._addSlice(new_slice)
                    # self.slices.append(new_slice)
                    self.moving = "none"
                    QtWidgets.QApplication.restoreOverrideCursor()

                    # self.active_slice = new_slice
                elif active_display_max <= display_max and active_display_min >= display_min:
                    self._removeSlice(slice)
                    self._removeSlice(self.active_slice)
                    # self.slices.remove(slice)
                    # self.slices.remove(self.active_slice)
                    new_slice = Slice(slice.getValues()[0], slice.getValues()[1])
                    self._addSlice(new_slice)
                    # self.slices.append(new_slice)
                    self.moving = "none"
                    QtWidgets.QApplication.restoreOverrideCursor()

                    # self.active_slice = new_slice


    def mousePressEvent(self, event):
        pos = self._getPos(event)

        for slice in self.slices:
            # if abs(self.updateDisplayValues(slice)[0] - 0.5 * self.bar_area_selection - pos) < (0.5 * self.bar_area_selection):
            if (self._fromValueToDisplay(slice)[0] - 0.5 * self.bar_area_selection) <= self._getPos(event) <= (self._fromValueToDisplay(slice)[0] + 0.5 * self.bar_area_selection):
                self.moving = "min"
                # print('slice', slice, 'min')
                self.active_slice = slice
                self.display_min = self._fromValueToDisplay(self.active_slice)[0]
                self.display_max = self._fromValueToDisplay(self.active_slice)[1]
            elif (self._fromValueToDisplay(slice)[1] - 0.5 * self.bar_area_selection) <= self._getPos(event) <= (self._fromValueToDisplay(slice)[1] + 0.5 * self.bar_area_selection):
                self.moving = "max"
                # print('slice', slice, 'max')
                self.active_slice = slice
                self.display_min = self._fromValueToDisplay(self.active_slice)[0]
                self.display_max = self._fromValueToDisplay(self.active_slice)[1]
            elif (pos > self._fromValueToDisplay(slice)[0]) and (pos < self._fromValueToDisplay(slice)[1]):
                self.moving = "bar"
                self.active_slice = slice
                self.display_min = self._fromValueToDisplay(self.active_slice)[0]
                self.display_max = self._fromValueToDisplay(self.active_slice)[1]

            self.start_pos = pos

    def mouseMoveEvent(self, event):
        # this is to switch cursor when on slider
        # todo something is wrong when adding a slice, it does NOT change the cursor over the old slices
        if not event.buttons(): #because only hovering on
            for segment in self.slices:
                if (self._fromValueToDisplay(segment)[0] - 0.5 * self.bar_area_selection) <= self._getPos(event) <= (self._fromValueToDisplay(segment)[0] + 0.5 * self.bar_area_selection):
                    self.setCursor(QtCore.Qt.SplitHCursor)
                    break
                elif (self._fromValueToDisplay(segment)[1] - 0.5 * self.bar_area_selection) <= self._getPos(event) <= (self._fromValueToDisplay(segment)[1] + 0.5 * self.bar_area_selection):
                    self.setCursor(QtCore.Qt.SplitHCursor)
                    break
                else:
                    self.setCursor(QtCore.Qt.ArrowCursor)
        else:

            # computing min and max.
            # basically, i'm searching for the biggest maximum in all the AbstractSlice,
            # and I'm setting it as the minimum in the mouseMoveEvent.
            # so the active_slice can only get to that minimum.
            # same for the maximum

            active_min = 0
            for slice in self.slices:
                if isinstance(slice, AbstractSlice):
                    if self.display_min >= self._fromValueToDisplay(slice)[1]:
                        if self._fromValueToDisplay(slice)[1] > active_min:
                            active_min = self._fromValueToDisplay(slice)[1]

            active_max = self.width()
            for slice in self.slices:
                if isinstance(slice, AbstractSlice):
                    if self.display_max <= self._fromValueToDisplay(slice)[0]:
                        if self._fromValueToDisplay(slice)[0] < active_max:
                            active_max = self._fromValueToDisplay(slice)[0]

            size = float(self.width() - 2 * self.bar_width - 1)
            single_step_display = int(size * (self.single_step - self.start) / self.scale) + self.bar_width
            active_min = active_min + single_step_display
            active_max = active_max - single_step_display
            size = self.width() # the length of the whole slider bar
            # start pos --> the position clicked
            # getPos --> the current position
            # compute 'diff', which is the distance I moved from the point that i clicked
            diff = self.start_pos - self._getPos(event)
            if self.moving == "min":
                # compute 'temp', I apply diff on the display value corresponding to the slider value
                # point where I am now with the mouse w.r.t the display_min
                temp = self.display_min - diff
                # this is the maximum and the minimum (minimum--> bar_width (1) maximum self.width())
                if temp >= active_min + self.bar_width: # todo i may be wrong but this is useless --> and (temp < size - self.bar_width):
                    # updated min position after dragging the slider
                    self.current_display_min = temp
                    self._updateCurrentSlice()
                    # logic to switch max with min:
                    if self.active_slice.getValues()[0] > self.active_slice.getValues()[1]:
                        # switch the min/max values of the current slice
                        temp_val = self.active_slice.getValues()[1]
                        self.active_slice.setMax(self.active_slice.getValues()[0])
                        self.active_slice.setMin(temp_val)
                        # reset start position 'start_pos'
                        self.start_pos = self._getPos(event)
                        # go to scenario 'max'
                        self.moving = 'max'
                        # make 'display_max' the point where I am now with the mouse w.r.t the display_min
                        self.display_max = temp

                    if self.emit_while_moving:
                        self._emitRanges()

            elif self.moving == "max":
                temp = self.display_max - diff
                if temp < active_max - self.bar_width: # todo I may be wrong but this is useless (temp >= self.bar_width)
                    self.current_display_max = temp
                    self._updateCurrentSlice()
                    if self.active_slice.getValues()[1] < self.active_slice.getValues()[0]:
                        temp_val = self.active_slice.getValues()[1]
                        self.active_slice.setMax(self.active_slice.getValues()[0])
                        self.active_slice.setMin(temp_val)
                        self.start_pos = self._getPos(event)
                        self.moving = 'min'
                        self.display_min = temp

                    if self.emit_while_moving:
                        self._emitRanges()

            elif self.moving == "bar":
                temp = self.display_min - diff
                if (temp > active_min + self.bar_width) and (temp < active_max - self.bar_width - (self.display_max - self.display_min)):
                    self.current_display_min = temp
                    self.current_display_max = self.display_max - diff
                    self._updateCurrentSlice()

                    if self.emit_while_moving:
                        self._emitRanges()

    def _emitRanges(self):
        # todo should emit ALL THE VALUES and not only .min .max?
        all_ranges = [slice.getValues() for slice in self.slices]
        self.on_slices_changed(all_ranges)
        # self.active_slice.emitRange()

    def mouseReleaseEvent(self, event):
        if not (self.moving == "none"):
            self._emitRanges()
        self.moving = "none"

    def _checkScale(self, slider_range):
        # this is to handle the slider if the range is smaller than the single step
        if slider_range[1] - slider_range[0] < slider_range[2]:
            self.scale = slider_range[2]
            self.setEnabled(False)
            self.slices.clear()
        else:
            self.setEnabled(True)

    def updateRange(self, slider_range):
        self.start = slider_range[0]
        self.scale = slider_range[1] - slider_range[0]

        self._checkScale(slider_range + [self.single_step])

        for slice in self.slices:
            if slice.getValues()[1] > slider_range[1]:
                slice.setMax(slider_range[1])
            if slice.getValues()[0] < slider_range[0]:
                slice.setMin(slider_range[0])

    def updateSlices(self, ranges):
        self._reset()

        if any(isinstance(el, list) for el in ranges):
            for range in ranges:
                new_slice = Slice(range[0], range[1])
                self.slices.append(new_slice)
        else:
            new_slice = Slice(ranges[0], ranges[1])
            self.slices.append(new_slice)

        # self.active_slice = new_slice
        # self.display_min = self.fromValueToDisplay(self.active_slice)[0]
        # self.display_max = self.fromValueToDisplay(self.active_slice)[1]

    def getRanges(self):

        # todo make this for everything
        elements = [slice.getValues() for slice in self.slices]

        for i in range(len(elements)):
            if isinstance(elements[i], list):
                for j in range(len(elements[i])):
                    elements[i][j] = int(elements[i][j])
            else:
                elements[i] = int(elements[i])

        if any(isinstance(el, list) for el in elements):
            unraveled_elem = list()
            for el in elements:
                temp = list(range(el[0], el[1] + 1))  # +1 # todo cannot add+1 here?
                for item in temp:
                    unraveled_elem.append(item) if item not in unraveled_elem else unraveled_elem
        elif isinstance(elements, list):
            unraveled_elem = list()
            temp = list(range(elements[0], elements[1] + 1))  # +1
            for item in temp:
                unraveled_elem.append(item)

        return unraveled_elem

    @QtCore.pyqtSlot()
    def on_slices_changed(self, ranges):
        self.slicesChanged.emit(ranges)

class MultiSliderWithNumbers(QtWidgets.QWidget):
    def __init__(self, slider_range, values=None, parent=None, options=None):
        super(MultiSliderWithNumbers, self).__init__(parent)

        self.scale = slider_range[1] - slider_range[0]

        self.main_layout = QtWidgets.QVBoxLayout(self)
        self.main_layout.setSpacing(0)

        numbers_bar = TicksLine(self.scale, single_step=1, bar_width=1)
        self.main_layout.addWidget(numbers_bar)

        self.window_bordered = QtWidgets.QWidget()
        self.window_bordered_layout = QtWidgets.QHBoxLayout(self.window_bordered)
        hslider = QMultiSlider(slider_range=[-5.0, 5.0, 1], values=[1, 2])
        self.window_bordered_layout.addWidget(hslider)
        self.window_bordered_layout.setContentsMargins(10, 0, 10, 0)
        self.main_layout.addWidget(self.window_bordered)


if (__name__ == "__main__"):
    import sys

    app = QtWidgets.QApplication(sys.argv)
    # mywidget = MyWidget()
    # mywidget.show()
    hslider = QMultiSlider([0, 30, 1], values=[7,9], number_bar=True)

    hslider.disableValues(0, 2)
    hslider.disableValues(5, 5)

    hslider.disableValues(20, 25)
    hslider.disableValues(26, 30)

    hslider.show()

    # hslider.setEmitWhileMoving(True)
    sys.exit(app.exec_())
