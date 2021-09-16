import numpy as np
from PyQt5.QtWidgets import QWidget, QGridLayout, QVBoxLayout, QPushButton, QApplication, QRubberBand, QLabel
from PyQt5.QtGui import QPalette, QFont
from PyQt5.QtCore import Qt, QRect, QSize, QPoint, pyqtSlot, pyqtSignal
from horizon_gui.custom_widgets.infinity_spinbox import InfinitySpinBox
from functools import partial

import sys


class SpinboxLine(QWidget):
    valueChanged = pyqtSignal(int, list)

    def __init__(self, title_name, nodes, dim, disabled_nodes=None, initial_values=None, parent=None, colors=None):
        super().__init__(parent)

        self.dim = dim
        self.n_nodes = nodes
        self.values_matrix = np.matrix(np.ones((dim, nodes)) * 0.)
        self.hidden_nodes = []

        if disabled_nodes:
            self.disabled_nodes = disabled_nodes
            self.enabled_nodes = [node for node in range(nodes) if node not in disabled_nodes]
        else:
            self.disabled_nodes = []
            self.enabled_nodes = range(nodes)

        if initial_values is None:
            self.initial_values = np.ones([dim, nodes]) * 0.
        else:
            self.initial_values = initial_values


        if colors:
            self.color_base = colors['base']
            self.color_selected = colors['selected']
            self.color_background = colors['background']
            self.color_border = colors['border']
        else:
            self.color_base = "MediumSeaGreen"
            self.color_selected = "Crimson"
            self.color_background = "rgb(255,0,0)"
            self.color_border = "rgb(0, 255, 0)"

        self.setStyleSheet("background-color:" + self.color_background + "; border:1px solid " + self.color_border + ";")

        # main layout
        self.main_layout = QVBoxLayout(self)
        self.main_layout.setContentsMargins(0, 0, 0, 0)
        self.main_layout.setSpacing(0)

        # spinboxes line
        self.spinbox_widget = QWidget()
        self.spinbox_layout = QGridLayout(self.spinbox_widget)
        self.spinbox_layout.setContentsMargins(0, 20, 0, 20)
        self.spinbox_layout.setSpacing(0)

        palette = QPalette()
        palette.setColor(QPalette.Text, Qt.red)

        font = QFont("Times", 12, QFont.Bold)

        title = QLabel(title_name)
        title.setAlignment(Qt.AlignCenter)
        title.setPalette(palette)
        title.setFont(font)

        self.main_layout.addWidget(title)
        self.main_layout.addWidget(self.spinbox_widget)

        self.rubberband = QRubberBand(QRubberBand.Rectangle, self)
        self.selected = []


        if disabled_nodes is None:
            disabled_nodes = []

        self.setValues(self.initial_values)
        self.setNodes(nodes)

        self.setHiddenNodes(disabled_nodes)

        QApplication.instance().focusChanged.connect(self.on_focusChanged)

    def setNodes(self, nodes):
        self.n_nodes = nodes
        self._updateNodes()

    # set bounds at specific nodes
    def setValues(self, bounds, nodes=None):
        if nodes is None:
            self.values_matrix[:, self.enabled_nodes] = bounds
        else:
            self.values_matrix[:, nodes] = bounds
        self._updateNodes()

    def _updateNodes(self):
        # Remove all the nodes
        for i in reversed(range(self.spinbox_layout.count())):
            self.spinbox_layout.itemAt(i).widget().deleteLater()
            self.spinbox_layout.removeItem(self.spinbox_layout.itemAt(i))

        temp_values = np.matrix(np.zeros((self.dim, self.n_nodes)))

        temp_values[:self.values_matrix.shape[0], :self.values_matrix.shape[1]] = self.values_matrix[:temp_values.shape[0], :temp_values.shape[1]]

        self.values_matrix = temp_values

        self.createCustomSpinboxLine()
        self.hideNodes()

        # hide nodes
        for i in range(self.dim):
            for j in range(self.n_nodes):
                spinbox = self.spinbox_layout.itemAtPosition(i, j).widget()
                if j in self.hidden_nodes:
                    sp_retain = spinbox.sizePolicy()
                    sp_retain.setRetainSizeWhenHidden(True)
                    spinbox.setSizePolicy(sp_retain)
                    spinbox.hide()

    def createCustomSpinboxLine(self):
        for i in range(self.dim):
            for j in range(self.n_nodes):
                n_i = InfinitySpinBox(color_base=self.color_base, color_selected=self.color_selected)
                n_i.setValue(self.values_matrix[i, j])
                n_i.valueChanged.connect(self.multipleSet)
                n_i.valueChanged.connect(partial(self.updateValues, i, j))
                self.spinbox_layout.addWidget(n_i, i, j)


    def hideNodes(self):
        for i in range(self.dim):
            for j in range(self.n_nodes):
                spinbox = self.spinbox_layout.itemAtPosition(i, j).widget()
                if j in self.hidden_nodes:
                    sp_retain = spinbox.sizePolicy()
                    sp_retain.setRetainSizeWhenHidden(True)
                    spinbox.setSizePolicy(sp_retain)
                    spinbox.hide()
                else:
                    spinbox.show()

    def setHiddenNodes(self, node_list):
        self.hidden_nodes = node_list
        self.hideNodes()

    # def showNodes(self, node_list):
    #     self.hidden_nodes = [node for node in self.hidden_nodes if node not in node_list]
    #
    #     for i in range(self.dim):
    #         for j in range(self.n_nodes):
    #             spinbox = self.spinbox_layout.itemAtPosition(i, j).widget()
    #             if j in node_list and j not in self.disabled_nodes:
    #                 spinbox.show()

    def multipleSet(self, i):
        for child in self.findChildren(InfinitySpinBox):
            if child.isSelected():
                child.setValue(i)

    def updateValues(self, dim, node, val):
        self.values_matrix[dim, node] = val
        bounds_list = [item for sublist in self.values_matrix[:, node].tolist() for item in sublist]
        self.valueChanged.emit(node, bounds_list)  # pass only the bounds at the changed node

    @pyqtSlot("QWidget*", "QWidget*")
    def on_focusChanged(self, old, now):
        for child in self.findChildren(InfinitySpinBox):
            if child == now:
                child.select(True)
            if child == old:
                modifiers = QApplication.keyboardModifiers()
                if modifiers != Qt.ShiftModifier:
                    child.select(False)

    def getElemWidth(self):
        # simply the width of the window divided by nodes
        return self.width() / self.n_nodes

    def mousePressEvent(self, event):
        modifiers = QApplication.keyboardModifiers()
        if modifiers != Qt.ShiftModifier:
            for child in self.findChildren(InfinitySpinBox):
                child.select(False)

        self.origin = event.pos()
        self.rubberband.setGeometry(QRect(self.origin, QSize()))
        self.rubberband.show()
        QWidget.mousePressEvent(self, event)

    def mouseMoveEvent(self, event):
        if self.rubberband.isVisible():
            self.rubberband.setGeometry(QRect(self.origin, event.pos()).normalized())
        QWidget.mouseMoveEvent(self, event)

    def mouseReleaseEvent(self, event):
        if self.rubberband.isVisible():
            self.rubberband.hide()

            rect = self.rubberband.geometry()
            first = True
            for child in self.findChildren(InfinitySpinBox):

                # b_a = child.geometry()
                gp = child.mapToGlobal(QPoint(0, 0))
                b_a = self.mapFromGlobal(gp)

                # # todo check if possible, right now im subtracting a small value to make the geometry of the checkbox smaller
                if rect.intersects(QRect(b_a.x(), b_a.y(), child.geometry().width(), child.geometry().height())):
                    if first:
                        # set focus to first widget only (otherwise it does not focus on the selected items
                        first = False
                        child.setFocus(True)

                    child.select(True)

        QWidget.mouseReleaseEvent(self, event)

    # def updateLowerBoundsAll(self, dim, node, val):
    #     print('lb at node {}, dimension {}, changed: {}'.format(node, dim, val))
    #     self.lb[dim, node] = val
    #     self.lbChanged.emit(self.lb) # pass all the bounds
    #
    # def updateUpperBoundsAll(self, dim, node, val):
    #     print('ub at node {}, dimension {}, changed: {}'.format(node, dim, val))
    #     self.ub[dim, node] = val
    #     self.ubChanged.emit(self.ub)

    def getName(self):
        return self.name

if __name__ == '__main__':

    app = QApplication(sys.argv)
    # row = dim
    # column = nodes
    iv = np.ones([3, 3])
    gui = SpinboxLine('spin', nodes=5, dim=3, disabled_nodes=None)
    # gui = SpinboxLine('spin', nodes=5, dim=3, disabled_nodes=None, initial_values=iv)
    gui.setHiddenNodes([2])
    gui.setHiddenNodes([])
    # gui.showNodes([2])

    gui.setValues(iv, [2,3,4])

    gui.show()
    sys.exit(app.exec_())
