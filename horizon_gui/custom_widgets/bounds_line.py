import numpy as np
from PyQt5.QtWidgets import QWidget, QGridLayout, QVBoxLayout, QPushButton, QApplication, QRubberBand, QLabel
from PyQt5.QtGui import QPalette, QFont
from PyQt5.QtCore import Qt, QRect, QSize, QPoint, pyqtSlot, pyqtSignal
from horizon_gui.custom_widgets.infinity_spinbox import InfinitySpinBox
from functools import partial

import sys

# todo
# 1. ALL THE NODES, BUT ACTIVE ONLY THE SELECTED ~OK
# 2. WHEN SELECTED, CHANGE COLOR ~OK
# 3. FIX HIGHLIGHTER ~OK

# 4. SLIDER ON SPINBOX!!!!!!!! figata
# 5. ONE FOR EACH DIMENSION OF VARIABLE

class BoundsLine(QWidget):
    # lbChangedAll = pyqtSignal(np.matrix)
    # ubChangedAll = pyqtSignal(np.matrix)
    lbChanged = pyqtSignal(int, list)
    ubChanged = pyqtSignal(int, list)

    def __init__(self, name, nodes=0, dim=1, initial_bounds=None, parent=None):
        super().__init__(parent)

        self.name = name

        if initial_bounds is None:
            self.lb = np.matrix(np.ones((dim, nodes)) * 0.)
            self.ub = np.matrix(np.ones((dim, nodes)) * 0.)
        else:
            self.lb = initial_bounds[0]
            self.ub = initial_bounds[1]

        self.min_color_base = "MediumSeaGreen"
        self.min_color_selected = "Crimson"
        self.max_color_base = "turquoise"
        self.max_color_selected = "Crimson"

        self.setStyleSheet("background-color: rgb(255,0,0); border:1px solid rgb(0, 255, 0); ")
        self.n_nodes = nodes
        self.dim = dim

        self.layouts = []
        self.main_layout = QVBoxLayout(self)
        self.main_layout.setContentsMargins(0, 0, 0, 0)
        self.main_layout.setSpacing(0)

        self.ub_widget = QWidget()
        self.ub_layout = QGridLayout(self.ub_widget)
        self.ub_layout.setContentsMargins(0, 20, 0, 20)
        self.ub_layout.setSpacing(0)

        self.lb_widget = QWidget()
        self.lb_layout = QGridLayout(self.lb_widget)
        self.lb_layout.setContentsMargins(0, 20, 0, 20)

        palette = QPalette()
        palette.setColor(QPalette.Text, Qt.red)

        font = QFont("Times", 12, QFont.Bold)

        title_min = QLabel('MIN')
        title_min.setAlignment(Qt.AlignCenter)
        title_min.setPalette(palette)
        title_min.setFont(font)

        title_max = QLabel('MAX')
        title_max.setAlignment(Qt.AlignCenter)
        title_max.setPalette(palette)
        title_max.setFont(font)

        self.main_layout.addWidget(title_min)
        self.main_layout.addWidget(self.lb_widget)
        self.main_layout.addWidget(title_max)
        self.main_layout.addWidget(self.ub_widget)

        self.bounds_widgets = [self.lb_widget, self.ub_widget]

        self.rubberband = QRubberBand(QRubberBand.Rectangle, self)
        self.selected = []
        self.setNodes(nodes)

        QApplication.instance().focusChanged.connect(self.on_focusChanged)

    def setNodes(self, nodes):
        self.n_nodes = nodes
        self._updateNodes()

    def getElemWidth(self):
        # simply the width of the window divided by nodes
        return self.width()/self.n_nodes

    def _updateNodes(self):

        for item in self.bounds_widgets:
            layout = item.layout()
            # Remove all the nodes
            for i in reversed(range(layout.count())):
                layout.itemAt(i).widget().deleteLater()
                layout.removeItem(layout.itemAt(i))

        temp_lb = np.matrix(np.ones((self.dim, self.n_nodes)) * -np.inf)
        temp_ub = np.matrix(np.ones((self.dim, self.n_nodes)) * np.inf)

        temp_lb[:self.lb.shape[0], :self.lb.shape[1]] = self.lb[:temp_lb.shape[0], :temp_lb.shape[1]]
        temp_ub[:self.ub.shape[0], :self.ub.shape[1]] = self.ub[:temp_ub.shape[0], :temp_ub.shape[1]]
        self.lb = temp_lb
        self.ub = temp_ub
        # self.lb = np.resize(self.lb, (self.dim, self.n_nodes))
        # self.ub = np.resize(self.ub, (self.dim, self.n_nodes))

        self.createCustomSpinboxLine(self.bounds_widgets[0].layout(), updatefun=self.updateLowerBounds, values=self.lb, color_base="MediumSeaGreen", color_selected="Crimson")
        self.createCustomSpinboxLine(self.bounds_widgets[1].layout(), updatefun=self.updateUpperBounds, values=self.ub, color_base="turquoise", color_selected="Crimson")


    def createCustomSpinboxLine(self, parent, updatefun, values, color_base=None, color_selected=None):



        for i in range(self.dim):
            for j in range(self.n_nodes):
                n_i = InfinitySpinBox(color_base=color_base, color_selected=color_selected)
                n_i.setValue(values[i,j])
                n_i.valueChanged.connect(self.multipleSet)
                n_i.valueChanged.connect(partial(updatefun, i, j))
                parent.addWidget(n_i, i, j)


    def hideNodes(self, node_list):
        for item in self.bounds_widgets:
            layout = item.layout()
            for i in range(self.dim):
                for j in range(self.n_nodes):
                    spinbox = layout.itemAtPosition(i, j).widget()
                    if j in node_list:
                        sp_retain = spinbox.sizePolicy()
                        sp_retain.setRetainSizeWhenHidden(True)
                        spinbox.setSizePolicy(sp_retain)
                        spinbox.hide()

    def showNodes(self, node_list):
        for item in self.bounds_widgets:
            layout = item.layout()
            for i in range(self.dim):
                for j in range(self.n_nodes):
                    spinbox = layout.itemAtPosition(i, j).widget()
                    if j in node_list:
                        spinbox.show()

    def mousePressEvent(self, event):
        modifiers = QApplication.keyboardModifiers()
        if modifiers != Qt.ShiftModifier:
            for child in self.findChildren(InfinitySpinBox):
                child.select(False)

        self.origin = event.pos()
        self.rubberband.setGeometry(QRect(self.origin, QSize()))
        self.rubberband.show()
        QWidget.mousePressEvent(self, event)
    #
    def mouseMoveEvent(self, event):
        if self.rubberband.isVisible():
            self.rubberband.setGeometry(QRect(self.origin, event.pos()).normalized())
        QWidget.mouseMoveEvent(self, event)
    #
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

    def multipleSet(self, i):
        for child in self.findChildren(InfinitySpinBox):
            if child.isSelected():
                child.setValue(i)

    # def updateLowerBoundsAll(self, dim, node, val):
    #     print('lb at node {}, dimension {}, changed: {}'.format(node, dim, val))
    #     self.lb[dim, node] = val
    #     self.lbChanged.emit(self.lb) # pass all the bounds
    #
    # def updateUpperBoundsAll(self, dim, node, val):
    #     print('ub at node {}, dimension {}, changed: {}'.format(node, dim, val))
    #     self.ub[dim, node] = val
    #     self.ubChanged.emit(self.ub)

    def updateLowerBounds(self, dim, node, val):
        self.lb[dim, node] = val
        bounds_list = [item for sublist in self.lb[:, node].tolist() for item in sublist]
        self.lbChanged.emit(node, bounds_list) # pass only the bounds at the changed node

    def updateUpperBounds(self, dim, node, val):
        self.ub[dim, node] = val
        bounds_list = [item for sublist in self.ub[:, node].tolist() for item in sublist]
        self.ubChanged.emit(node, bounds_list)

    @pyqtSlot("QWidget*", "QWidget*")
    def on_focusChanged(self, old, now):
        for child in self.findChildren(InfinitySpinBox):
            if child == now:
                child.select(True)
            if child == old:
                modifiers = QApplication.keyboardModifiers()
                if modifiers != Qt.ShiftModifier:
                    child.select(False)

    def getName(self):
        return self.name

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = BoundsLine('daniele', nodes=1, dim=3)
    gui.setNodes(10)
    gui.hideNodes([0,1,2,4,5,6,7,8])
    # gui.showNodes([1,4,5])

    pushbutton = QPushButton('daniele')
    pushbutton.clicked.connect(partial(gui.setNodes, 5))
    pushbutton.show()
    gui.show()
    sys.exit(app.exec_())
