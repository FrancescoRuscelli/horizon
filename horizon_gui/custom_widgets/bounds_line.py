import numpy as np
from PyQt5.QtWidgets import QWidget, QGridLayout, QVBoxLayout, QPushButton, QApplication, QRubberBand, QLabel
from PyQt5.QtGui import QPalette, QFont
from PyQt5.QtCore import Qt, QRect, QSize, QPoint, pyqtSlot, pyqtSignal
from horizon_gui.custom_widgets.spinbox_line import SpinboxLine
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

    def __init__(self, name, nodes=0, dim=1, disabled_nodes=None, initial_bounds=None, parent=None):

        super().__init__(parent)

        self.name = name
        self.main_layout = QVBoxLayout(self)
        self.main_layout.setSpacing(0)
        self.nodes = nodes
        self.dim = dim
        self.disabled_nodes = disabled_nodes
        self.lines = []

        if initial_bounds is None:
            self.addLine('Lower Bounds', self.emitLowerBounds)
            self.addLine('Upper Bounds', self.emitUpperBounds)
        else:
            self.addLine('Lower Bounds', self.emitLowerBounds, initial_bounds[0])
            self.addLine('Upper Bounds', self.emitUpperBounds, initial_bounds[1])

    def addLine(self, title, emitter, initial_values=None):

        line = SpinboxLine(title, self.nodes, self.dim, self.disabled_nodes, initial_values)
        line.valueChanged.connect(emitter)
        self.main_layout.addWidget(line)
        self.lines.append(line)

    def setNNodes(self, nodes):
        self.n_nodes = nodes
        for line in self.lines:
            line.setNodes(self.n_nodes)

    def hideNodes(self, nodes_list):
        for line in self.lines:
            line.hideNodes(nodes_list)

    def showNodes(self, nodes_list):
        for line in self.lines:
            line.showNodes(nodes_list)

    def emitLowerBounds(self, node, bounds_list):
        self.lbChanged.emit(node, bounds_list)  # pass only the bounds at the changed node
        print(node, bounds_list)

    def emitUpperBounds(self, node, bounds_list):
        self.ubChanged.emit(node, bounds_list)  # pass only the bounds at the changed node
        print(node, bounds_list)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = BoundsLine('daniele', nodes=1, dim=3)
    gui.setNNodes(10)
    gui.hideNodes([0,1,2,4,5,6,7,8])
    # gui.showNodes([1,4,5])

    pushbutton = QPushButton('daniele')
    pushbutton.clicked.connect(partial(gui.setNNodes, 5))
    pushbutton.show()
    gui.show()
    sys.exit(app.exec_())
