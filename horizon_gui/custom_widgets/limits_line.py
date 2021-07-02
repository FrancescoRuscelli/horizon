import numpy as np
from PyQt5.QtWidgets import QWidget, QHBoxLayout, QDoubleSpinBox, QSpinBox, QApplication, QRubberBand, QLineEdit
from PyQt5.QtCore import Qt, QRect, QSize, QPoint, pyqtSlot
from PyQt5.QtGui import QStandardItemModel

import sys

# todo
# 1. ALL THE NODES, BUT ACTIVE ONLY THE SELECTED
# 2. WHEN SELECTED, CHANGE COLOR
# 3. FIX HIGHLIGHTER
# 4. SLIDER ON SPINBOX!!!!!!!! figata

class LimitsLine(QWidget):
    def __init__(self, nodes=0, parent=None):
        super().__init__(parent)

        self.node_box_layout = QHBoxLayout(self)
        self.node_box_layout.setContentsMargins(0, 0, 0, 0)
        self.node_box_layout.setSpacing(0)

        self.rubberband = QRubberBand(QRubberBand.Rectangle, self)
        self.selected = []
        self.setNodes(nodes)

        QApplication.instance().focusChanged.connect(self.on_focusChanged)

    @pyqtSlot("QWidget*", "QWidget*")
    def on_focusChanged(self, old, now):
        for child in self.findChildren(QDoubleSpinBox):
            if child == now:
                child.setStyleSheet("background-color: Crimson;")
                self.selected.append(child)
            if child == old:
                modifiers = QApplication.keyboardModifiers()
                if modifiers != Qt.ShiftModifier:
                    child.setStyleSheet("background-color: MediumSeaGreen;")
                    # sometimes child may not exist in selected because it is removed by clicking in the widget (by mousePressEvent)
                    if child in self.selected:
                        self.selected.remove(child)

    def setNodes(self, nodes):
        self.n_nodes = nodes
        self._updateNodes()

    def getElemWidth(self):
        # simply the width of the window divided by nodes
        return self.width()/self.n_nodes

    def _updateNodes(self):

        # Add nodes to the layout
        for i in reversed(range(self.node_box_layout.count())):
            self.node_box_layout.itemAt(i).widget().deleteLater()
            self.node_box_layout.removeItem(self.node_box_layout.itemAt(i))

        for i in range(self.n_nodes):
            n_i = QDoubleSpinBox()
            n_i.valueChanged.connect(self.multipleSet)
            n_i.setStyleSheet("background-color: MediumSeaGreen;")

            self.node_box_layout.addWidget(n_i)

    def hideNodes(self, node_list):
            for i in range(self.node_box_layout.count()):
                spinbox = self.node_box_layout.itemAt(i).widget()
                if i in node_list:
                    sp_retain = spinbox.sizePolicy()
                    sp_retain.setRetainSizeWhenHidden(True)
                    spinbox.setSizePolicy(sp_retain)
                    spinbox.hide()

    def showNodes(self, node_list):
        for i in range(self.node_box_layout.count()):
            spinbox = self.node_box_layout.itemAt(i).widget()
            if i in node_list:
                spinbox.show()
                spinbox.setStyleSheet("background-color: MediumSeaGreen;")

    def mousePressEvent(self, event):
        modifiers = QApplication.keyboardModifiers()
        if modifiers != Qt.ShiftModifier:
            for item in self.selected:
                item.lineEdit().deselect()
                item.setStyleSheet("background-color: MediumSeaGreen;")
            self.selected.clear()

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
                for child in self.findChildren(QDoubleSpinBox):
                    b_a = child.geometry()
                    # gp = child.mapToGlobal(QPoint(0, 0))
                    # b_a = self.mapFromGlobal(gp)

                    # # todo check if possible, right now im subtracting a small value to make the geometry of the checkbox smaller
                    if rect.intersects(QRect(b_a.x(), b_a.y(), child.geometry().width(), child.geometry().height())):
                        self.selected.append(child)

            for item in self.selected:
                if item.isEnabled():
                    item.lineEdit().selectAll()
                    item.setStyleSheet('background-color: Crimson;')

            QWidget.mouseReleaseEvent(self, event)

    def multipleSet(self, i):
        for spinbox in self.selected:
            spinbox.setValue(i)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = LimitsLine()
    gui.setNodes(12)
    # gui.hideNodes([1,4,5,6,7])
    # gui.showNodes([1,4,5])
    # gui._showNodes([1])
    gui.show()
    sys.exit(app.exec_())
