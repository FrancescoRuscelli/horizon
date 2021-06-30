from PyQt5.QtWidgets import QWidget, QHBoxLayout, QSpinBox, QApplication, QRubberBand, QLineEdit
from PyQt5.QtCore import QRect, QSize, QPoint
from PyQt5.QtGui import QStandardItemModel

import sys

# todo
# 1. ALL THE NODES, BUT ACTIVE ONLY THE SELECTED
# 2. WHEN SELECTED, CHANGE COLOR
# 3. FIX HIGHLIGHTER

class LimitsLine(QWidget):
    def __init__(self, nodes=0, parent=None):
        super().__init__(parent)

        self.node_box_layout = QHBoxLayout(self)
        margins = self.node_box_layout.contentsMargins()
        margins.setBottom(0)
        self.node_box_layout.setContentsMargins(margins)
        self.node_box_layout.setSpacing(0)

        self.rubberband = QRubberBand(QRubberBand.Rectangle, self)
        self.selected = []
        self.setNodes(nodes)

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
            n_i = QSpinBox()
            n_i.setStyleSheet('background-color: MediumSeaGreen;')
            self.node_box_layout.addWidget(n_i)

    def mousePressEvent(self, event):

        for item in self.selected:
            item.lineEdit().deselect()

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
                for child in self.findChildren(QSpinBox):
                #         # get absolute geometry and then map it to self (HorizonLine)
                #         # the problem was that .geometry() returns the position w.r.t. the parent:
                #         # I was comparing the geometry of the rubberband taken in the HorizonLine's coordinates with
                #         # the geometry of the QCheckBox taken in the QGridLayout's coordinates.
                    b_a = child.geometry()
                    # gp = child.mapToGlobal(QPoint(0, 0))
                    # b_a = self.mapFromGlobal(gp)

                    # # todo check if possible, right now im subtracting a small value to make the geometry of the checkbox smaller
                    if rect.intersects(QRect(b_a.x(), b_a.y(), child.geometry().width(), child.geometry().height())):
                        self.selected.append(child)

            for spinbox_selected in self.selected:
                spinbox_selected.lineEdit().selectAll()
                # spinbox_selected.selectAll()

            if self.selected:
                self.selected[0].setFocus()
                self.selected[0].valueChanged.connect(self.multipleSet)

            QWidget.mouseReleaseEvent(self, event)

    def multipleSet(self, i):
        for spinbox in self.selected:
            spinbox.setValue(i)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = LimitsLine()
    gui.setNodes(4)
    gui.show()
    sys.exit(app.exec_())
