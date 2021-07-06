import numpy as np
from PyQt5.QtWidgets import QWidget, QGridLayout, QVBoxLayout, QSpinBox, QApplication, QRubberBand, QLabel
from PyQt5.QtGui import QPalette, QFont
from PyQt5.QtCore import Qt, QRect, QSize, QPoint, pyqtSlot
from horizon_gui.custom_widgets.infinity_spinbox import InfinitySpinBox

import sys

# todo
# 1. ALL THE NODES, BUT ACTIVE ONLY THE SELECTED ~OK
# 2. WHEN SELECTED, CHANGE COLOR ~OK
# 3. FIX HIGHLIGHTER ~OK

# 4. SLIDER ON SPINBOX!!!!!!!! figata
# 5. ONE FOR EACH DIMENSION OF VARIABLE

class LimitsLine(QWidget):
    def __init__(self, nodes=0, dim=1, parent=None):
        super().__init__(parent)

        self.min_color_base = "MediumSeaGreen"
        self.min_color_selected = "Crimson"
        self.max_color_base = "turquoise"
        self.max_color_selected = "Crimson"

        self.setStyleSheet("background-color: rgb(255,0,0); margin:5px; border:1px solid rgb(0, 255, 0); ")
        self.n_nodes = nodes
        self.dim = dim

        self.layouts = []
        self.main_layout = QVBoxLayout(self)
        self.main_layout.setSpacing(0)

        self.max_lims = QWidget()
        self.max_lims_layout = QGridLayout(self.max_lims)
        self.max_lims_layout.setContentsMargins(0, 20, 0, 20)
        self.max_lims_layout.setSpacing(0)

        self.min_lims = QWidget()
        self.min_lims_layout = QGridLayout(self.min_lims)
        self.min_lims_layout.setContentsMargins(0, 20, 0, 20)

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
        self.main_layout.addWidget(self.min_lims)
        self.main_layout.addWidget(title_max)
        self.main_layout.addWidget(self.max_lims)

        self.lims = [self.min_lims, self.max_lims]

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

        for item in self.lims:
            layout = item.layout()
            # Remove all the nodes
            for i in reversed(range(layout.count())):
                layout.itemAt(i).widget().deleteLater()
                layout.removeItem(layout.itemAt(i))

        self.createCustomSpinboxLine(self.lims[0].layout(), value=-np.inf, color_base="MediumSeaGreen", color_selected="Crimson")
        self.createCustomSpinboxLine(self.lims[1].layout(), value=np.inf, color_base="turquoise", color_selected="Crimson")


    def createCustomSpinboxLine(self, parent, value=0, color_base=None, color_selected=None):
        for i in range(self.dim):
            for j in range(self.n_nodes):
                n_i = InfinitySpinBox(color_base=color_base, color_selected=color_selected)
                n_i.setValue(value)
                n_i.valueChanged.connect(self.multipleSet)
                parent.addWidget(n_i, i, j)

    def hideNodes(self, node_list):
        for item in self.lims:
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
        for item in self.lims:
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

    @pyqtSlot("QWidget*", "QWidget*")
    def on_focusChanged(self, old, now):
        for child in self.findChildren(InfinitySpinBox):
            if child == now:
                child.select(True)
            if child == old:
                modifiers = QApplication.keyboardModifiers()
                if modifiers != Qt.ShiftModifier:
                    child.select(False)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = LimitsLine(dim=3)
    gui.setNodes(12)
    gui.hideNodes([1,4,5,6,7])
    # gui.showNodes([1,4,5])
    gui.show()
    sys.exit(app.exec_())
