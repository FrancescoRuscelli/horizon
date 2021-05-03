from PyQt5.QtWidgets import QWidget, QGridLayout, QRubberBand, QPushButton, QTabWidget, QHBoxLayout, QCheckBox
from PyQt5.QtCore import Qt, QRect, QSize, QPoint, pyqtSignal, pyqtSlot

from functools import partial

from custom_widgets.multi_slider import QMultiSlider

# class TabButtonWidget(QWidget):
#     def __init__(self, parent):
#         super().__init__(parent)
#         # Create button's
#         self.button_add = QPushButton("+")
#         self.button_remove = QPushButton("-")
#
#         # Set button size
#         self.button_add.setFixedSize(16, 16)
#         self.button_remove.setFixedSize(16, 16)
#
#         # Create layout
#         self.layout = QVBoxLayout()
#         self.layout.setSpacing(0)
#         self.layout.setContentsMargins(0, 0, 0, 0)
#
#         # Add button's to layout
#         self.layout.addWidget(self.button_add)
#         self.layout.addWidget(self.button_remove)
#
#         # Use layout in widget
#         self.setLayout(self.layout)
class ConstraintTab_Slider(QWidget):
    def __init__(self, name, n_nodes, parent=None):
        super().__init__(parent)

        self.ct = list()

        self.name = name
        self.n_nodes = n_nodes

        self.hlayout = QHBoxLayout(self)
        self.hlayout.setContentsMargins(0, 0, 0, 0)
        self.hlayout.setSpacing(0)

        self.slider = QMultiSlider(slider_range=[-5.0, 5.0, 0.5], values=[-4, -3], parent=self)
    # hslider.setEmitWhileMoving(True)
        self.slider.show()

class ConstraintTab_CheckBox(QWidget):
    def __init__(self, name, n_nodes, parent=None):
        super().__init__(parent)

        self.ct = list()

        self.name = name
        self.n_nodes = n_nodes

        self.hlayout = QHBoxLayout(self)
        self.hlayout.setContentsMargins(0, 0, 0, 0)
        self.hlayout.setSpacing(0)

        for node in range(self.n_nodes):
            self.ct_node = QCheckBox(self)
            self.hlayout.addWidget(self.ct_node)
            self.ct_node.stateChanged.connect(partial(self.checkConstraintNodes, node))
            self.ct.append(self.ct_node)

        # layout.addStretch()
        # self.ct.setLayout(self.ct.layout)

        # adding + - push button to tab
        # self.ct_tab.tabBar().setTabButton(0, self.ct_tab.tabBar().RightSide, TabButtonWidget())

    def getConstraint(self):
        return self.ct

    def checkConstraintNodes(self, node):
        if self.ct[node].isChecked():
            print("constraint '{}': node {} is CHECKED!".format(self.name, node))
        else:
            print("constraint '{}': node {} is CHECKED!".format(self.name, node))

class HorizonLine(QWidget):
    invalid_ct = pyqtSignal(str)

    def __init__(self, parent=None):
        QWidget.__init__(self, parent)

        self.n_nodes = 10
        self.setGeometry(QRect(210, 60, 341, 231))
        self.layout = QGridLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.layout.setSpacing(1)
        self.constraints = dict()

        self.rubberband = QRubberBand(QRubberBand.Rectangle, self)
        self.setMouseTracking(True)


        # Add widgets to the layout
        for i in range(self.n_nodes):
            n_i = QPushButton("{}".format(i), self)
            n_i.setFixedSize(20, 20)
            self.layout.addWidget(n_i, 0, i, 1, 1)

        self.ct_tab = QTabWidget(self)
        self.ct_tab.setGeometry(200, 100, 300, 150)
        self.layout.addWidget(self.ct_tab, 1, 0, 1, self.n_nodes)

    @pyqtSlot()
    def on_invalid_ct(self, str):
        self.invalid_ct.emit(str)

    def addConstraintToHorizon(self, name):
        # check if constraint is already in the archive
        if name in self.constraints:
            self.on_invalid_ct("constraint already inserted")
            return
        elif name == "":
            self.on_invalid_ct("empty name of constraint not allowed")
            return

        self.ct = ConstraintTab_Slider(name, self.n_nodes)
        self.ct_tab.addTab(self.ct, str(name))
        self.constraints[name] = self.ct.getConstraint()

    def getConstraint(self, name):
        return self.constraints[name]

    def mousePressEvent(self, event):
        self.origin = event.pos()
        self.rubberband.setGeometry(QRect(self.origin, QSize()))
        self.rubberband.show()
        QWidget.mousePressEvent(self, event)

    def mouseMoveEvent(self, event):
        if self.rubberband.isVisible():
            self.rubberband.setGeometry(QRect(self.origin, event.pos()).normalized())
        QWidget.mouseMoveEvent(self, event)
    #
    def mouseReleaseEvent(self, event):
            if self.rubberband.isVisible():
                self.rubberband.hide()
                selected = []
                rect = self.rubberband.geometry()
                if self.ct_tab.widget(self.ct_tab.currentIndex()) is not None:
                    for child in self.ct_tab.widget(self.ct_tab.currentIndex()).findChildren(QCheckBox):
                        # get absolute geometry and then map it to self (HorizonLine)
                        # the problem was that .geometry() returns the position w.r.t. the parent:
                        # I was comparing the geometry of the rubberband taken in the HorizonLine's coordinates with
                        # the geometry of the QCheckBox taken in the QGridLayout's coordinates.

                        gp = child.mapToGlobal(QPoint(0, 0))
                        b_a = self.mapFromGlobal(gp)

                        # todo check if possible, right now im subtracting a small value to make the geometry of the checkbox smaller
                        if rect.intersects(QRect(b_a.x(), b_a.y(), child.geometry().width()-20, child.geometry().height())):
                            selected.append(child)

                for elem in selected:
                    if event.button() == Qt.RightButton:
                        elem.setChecked(False)
                    elif event.button() == Qt.LeftButton:
                        elem.setChecked(True)

            QWidget.mouseReleaseEvent(self, event)