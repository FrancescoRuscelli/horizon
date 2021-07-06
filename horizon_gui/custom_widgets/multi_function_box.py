from PyQt5.QtWidgets import QWidget, QGridLayout, QPushButton, QLabel, QApplication, QStyle, QScrollArea, QSpacerItem, QSizePolicy
from PyQt5.QtGui import QDrag, QPalette
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QMimeData

from horizon_gui.custom_widgets.function_line import FunctionLine
import sys
from functools import partial

class MultiFunctionBox(QScrollArea):
    funNodesChanged = pyqtSignal(str, list)
    funCloseRequested = pyqtSignal(str)

    def __init__(self, n_nodes, options=None, parent=None):
        super().__init__(parent)

        self.height_bar = 80
        self.distance_between_bars = 10
        self.main_widget = QWidget()
        self.setBackgroundRole(QPalette.Dark)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setWidgetResizable(True)

        self.setWidget(self.main_widget)

        self.n_nodes = n_nodes
        self.options = options
        self.target = None

        self.main_layout = QGridLayout(self.main_widget)
        self.main_layout.setVerticalSpacing(self.distance_between_bars)
        self.fun_list = dict()
        self.close_buttons = dict()
        self.titles = dict()

    def getNFunctions(self):
        return len(self.fun_list)

    def addFunctionToGUI(self, fun_name):

        # removing the QSpacerItem, if present, before adding a new multiline widget
        for i in range(self.main_layout.count()):
            item = self.main_layout.itemAt(i)
            if item and isinstance(item, QSpacerItem):
                self.main_layout.takeAt(i)

        self.function_line = QWidget()
        self.function_line.setStyleSheet('background-color: orange;')
        self.function_line.setMaximumHeight(self.height_bar)
        self.function_line.setMinimumHeight(self.height_bar)
        self.function_line.setObjectName(fun_name)

        self.titles[fun_name] = QLabel(fun_name)
        self.titles[fun_name].setStyleSheet("font:Bold")
        self.titles[fun_name].setStyleSheet('background-color: green;')

        self.close_buttons[fun_name] = QPushButton()
        self.close_buttons[fun_name].setIcon(self.style().standardIcon(QStyle.SP_MessageBoxCritical))  # icon
        self.close_buttons[fun_name].setStyleSheet('background-color: blue;')

        self.fun_list[fun_name] = FunctionLine(fun_name, self.n_nodes, options=self.options)
        self.fun_list[fun_name].nodesChanged.connect(self.emitFunctionNodes)
        self.fun_list[fun_name].setStyleSheet('background-color: red;')

        self.row_layout = QGridLayout()
        self.row_layout.addWidget(self.titles[fun_name], 0, 0, Qt.AlignRight)
        self.row_layout.addWidget(self.fun_list[fun_name], 1, 0, 1, 2)
        self.row_layout.addWidget(self.close_buttons[fun_name], 0, 1, Qt.AlignLeft)
        self.row_layout.setSpacing(0)
        self.row_layout.setContentsMargins(0, 0, 0, 0)
        self.function_line.setLayout(self.row_layout)

        self.main_layout.addWidget(self.function_line, len(self.fun_list), 0, Qt.AlignTop)
        # adding a stretch to make all the widget piling from above
        qspacer = QSpacerItem(0, 0, QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.main_layout.addItem(qspacer)

        self.close_buttons[fun_name].clicked.connect(partial(self.on_fun_close_requested, fun_name))

    @pyqtSlot()
    def on_fun_close_requested(self, fun_name):
        self.funCloseRequested.emit(fun_name)

    def removeFunctionFromGUI(self, fun_name):
        del self.fun_list[fun_name]
        for i in range(self.main_layout.count()):
            current_widget = self.main_layout.itemAt(i).widget()
            if current_widget:
                if fun_name == current_widget.objectName():
                    current_widget.deleteLater()


    def emitFunctionNodes(self, name, ranges):
        self.on_fun_nodes_changed(name, ranges)

    @pyqtSlot()
    def on_fun_nodes_changed(self, fun_name, ranges):
        self.funNodesChanged.emit(fun_name, ranges)

    def setHorizonNodes(self, nodes):
        self.n_nodes = nodes

        for item_fl in self.findChildren(FunctionLine):
            item_fl.updateHorizonNodes(nodes)

    def setFunctionNodes(self, fun_name, ranges):
        for item_fl in self.findChildren(FunctionLine):
            if item_fl.getName() == fun_name:
                item_fl.updateSlices(ranges)

    def updateMargins(self, margins):
        for i in range(self.main_layout.count()):
            self.main_layout.setContentsMargins(margins)

    # def get_index(self, pos):
    #     for i in range(self.main_layout.count()):
    #         if self.main_layout.itemAt(i).geometry().contains(pos) and i != self.target:
    #             return i
    #
    # def mousePressEvent(self, event):
    #     if event.button() == Qt.LeftButton:
    #         self.target = self.get_index(event.windowPos().toPoint())
    #         print(self.target)
    #     else:
    #         self.target = None
    #
    # def mouseMoveEvent(self, event):
    #     if event.buttons() & Qt.LeftButton and self.target is not None:
    #         drag = QDrag(self.main_layout.itemAt(self.target))
    #         pix = self.main_layout.itemAt(self.target).itemAt(0).widget().grab()
    #         mimedata = QMimeData()
    #         mimedata.setImageData(pix)
    #         drag.setMimeData(mimedata)
    #         drag.setPixmap(pix)
    #         drag.setHotSpot(event.pos())
    #         drag.exec_()
    #
    # def mouseReleaseEvent(self, event):
    #     self.target = None
    #
    # def dragEnterEvent(self, event):
    #     if event.mimeData().hasImage():
    #         event.accept()
    #     else:
    #         event.ignore()
    #
    # def dropEvent(self, event):
    #     if not event.source().geometry().contains(event.pos()):
    #         source = self.get_index(event.pos())
    #         if source is None:
    #             return
    #
    #         i, j = max(self.target, source), min(self.target, source)
    #         p1, p2 = self.gridLayout.getItemPosition(i), self.gridLayout.getItemPosition(j)
    #
    #         self.gridLayout.addItem(self.gridLayout.takeAt(i), *p2)
    #         self.gridLayout.addItem(self.gridLayout.takeAt(j), *p1)

if __name__ == '__main__':


    app = QApplication(sys.argv)
    gui = MultiFunctionBox(10)
    gui.addFunctionToGUI('daniele')
    gui.addFunctionToGUI('pino')
    gui.addFunctionToGUI('sarcopodo')


    gui.show()
    sys.exit(app.exec_())



