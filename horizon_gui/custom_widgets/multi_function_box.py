from PyQt5.QtWidgets import QWidget, QGridLayout, QHBoxLayout, QVBoxLayout, QPushButton, QLabel, QApplication, QStyle, QScrollArea
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

        self.height_bar = 100
        self.distance_between_bars = 200
        self.main_widget = QWidget()
        # self.setBackgroundRole(QPalette.Dark)
        # self.setFixedHeight(100)
        self.setWidgetResizable(True)
        self.setWidget(self.main_widget)

        self.n_nodes = n_nodes
        self.options = options
        self.target = None

        self.main_layout = QGridLayout(self.main_widget)
        self.main_layout.setVerticalSpacing(200)
        self.fun_list = dict()
        self.close_buttons = dict()
        self.titles = dict()


    def addFunctionToGUI(self, fun_name):

        self.function_line = QWidget()
        self.function_line.setMaximumHeight(self.height_bar)
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
        self.row_layout.addWidget(self.titles[fun_name], 0, 0)
        self.row_layout.addWidget(self.fun_list[fun_name], 1, 0)
        self.row_layout.addWidget(self.close_buttons[fun_name], 0, 1)
        # self.row_layout.setStretch(1, 4)

        self.function_line.setLayout(self.row_layout)

        self.main_layout.addWidget(self.function_line, len(self.fun_list), 0, Qt.AlignTop)
        self.close_buttons[fun_name].clicked.connect(partial(self.on_fun_close_requested, fun_name))

    @pyqtSlot()
    def on_fun_close_requested(self, fun_name):
        self.funCloseRequested.emit(fun_name)

    def removeFunctionFromGUI(self, fun_name):

        for i in range(self.main_layout.count()):
            current_widget = self.main_layout.itemAt(i)
            if fun_name == current_widget.widget().objectName():
                current_widget.widget().deleteLater()

    def emitFunctionNodes(self, name, ranges):
        self.on_fun_nodes_changed(name, ranges)

    @pyqtSlot()
    def on_fun_nodes_changed(self, fun_name, ranges):
        self.funNodesChanged.emit(fun_name, ranges)

    def setHorizonNodes(self, nodes):
        self.n_nodes = nodes
        for i in range(self.main_layout.count()):
            if isinstance(self.main_layout.itemAt(i).widget(), FunctionLine):
                self.main_layout.itemAt(i).widget().updateHorizonNodes(nodes)

    def setFunctionNodes(self, fun_name, ranges):
        for i in range(self.main_layout.count()):
            if isinstance(self.main_layout.itemAt(i).widget(), FunctionLine) and self.main_layout.itemAt(i).widget().getName() == fun_name:
                self.main_layout.itemAt(i).widget().updateSlices(ranges)

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



