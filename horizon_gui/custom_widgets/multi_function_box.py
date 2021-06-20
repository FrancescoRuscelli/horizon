from PyQt5.QtWidgets import QWidget, QGridLayout, QPushButton, QLabel, QApplication, QStyle

from horizon_gui.custom_widgets import function_tab
from PyQt5.QtCore import pyqtSignal, pyqtSlot
import sys
from functools import partial

class MultiFunctionBox(QWidget):
    funNodesChanged = pyqtSignal(str, list)
    funCloseRequested = pyqtSignal(str)

    def __init__(self, n_nodes, options=None, parent=None):
        super().__init__(parent)

        self.n_nodes = n_nodes
        self.options = options

        self.main_layout = QGridLayout(self)
        self.fun_list = dict()
        self.close_buttons = dict()
        self.titles = dict()


    def addFunctionToGUI(self, fun_name):


        self.titles[fun_name] = QLabel(fun_name)
        self.titles[fun_name].setStyleSheet("font:Bold")

        self.close_buttons[fun_name] = QPushButton()
        self.close_buttons[fun_name].setIcon(self.style().standardIcon(QStyle.SP_MessageBoxCritical))  # icon

        self.fun_list[fun_name] = function_tab.FunctionLine(fun_name, self.n_nodes, options=self.options)
        self.fun_list[fun_name].nodesChanged.connect(self.emitFunctionNodes)

        self.main_layout.addWidget(self.titles[fun_name], len(self.fun_list), 0)
        self.main_layout.addWidget(self.fun_list[fun_name], len(self.fun_list), 1)
        self.main_layout.addWidget(self.close_buttons[fun_name], len(self.fun_list), 2)
        self.main_layout.setColumnStretch(1, 4)

        self.close_buttons[fun_name].clicked.connect(partial(self.removeFunctionFromGUI, fun_name))

        # self.fun_list[fun_name].nodesChanged.connect(self.emitFunctionNodes)

    def removeFunctionFromGUI(self, fun_name):

        for name, label in self.titles.items():
            if fun_name == name:
                index = self.main_layout.indexOf(label)

                row = self.main_layout.getItemPosition(index)[0]
                for column in range(self.main_layout.columnCount()):
                    layout = self.main_layout.itemAtPosition(row, column)
                    if layout is not None:
                        layout.widget().deleteLater()
                        self.main_layout.removeItem(layout)

        # self.main_layout.removeWidget(self.titles[fun_name])
        # self.main_layout.removeWidget(self.close_buttons[fun_name])
        # self.main_layout.removeWidget(self.fun_list[fun_name])
        #
        # self.titles[fun_name].deleteLater()
        # self.close_buttons[fun_name].deleteLater()
        # self.fun_list[fun_name].deleteLater()
        #
        # del self.titles[fun_name]
        # del self.close_buttons[fun_name]
        # del self.fun_list[fun_name]



    def emitFunctionNodes(self, name, ranges):
        self.on_fun_nodes_changed(name, ranges)
        #todo

    @pyqtSlot()
    def on_fun_nodes_changed(self, fun_name, ranges):
        self.funNodesChanged.emit(fun_name, ranges)


if __name__ == '__main__':


    app = QApplication(sys.argv)
    gui = MultiFunctionBox(10)
    gui.addFunction('daniele')
    gui.addFunction('pino')
    gui.addFunction('sarcopodo')


    gui.show()
    sys.exit(app.exec_())



