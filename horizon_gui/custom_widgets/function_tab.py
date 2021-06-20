from PyQt5.QtWidgets import QWidget, QTabWidget, QHBoxLayout, QVBoxLayout
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot

from horizon_gui.definitions import CSS_DIR
from horizon_gui.custom_widgets.multi_slider import QMultiSlider


class FunctionLine(QWidget):
    nodesChanged = pyqtSignal(str, list)

    def __init__(self, name, n_nodes, options=None, parent=None):
        super().__init__(parent)

        self.name = name
        self.n_nodes = n_nodes

        self.hlayout = QHBoxLayout(self)
        self.slider = QMultiSlider(slider_range=[0, self.n_nodes - 1, 1], values=[0, self.n_nodes-1], options=options)
        self.hlayout.addWidget(self.slider)
        self.hlayout.setContentsMargins(0, 0, 0, 0)
        self.slider.slicesChanged.connect(self.on_nodes_changed)

    def on_nodes_changed(self, range_list):
        self.nodesChanged.emit(self.name, range_list)
        # adding + - push button to tab
        # self.ct_tab.tabBar().setTabButton(0, self.ct_tab.tabBar().RightSide, TabButtonWidget())

    def updateRange(self, n_nodes):
        self.n_nodes = n_nodes
        self.slider.updateRange([0, self.n_nodes-1])


class FunctionTabWidget(QTabWidget):
    funNodesChanged = pyqtSignal(str, list)

    def __init__(self, n_nodes, options=None, parent=None):
        super().__init__(parent)

        self.n_nodes = n_nodes
        self.options = options

        with open(CSS_DIR + '/tab.css', 'r') as f:
            self.setStyleSheet(f.read())

        self.setTabPosition(1)
        self.setTabsClosable(True)
        self.setAttribute(Qt.WA_StyledBackground, True)


    def addFunctionToGUI(self, fun_name):

        self.ft = FunctionLine(fun_name, self.n_nodes, options=self.options)
        self.ft.nodesChanged.connect(self.emitFunctionNodes)


        # todo hardcoded bottom margin
        self.intab_layout = QVBoxLayout()

        self.intab_layout.addWidget(self.ft)
        self.intab_layout.addSpacing(120)


        self.tab = QWidget()
        self.tab.setStyleSheet('background-color: blue;')
        self.tab.setLayout(self.intab_layout)
        self.addTab(self.tab, str(fun_name))


    @pyqtSlot()
    def on_fun_nodes_changed(self, fun_name, ranges):
        self.funNodesChanged.emit(fun_name, ranges)

    def emitFunctionNodes(self, name, ranges):
        self.on_fun_nodes_changed(name, ranges)

    def getFunctionNodes(self, fun_name):
        for i in range(self.count()):
            if self.tabText(i) == fun_name:
                print('Nodes of function {}: {}'.format(fun_name, self.widget(i).getNodes()))
                return self.widget(i).getNodes()
            else:
                pass

    def updateMargins(self, margins):
        for i in range(self.count()):
            self.intab_layout.setContentsMargins(margins)

    def setNodes(self, nodes):
        self.n_nodes = nodes
        for i in range(self.count()):
            self.widget(i).findChild(FunctionLine).updateRange(self.n_nodes)


