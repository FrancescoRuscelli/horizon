from PyQt5.QtWidgets import QWidget, QStackedWidget, QVBoxLayout, QScrollArea
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot, QModelIndex, QMargins
from PyQt5.QtGui import QStandardItemModel

from horizon_gui.custom_widgets.node_box_line import NodeBoxLine
from horizon_gui.custom_widgets.function_tab import FunctionTabWidget
from horizon_gui.custom_widgets.multi_function_box import MultiFunctionBox

from horizon.misc_function import listOfListFLOATtoINT, unravelElements
from functools import partial

# horizon line
#     | | | |____ horizon
#     | | |
#     | | |______ nodes box line
#     | |
#     | |________ function tab
#     |                 |  | ______ function_line
#     |                 |__________ limits_line
#     |
#     |__________ multi_function_box


class HorizonLine(QScrollArea):
    add_fun_horizon = pyqtSignal(dict)
    remove_fun_horizon = pyqtSignal(dict)
    repeated_fun = pyqtSignal(str)

    def __init__(self, horizon, fun_type, nodes=0, logger=None, parent=None):
        super().__init__(parent)

        self.setAcceptDrops(True)
        self.main_widget = QWidget()
        self.setWidgetResizable(True)
        self.setWidget(self.main_widget)

        self.bounds_flag = True
        self.horizon_receiver = horizon
        self.fun_type = fun_type  # can be Constraint or Cost Function
        if fun_type == 'costfunction':
            self.bounds_flag = False
        self.n_nodes = nodes

        if logger:
            self.logger = logger

        self.main_layout = QVBoxLayout(self.main_widget)
        self.main_layout.setSpacing(0)

        self.nodes_line = NodeBoxLine(self.n_nodes)
        self.nodes_line.setAttribute(Qt.WA_StyledBackground, True)
        self.nodes_line.setStyleSheet('background-color: green;')
        self.main_layout.addWidget(self.nodes_line)

        self.stacked_lines = QStackedWidget()
        self.main_layout.addWidget(self.stacked_lines)

        self.Multi = 0
        self.Single = 1

        self._initOptions()
        self._initMultiLine()
        self._initSingleLine()

    def _initSingleLine(self):

        self.function_tab = FunctionTabWidget(self.n_nodes, options=self.options, bounds_flag=self.bounds_flag)
        self.stacked_lines.addWidget(self.function_tab)

        self.function_tab.tabCloseRequested.connect(self.removeActiveFunctionRequestFromIndex)
        self.function_tab.funNodesChanged.connect(partial(self.updateFunctionNodes, 'single'))
        self.function_tab.funLbChanged.connect(self.updateFunctionLb)
        self.function_tab.funUbChanged.connect(self.updateFunctionUb)


    def _initMultiLine(self):

        self.multi_function_box = MultiFunctionBox(self.n_nodes, options=self.options)
        self.stacked_lines.addWidget(self.multi_function_box)

        self.multi_function_box.funCloseRequested.connect(self.removeActiveFunctionRequestFromName)
        self.multi_function_box.funNodesChanged.connect(partial(self.updateFunctionNodes, 'multi'))

    def _initOptions(self):

        self.options = dict()
        if self.fun_type == 'constraint':
            self.options['background_color'] = Qt.darkGreen
            self.options['slice_color'] = Qt.green
            self.options['minmax_color'] = Qt.darkRed
            self.options['ticks_color'] = Qt.darkCyan
        elif self.fun_type == 'costfunction':
            self.options['background_color'] = Qt.darkBlue
            self.options['slice_color'] = Qt.blue
            self.options['minmax_color'] = Qt.darkRed
            self.options['ticks_color'] = Qt.darkCyan

    def switchPage(self, index):
        self.stacked_lines.setCurrentIndex(index)

    def updateFunctionNodes(self, parent, fun_name, ranges):
        # transform from float to INT (nodes are integer)
        ranges = listOfListFLOATtoINT(ranges)

        # update bounds widgets
        if self.fun_type == 'constraint':
            self.function_tab.setFunctionBounds(fun_name, ranges)
        # change nodes of function in horizon
        # print('New nodes for Function {}: {}'.format(fun_name, ranges))
        self.horizon_receiver.updateFunctionNodes(fun_name, ranges)

        # update ranges in sliders
        if parent == 'multi':
            self.function_tab.setFunctionNodes(fun_name, ranges)
        elif parent == 'single':
            self.multi_function_box.setFunctionNodes(fun_name, ranges)

    # # todo isn't it better to pass a matrix with rows as dim and columns as nodes?
    # def updateFunctionLbAll(self, fun_name, bounds):
    #     print('function {} changed LOWER bounds: {}'.format(fun_name, bounds))
    #     self.horizon_receiver.updateFunctionLowerBounds(fun_name, bounds)
    #
    # def updateFunctionUbAll(self, fun_name, bounds):
    #     print('function {} changed UPPER bounds: {}'.format(fun_name, bounds))
    #     self.horizon_receiver.updateFunctionLowerBounds(fun_name, )

    def updateFunctionLb(self, fun_name, node, bounds):
        # print('function {} changed at node {}, new LOWER bounds: {}'.format(fun_name, node, bounds.transpose()))
        self.horizon_receiver.updateFunctionLowerBounds(fun_name, bounds, node)

    def updateFunctionUb(self, fun_name, node, bounds):
        # print('function {} changed at node {}, new UPPER bounds: {}'.format(fun_name, node, bounds.transpose()))
        self.horizon_receiver.updateFunctionUpperBounds(fun_name, bounds, node)

    def setHorizonNodes(self, nodes):
        # update nodes
        self.n_nodes = nodes
        # update nodes in horizon
        self.horizon_receiver.setHorizonNodes(nodes)

        # update nodes in first widget (nodes line)
        self.nodes_line.setBoxNodes(nodes)

        # update nodes in second widget (function line) + set margins
        self.function_tab.setHorizonNodes(nodes)
        if self.function_tab.count():
            self.updateMarginsSingleLine()

        # update nodes in multi_function window widget
        self.multi_function_box.setHorizonNodes(nodes)
        if self.multi_function_box.getNFunctions():
            self.updateMarginsMultiLine()

    def updateMarginsSingleLine(self):
        node_box_width = self.nodes_line.getBoxWidth()
        margins = QMargins(node_box_width / 2, 0, node_box_width / 2, 0)  # todo why?
        self.function_tab.updateMargins(margins)

    def updateMarginsMultiLine(self):
        node_box_width = self.nodes_line.getBoxWidth()
        margins = QMargins(node_box_width / 2 + 11, 0, node_box_width / 2 + 11, 0)
        self.multi_function_box.updateMargins(margins)

    #
    def dragEnterEvent(self, event):
        # source_Widget = event.source()
        # items = source_Widget.selectedItems()
        # for i in items:
        #     source_Widget.takeItem(source_Widget.indexFromItem(i).row())
        #     # self.addItem(i)
        #     print(i.text())
        #     print('drop event')
        # standard format of mimeData from QListWidget
        if event.mimeData().hasFormat('application/x-qabstractitemmodeldatalist') and self.n_nodes != 0:
            event.accept()
        else:
            event.ignore()

    def dropEvent(self, event):
        source_item = QStandardItemModel()
        source_item.dropMimeData(event.mimeData(), Qt.CopyAction, 0, 0, QModelIndex())
        fun_name = source_item.item(0, 0).text()
        # fun = source_item.item(0, 0).data(Qt.UserRole)

        self.addFunctionToHorizon(fun_name)

    @pyqtSlot()
    def on_repeated_fun(self, str):
        self.repeated_fun.emit(str)

    def addFunctionToSingleLine(self, name, dim):
        self.function_tab.addFunctionToGUI(name, dim)
        self.updateMarginsSingleLine()

    def addFunctionToMultiLine(self, name):
        self.multi_function_box.addFunctionToGUI(name)
        self.updateMarginsMultiLine()

    def addFunctionToHorizon(self, name):
        flag, signal = self.horizon_receiver.activateFunction(name, self.fun_type)
        dim = self.horizon_receiver.getFunction(name)['active'].getDim()[0]
        if flag:

            self.addFunctionToSingleLine(name, dim)
            self.addFunctionToMultiLine(name)
            self.logger.info(signal)
        else:
            self.logger.warning(signal)

    #
    # def removeFunctionFromHorizon(self, name):
    #     flag, signal = self.horizon_receiver.removeActiveFunction(name)

    def removeActiveFunctionRequestFromIndex(self, index):

        fun_name = self.function_tab.tabText(index)
        self.removeActiveFunctionRequestFromName(fun_name)

    def removeActiveFunctionRequestFromName(self, fun_name):

        flag, signal = self.horizon_receiver.removeActiveFunction(fun_name)

        if flag:
            self.multi_function_box.removeFunctionFromGUI(fun_name)
            for i in range(self.function_tab.count()):
                if fun_name == self.function_tab.tabText(i):
                    self.function_tab.removeTab(i)

        self.logger.info(signal)
