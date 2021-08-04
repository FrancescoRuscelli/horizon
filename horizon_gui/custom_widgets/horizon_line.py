from PyQt5.QtWidgets import QWidget, QStackedWidget, QVBoxLayout, QScrollArea, QTableWidget, \
    QLabel, QTableWidgetItem, QGridLayout, QAbstractScrollArea, QSizePolicy, QHeaderView, QLayout

from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot, QModelIndex, QMargins
from PyQt5.QtGui import QStandardItemModel

from horizon_gui.custom_widgets.node_box_line import NodeBoxLine
from horizon_gui.custom_widgets.function_tab import FunctionTabWidget
from horizon_gui.custom_widgets.multi_function_box import MultiFunctionBox

from horizon_gui.custom_widgets.on_destroy_signal_window import DestroySignalWindow
from horizon.misc_function import listOfListFLOATtoINT, unravelElements
from functools import partial
import os

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
    active_fun_horizon = pyqtSignal()
    remove_fun_horizon = pyqtSignal()
    repeated_fun = pyqtSignal(str)
    bounds_changed = pyqtSignal()
    function_nodes_changed = pyqtSignal()

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
        self.nodes_line.buttonPressed.connect(self.openProblemDetails)
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
        self.function_tab.funNodesChanged.connect(self.function_nodes_changed.emit)
        self.function_tab.funLbChanged.connect(self.updateFunctionLb)
        self.function_tab.funLbChanged.connect(self.bounds_changed.emit)
        self.function_tab.funUbChanged.connect(self.updateFunctionUb)
        self.function_tab.funUbChanged.connect(self.bounds_changed.emit)


    def _initMultiLine(self):

        self.multi_function_box = MultiFunctionBox(self.n_nodes, options=self.options)
        self.stacked_lines.addWidget(self.multi_function_box)

        self.multi_function_box.funCloseRequested.connect(self.removeActiveFunctionRequestFromName)
        self.multi_function_box.funNodesChanged.connect(partial(self.updateFunctionNodes, 'multi'))
        self.multi_function_box.funNodesChanged.connect(self.function_nodes_changed.emit)

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
        # TODO HERE I SHOULD DO UNRAVEL
        # update bounds widgets
        nodes = unravelElements(ranges)
        if self.fun_type == 'constraint':
            self.function_tab.setFunctionBounds(fun_name, nodes)

        # change nodes of function in horizon
        # print('New nodes for Function {}: {}'.format(fun_name, ranges))
        self.horizon_receiver.updateFunctionNodes(fun_name, nodes)

        # update ranges in sliders
        if parent == 'multi':
            self.function_tab.setFunctionNodes(fun_name, ranges)
        elif parent == 'single':
            self.multi_function_box.setFunctionNodes(fun_name, ranges)

    def updateFunctionLb(self, fun_name, node, bounds):
        # print('function {} changed at node {}, new LOWER bounds: {}'.format(fun_name, node, bounds.transpose()))
        self.horizon_receiver.updateFunctionLowerBounds(fun_name, bounds, node)

    def updateFunctionUb(self, fun_name, node, bounds):
        # print('function {} changed at node {}, new UPPER bounds: {}'.format(fun_name, node, bounds.transpose()))
        self.horizon_receiver.updateFunctionUpperBounds(fun_name, bounds, node)

    def setHorizonNodes(self, nodes):
        # update nodes
        self.n_nodes = nodes
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
        self.active_fun_horizon.emit()
        self.addFunctionToHorizon(fun_name)

    @pyqtSlot()
    def on_repeated_fun(self, str):
        self.repeated_fun.emit(str)

    def addFunctionToSingleLine(self, name, dim, initial_bounds):
        self.function_tab.addFunctionToGUI(name, dim, initial_bounds)
        self.updateMarginsSingleLine()

    def addFunctionToMultiLine(self, name):
        self.multi_function_box.addFunctionToGUI(name)
        self.updateMarginsMultiLine()

    def addFunctionToHorizon(self, name):
        flag, signal = self.horizon_receiver.activateFunction(name, self.fun_type)

        dim = self.horizon_receiver.getFunction(name)['active'].getDim()[0]
        initial_bounds = self.horizon_receiver.getFunction(name)['active'].getBounds()
        if flag:

            self.addFunctionToSingleLine(name, dim, initial_bounds)
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

    def openProblemDetails(self, node):

        self.info_box = DestroySignalWindow()
        self.info_box.setWindowTitle('Node {}'.format(node))
        self.info_box.show()
        info_box_layout = QGridLayout(self.info_box)

        if os.name == 'posix':
            info_box_layout.setSizeConstraint(QLayout.SetFixedSize)

        label_title_state_var = QLabel('State Variables:')
        info_box_layout.addWidget(label_title_state_var, 0, 0)
        table_vars = QTableWidget()
        table_vars.setSizeAdjustPolicy(QAbstractScrollArea.AdjustToContents)
        table_vars.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)
        # table_vars.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

        table_vars.setColumnCount(4)
        table_vars.setHorizontalHeaderLabels(['Var Impl', 'Lower Bounds', 'Upper Bounds', 'Initial Guess'])

        info_box_layout.addWidget(table_vars, 0, 1)

        vars, cnstrs, costfuns = self.horizon_receiver.getInfoAtNodes(node)

        for elem in vars:
            rowPosition = table_vars.rowCount()
            table_vars.insertRow(rowPosition)
            table_vars.setItem(rowPosition, 0, QTableWidgetItem((str(elem['var']))))
            table_vars.setItem(rowPosition, 1, QTableWidgetItem((str(elem['lb']))))
            table_vars.setItem(rowPosition, 2, QTableWidgetItem((str(elem['ub']))))
            table_vars.setItem(rowPosition, 3, QTableWidgetItem((str(elem['w0']))))

        table_vars.resizeColumnsToContents()

        label_title_constraint = QLabel('Constraint:')
        info_box_layout.addWidget(label_title_constraint, 1, 0)
        table_constr = QTableWidget()
        table_constr.setSizeAdjustPolicy(QAbstractScrollArea.AdjustToContents)
        table_constr.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)

        table_constr.setColumnCount(4)
        table_constr.setHorizontalHeaderLabels(['Name', 'Function', 'Lower Bounds', 'Upper Bounds'])
        info_box_layout.addWidget(table_constr, 1, 1)

        if cnstrs is not None:
            for name, item in cnstrs.items():
                rowPosition = table_constr.rowCount()
                table_constr.insertRow(rowPosition)
                table_constr.setItem(rowPosition, 0, QTableWidgetItem(name))
                table_constr.setItem(rowPosition, 1, QTableWidgetItem((str(item['val']))))
                table_constr.setItem(rowPosition, 2, QTableWidgetItem((str(item['lb']))))
                table_constr.setItem(rowPosition, 3, QTableWidgetItem((str(item['ub']))))

        table_constr.resizeColumnsToContents()

        label_title_costfun = QLabel('Cost Functions:')
        info_box_layout.addWidget(label_title_costfun, 2, 0)
        table_costfun = QTableWidget()
        table_costfun.setSizeAdjustPolicy(QAbstractScrollArea.AdjustToContents)
        table_costfun.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)

        table_costfun.setColumnCount(2)
        table_costfun.setHorizontalHeaderLabels(['Name', 'Function'])
        info_box_layout.addWidget(table_costfun, 2, 1)

        if cnstrs is not None:
            for name, item in costfuns.items():
                rowPosition = table_costfun.rowCount()
                table_costfun.insertRow(rowPosition)
                table_costfun.setItem(rowPosition, 0, QTableWidgetItem(name))
                table_costfun.setItem(rowPosition, 1, QTableWidgetItem((str(item))))

        table_costfun.resizeColumnsToContents()