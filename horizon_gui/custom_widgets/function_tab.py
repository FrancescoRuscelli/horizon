from PyQt5.QtWidgets import QWidget, QTabWidget, QHBoxLayout, QVBoxLayout
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot

from horizon_gui.definitions import CSS_DIR
from horizon_gui.custom_widgets.function_line import FunctionLine
from horizon_gui.custom_widgets.bounds_line import BoundsLine

import numpy as np
from functools import partial

from horizon.misc_function import unravelElements

class FunctionTabWidget(QTabWidget):
    funNodesChanged = pyqtSignal(str, list)
    # funLbChanged = pyqtSignal(str, np.matrix)
    # funUbChanged = pyqtSignal(str, np.matrix)
    funLbChanged = pyqtSignal(str, int, list)
    funUbChanged = pyqtSignal(str, int, list)

    def __init__(self, n_nodes, options=None, parent=None, bounds_flag=True):
        super().__init__(parent)

        self.bounds_flag = bounds_flag
        self.n_nodes = n_nodes
        self.options = options

        with open(CSS_DIR + '/tab.css', 'r') as f:
            self.setStyleSheet(f.read())

        self.setTabPosition(1)
        self.setTabsClosable(True)
        self.setAttribute(Qt.WA_StyledBackground, True)


    def addFunctionToGUI(self, fun_name, dim, initial_bounds):
        self.ft = FunctionLine(fun_name, self.n_nodes, options=self.options)
        self.ft.nodesChanged.connect(self.on_fun_nodes_changed)

        # todo hardcoded bottom margin
        self.intab_layout = QVBoxLayout()
        self.intab_layout.setSpacing(0)

        self.intab_layout.addWidget(self.ft)
        # self.intab_layout.addSpacing(120)
        if self.bounds_flag:
            # todo adding initial value?
            self.bl = BoundsLine(fun_name, self.n_nodes, dim, initial_bounds)
            self.bl.lbChanged.connect(partial(self.on_fun_lb_changed, fun_name))
            self.bl.ubChanged.connect(partial(self.on_fun_ub_changed, fun_name))

            self.intab_layout.addWidget(self.bl)

        self.tab = QWidget()
        self.tab.setStyleSheet('background-color: blue;')
        self.tab.setLayout(self.intab_layout)
        self.addTab(self.tab, str(fun_name))

    @pyqtSlot(str, list)
    def on_fun_nodes_changed(self, fun_name, ranges):
        self.funNodesChanged.emit(fun_name, ranges)

    # @pyqtSlot(str, np.matrix)
    # def on_fun_lb_changed_all(self, fun_name, lims):
    #     self.funLbChanged.emit(fun_name, lims)
    #
    # @pyqtSlot(str, np.matrix)
    # def on_fun_ub_changed_all(self, fun_name, lims):
    #     self.funUbChanged.emit(fun_name, lims)

    @pyqtSlot(str, int, list)
    def on_fun_lb_changed(self, fun_name, node, lims):
        self.funLbChanged.emit(fun_name, node, lims)

    @pyqtSlot(str, int, list)
    def on_fun_ub_changed(self, fun_name, node, lims):
        self.funUbChanged.emit(fun_name, node, lims)


    def setFunctionNodes(self, fun_name, ranges):
        for widget_fl in self.findChildren(FunctionLine):
            if widget_fl.getName() == fun_name:
                widget_fl.updateSlices(ranges)


    def setFunctionBounds(self, fun_name, active_nodes):

        # # update the widget bounds (spin_boxes) for each functions
        inactive_nodes = [inactive_n for inactive_n in range(self.n_nodes) if inactive_n not in active_nodes]

        for widget_bl in self.findChildren(BoundsLine):
            if widget_bl.getName() == fun_name:
                widget_bl.showNodes(active_nodes)
                widget_bl.hideNodes(inactive_nodes)

    def updateMargins(self, margins):
        # self.intab_layout.setContentsMargins(margins)
        for i in range(self.intab_layout.count()):
            if isinstance(self.intab_layout.itemAt(i).widget(), FunctionLine):
                self.intab_layout.itemAt(i).widget().hlayout.setContentsMargins(margins)

    def setHorizonNodes(self, nodes):
        self.n_nodes = nodes
        for i in range(self.count()):
            functionline = self.widget(i).findChild(FunctionLine)
            if functionline:
                functionline.updateHorizonNodes(self.n_nodes)

            boundsline = self.widget(i).findChild(BoundsLine)
            if boundsline:
                boundsline.setNodes(self.n_nodes)

