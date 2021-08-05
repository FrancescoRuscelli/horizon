from PyQt5.QtWidgets import QWidget, QTabWidget, QHBoxLayout, QVBoxLayout
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot

from horizon_gui.definitions import CSS_DIR
from horizon_gui.custom_widgets.multi_slider import QMultiSlider

# todo isolate it in its own file?
class FunctionLine(QWidget):
    nodesChanged = pyqtSignal(str, list)

    def __init__(self, name, n_nodes, disabled_nodes=None, options=None, parent=None):
        super().__init__(parent)

        minimum_bar_height = 80
        maximum_bar_height = 80

        self.name = name
        self.n_nodes = n_nodes

        self.hlayout = QHBoxLayout(self)
        self.hlayout.setAlignment(Qt.AlignTop)
        self.slider = QMultiSlider(slider_range=[0, self.n_nodes - 1, 1], values=[0, self.n_nodes-1], options=options)
        # if disabled_nodes is not None:
        # todo disable the necessary nodes
        #     self.slider.disableValues()
        self.slider.setMinimumHeight(minimum_bar_height)
        self.slider.setMaximumHeight(maximum_bar_height)
        self.hlayout.addWidget(self.slider)
        self.hlayout.setContentsMargins(0, 0, 0, 0)
        self.slider.slicesChanged.connect(self.on_nodes_changed)

    @pyqtSlot(list)
    def on_nodes_changed(self, range_list):
        self.nodesChanged.emit(self.name, range_list)

        # adding + - push button to tab
        # self.ct_tab.tabBar().setTabButton(0, self.ct_tab.tabBar().RightSide, TabButtonWidget())

    def updateHorizonNodes(self, n_nodes):
        self.n_nodes = n_nodes
        self.slider.updateRange([0, self.n_nodes-1])

    def updateSlices(self, slices):
        self.slider.updateSlices(slices)

    def getName(self):
        return self.name