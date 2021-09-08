from PyQt5.QtWidgets import QLabel, QStackedWidget, QWidget, QApplication, QVBoxLayout, QHBoxLayout, QRadioButton, QGridLayout, QPushButton, QSizePolicy, QSpacerItem
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot
from horizon_gui.custom_widgets import on_destroy_signal_window, highlight_delegate, bounds_line, multi_slider, spinbox_line
from horizon_gui.gui.gui_receiver import horizonImpl
from horizon_gui.custom_widgets.horizon_line import HorizonLine
from functools import partial
import sys
import numpy as np

class ProblemGui(QWidget):
    def __init__(self, horizon_receiver: horizonImpl, logger=None, parent=None):
        super().__init__(parent)

        self.horizon_receiver = horizon_receiver
        self.logger = logger

        self._initWidget()
        self._initLines()
        self._connectButtons()

    def _connectButtons(self):

        self.switch_page_button.clicked.connect(self.switchPage)
        self.single_line_button.toggled.connect(partial(self.constraint_line.switchPage, self.constraint_line.Single))
        self.single_line_button.toggled.connect(partial(self.cost_line.switchPage, self.constraint_line.Single))
        self.multiple_line_button.toggled.connect(partial(self.constraint_line.switchPage, self.cost_line.Multi))
        self.multiple_line_button.toggled.connect(partial(self.cost_line.switchPage, self.cost_line.Multi))

    def _initWidget(self):

        self.main_layout = QVBoxLayout(self)
        self.stacked_widget = QStackedWidget()

        self.constraint_page = QWidget()
        self.stacked_widget.addWidget(self.constraint_page)

        self.cost_page = QWidget()
        self.stacked_widget.addWidget(self.cost_page)

        self.main_layout.addWidget(self.stacked_widget)

        self.widget_buttons = QWidget()
        self.widget_buttons_layer = QHBoxLayout(self.widget_buttons)

        # todo how to put choose position of spacer item?
        spacer_item = QSpacerItem(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.widget_buttons_layer.addItem(spacer_item)

        self.line_view = QWidget()
        self.line_view_layout = QHBoxLayout(self.line_view)
        self.label_view_title = QLabel("View:")
        self.line_view_layout.addWidget(self.label_view_title)

        self.multiple_line_button = QRadioButton()
        self.multiple_line_button.setText('Single')
        self.multiple_line_button.setChecked(True)
        self.line_view_layout.addWidget(self.multiple_line_button)

        self.single_line_button = QRadioButton()
        self.single_line_button.setText('Multiple')
        self.single_line_button.setChecked(False)
        self.line_view_layout.addWidget(self.single_line_button)
        self.widget_buttons_layer.addWidget(self.line_view)

        self.switch_page_button = QPushButton('Switch to Cost Functions')
        self.widget_buttons_layer.addWidget(self.switch_page_button)

        self.widget_buttons_layer.addStretch()
        self.main_layout.addWidget(self.widget_buttons)


    def _initLines(self):

        nodes = self.horizon_receiver.getNodes()
        self.constraint_page_layout = QVBoxLayout(self.constraint_page)

        constraint_title = QLabel('Constraints')
        constraint_title.setFont(QFont("Times", 12, QFont.Bold))
        constraint_title.setAlignment(Qt.AlignCenter)
        self.constraint_page_layout.addWidget(constraint_title)

        # self.constraintLine = horizon_multi_line.HorizonMultiLine(self.horizon_receiver, 'constraint', nodes=N, logger=self.logger)
        self.constraint_line = HorizonLine(self.horizon_receiver, 'constraint', nodes=nodes, logger=self.logger)
        self.constraint_line.setContentsMargins(0, 40, 0, 0)
        self.constraint_page_layout.addWidget(self.constraint_line)

        self.cost_page_layout = QVBoxLayout(self.cost_page)

        cost_title = QLabel('Cost Functions')
        cost_title.setFont(QFont("Times", 12, QFont.Bold))
        cost_title.setAlignment(Qt.AlignCenter)
        self.cost_page_layout.addWidget(cost_title)

        self.cost_line = HorizonLine(self.horizon_receiver, 'cost', nodes=nodes, logger=self.logger)
        self.cost_line.setContentsMargins(0, 40, 0, 0)
        self.cost_page_layout.addWidget(self.cost_line)

    def switchPage(self):
        index = self.stacked_widget.currentIndex()
        if index == 0:
            self.switch_page_button.setText('Switch to Constraints')
        else:
            self.switch_page_button.setText('Switch to Cost Functions')

        self.stacked_widget.setCurrentIndex(abs(index-1))

if __name__ == '__main__':
    hi = horizonImpl(2)
    app = QApplication(sys.argv)
    gui = ProblemGui(hi)
    gui.show()

    sys.exit(app.exec_())