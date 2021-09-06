import os
from casadi_kin_dyn import pycasadi_kin_dyn as cas_kin_dyn
from horizon_gui.custom_widgets.display_line import DisplayLine
from horizon_gui.custom_widgets.on_destroy_signal_window import DestroySignalWindow
from PyQt5.QtWidgets import QFileDialog, QApplication, QWidget, QHBoxLayout, QPushButton, QVBoxLayout, QGridLayout
from PyQt5.QtCore import pyqtSignal
import sys

class GenericDisplayMask(QWidget):
    modelLoaded = pyqtSignal(bool)

    def __init__(self, logger=None, parent=None):
        super().__init__(parent)

        self.logger = logger
        self.main_layout = QHBoxLayout(self)

        self.display = DisplayLine()
        self.main_layout.addWidget(self.display)

        self.open_button = QPushButton('...')
        self.open_button.clicked.connect(self.openWindow)
        self.main_layout.addWidget(self.open_button)

    def openWindow(self):

        self.win = DestroySignalWindow()

        # create a layout for the window
        # widget, fromRow, int fromColumn, int rowSpan, int columnSpan, Qt::Alignment alignment = Qt::Alignment()
        self.trans_window_layout = QGridLayout(self.win)
        self.main_widget = QWidget()

        self.trans_window_layout.addWidget(self.main_widget)
        self.initDialog(self.trans_window_layout)

        self.win.show()

    def initDialog(self, parent_layout):
        widget_dialog = QWidget()
        widget_dialog_layout = QHBoxLayout(widget_dialog)
        parent_layout.addWidget(widget_dialog)

        self.no_button = QPushButton('Cancel')
        self.yes_button = QPushButton('Load')

        self.yes_button.setDisabled(True)

        widget_dialog_layout.addWidget(self.no_button)
        widget_dialog_layout.addWidget(self.yes_button)




if __name__ == '__main__':

    app = QApplication(sys.argv)
    gui = GenericDisplayMask()
    gui.show()

    sys.exit(app.exec_())