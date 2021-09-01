import sys
from functools import partial

from PyQt5.QtWidgets import (QGridLayout, QLabel, QPushButton, QRadioButton, QVBoxLayout, QHBoxLayout, QWidget,
                             QLineEdit, QTableWidgetItem, QTableWidget, QCompleter, QHeaderView, QDialogButtonBox,
                             QComboBox, QSpinBox, QFrame, QApplication)

from PyQt5.QtGui import QPalette, QFont, QColor
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot


class OptionContainer(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.layout = QGridLayout(self)
        self.row = 0
        self.options_dict = dict()

    def addOption(self, name, widget, getter_method, aligned=False, width=1):

        label_option = QLabel(f'{name}:')
        self.layout.addWidget(label_option, self.row, 0)
        if aligned:
            self.layout.addWidget(widget, self.row, 1, 1, width)
            self.row = self.row + 1
        else:
            self.row = self.row + 1
            self.layout.addWidget(widget, self.row, 0, 1, width)
            self.row = self.row + 1

        self.options_dict[name] = getter_method

    def getOptions(self, name=None):
        if name is None:
            opts = dict()
            for name, item in self.options_dict.items():
                opts[name.lower()] = self.options_dict[name]()
            return opts
        else:

            if name in self.options_dict:
                val = self.options_dict[name]()
            else:
                val = None

            return val


if __name__ == '__main__':

    app = QApplication(sys.argv)
    gui = OptionContainer()


    integrator_combo_box = QComboBox()
    integrator_combo_box.addItem('EULER')
    integrator_combo_box.addItem('RK2')
    integrator_combo_box.addItem('RK4')
    integrator_combo_box.addItem('LEAPFROG')

    gui.addOption('Integrator', integrator_combo_box, integrator_combo_box.currentText, width=2)

    degree_spin_box = QSpinBox()
    degree_spin_box.setValue(3)
    gui.addOption('Degree', degree_spin_box, degree_spin_box.value, aligned=True)

    button = QPushButton('daniele')
    button.clicked.connect(lambda: gui.getOptions())
    button.show()

    gui.show()
    sys.exit(app.exec_())
