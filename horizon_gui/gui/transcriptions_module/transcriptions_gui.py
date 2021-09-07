from PyQt5.QtWidgets import QGridLayout, QLabel, QPushButton, QHBoxLayout, QWidget, QComboBox, QSpinBox, QApplication
from horizon_gui.custom_widgets import on_destroy_signal_window, separation_line, option_container
from horizon_gui.custom_widgets.generic_display_mask import GenericDisplayMask
from PyQt5.QtCore import pyqtSignal
from horizon_gui.gui.gui_receiver import horizonImpl
import sys

class TranscriptionGui(GenericDisplayMask):
    def __init__(self, horizon_receiver: horizonImpl, logger=None, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f'{self.__class__.__name__}')
        self.horizon_receiver = horizon_receiver
        self.logger = logger

        self.trans_method = None
        self.options = None


    def openWindow(self):

        super().openWindow()

        self.main_widget_layout = QGridLayout()
        self.main_widget.setLayout(self.main_widget_layout)

        self.populateWindow()

    def populateWindow(self):

        n_row = 0
        # populate it
        self.label_type = QLabel("Types:")
        self.main_widget_layout.addWidget(self.label_type, n_row, 0, 1, 1)

        n_row = n_row + 1
        self.trans_combo_box = QComboBox()
        self.trans_combo_box.addItem('multiple_shooting')
        self.trans_combo_box.addItem('direct_collocation')
        self.trans_combo_box.setCurrentIndex(-1)
        self.trans_combo_box.currentTextChanged.connect(self.editTranscriptionOptions)
        self.main_widget_layout.addWidget(self.trans_combo_box, n_row, 0, 1, 1)

        n_row = n_row + 1
        separation_line_1 = separation_line.SeparationLine()
        self.main_widget_layout.addWidget(separation_line_1, n_row, 0, 1, 1)

        n_row = n_row + 1
        self.label_options = QLabel("Options:")
        self.main_widget_layout.addWidget(self.label_options, n_row, 0, 1, 1)

        n_row = n_row + 1
        self.option_widget = option_container.OptionContainer()
        self.main_widget_layout.addWidget(self.option_widget, n_row, 0, 1, 1)

        n_row = n_row + 1
        separation_line_2 = separation_line.SeparationLine()
        self.main_widget_layout.addWidget(separation_line_2, n_row, 0, 1, 1)

        # connect buttons
        self.no_button.clicked.connect(self.win.close)
        self.yes_button.clicked.connect(self.yesClicked)
        self.yes_button.clicked.connect(self.win.close)


    def editTranscriptionOptions(self, type):
        self.option_widget.clear()
        self.yes_button.setDisabled(False)

        if type == 'multiple_shooting':
            # todo optimize
            integrator_combo_box = QComboBox()
            integrator_combo_box.addItem('EULER')
            integrator_combo_box.addItem('RK2')
            integrator_combo_box.addItem('RK4')
            integrator_combo_box.addItem('LEAPFROG')
            self.option_widget.addOption('Integrator', integrator_combo_box, integrator_combo_box.currentText)

        elif type == 'direct_collocation':
            degree_spin_box = QSpinBox()
            degree_spin_box.setValue(3)
            self.option_widget.addOption('Degree', degree_spin_box, degree_spin_box.value)

    def yesClicked(self):
        type = self.trans_combo_box.currentText()
        opts = self.option_widget.getOptions()

        self.setTranscriptionMethod(type, opts)

    def setTranscriptionMethod(self, type, opts):
        self.horizon_receiver.setTranscriptionMethod(type, opts)

        self.display.setText(type)
        self.display.setReady(True)

    def updateTranscriptionMethod(self):
        trans_meth = self.horizon_receiver.getTranscriptionMethod()
        if trans_meth is not None:
            self.setTranscriptionMethod(trans_meth['type'], trans_meth['opts'])


if __name__ == '__main__':
    hi = horizonImpl(2)
    app = QApplication(sys.argv)
    gui = TranscriptionGui(hi)
    gui.show()

    sys.exit(app.exec_())