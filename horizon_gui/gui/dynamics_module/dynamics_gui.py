from PyQt5.QtWidgets import QGridLayout, QLabel, QPushButton, QHBoxLayout, QWidget, QComboBox, QTabWidget, QApplication
from horizon_gui.custom_widgets import on_destroy_signal_window
from PyQt5.QtCore import Qt, QRect, QSize, QPoint, pyqtSlot, pyqtSignal
from horizon_gui.custom_widgets.generic_display_mask import GenericDisplayMask
from horizon_gui.gui.variables_module.variables_gui import VariablesGui
from horizon_gui.gui.gui_receiver import horizonImpl
from horizon_gui.custom_widgets import line_edit
import sys

class DynamicsGui(GenericDisplayMask):
    def __init__(self, horizon_receiver: horizonImpl, logger=None, parent=None):
        super().__init__(parent)

        self.horizon_receiver = horizon_receiver
        self.logger = logger

        # todo instead of using signals and slot, I embedded the display from the main_interface to this gui.
        #   probably the best thing to do is making this widget completely independent, and adding it to the main_interface
        #   even doing a small separate ui would be nice

    def openWindow(self):

        super().openWindow()

        self.main_widget_layout = QGridLayout()
        self.main_widget.setLayout(self.main_widget_layout)

        self.populateWindow()


    def populateWindow(self):

        self.dyn_tab = QTabWidget()
        self.main_widget_layout.addWidget(self.dyn_tab, 0, 0, 1, 1)

        tab_default = QWidget()
        tab_default.setObjectName('default')
        tab_default_layout = QHBoxLayout(tab_default)

        label_dyn_default = QLabel('xdot:')
        tab_default_layout.addWidget(label_dyn_default)

        self.dyn_combo_box = QComboBox()
        dyn_methods_list = self.horizon_receiver.getDefaultDynList()
        for item in dyn_methods_list:
            self.dyn_combo_box.addItem(item)

        self.dyn_combo_box.setCurrentIndex(-1)
        self.dyn_combo_box.currentIndexChanged.connect(self.manageYesDynButton)
        tab_default_layout.addWidget(self.dyn_combo_box)

        self.dyn_tab.addTab(tab_default, 'Default')

        tab_custom = QWidget()
        tab_custom.setObjectName('custom')
        tab_custom_layout = QHBoxLayout(tab_custom)

        label_dyn_default = QLabel('xdot:')
        tab_custom_layout.addWidget(label_dyn_default)

        self.dyn_line_edit = line_edit.LineEdit()
        self.dyn_line_edit.textChanged.connect(self.manageYesDynButton)
        tab_custom_layout.addWidget(self.dyn_line_edit)

        self.dyn_tab.addTab(tab_custom, 'Custom')

        self.no_button.clicked.connect(self.win.close)
        self.yes_button.clicked.connect(self.setDynamics)
        self.yes_button.clicked.connect(self.win.close)

        self.dyn_tab.currentChanged.connect(self.manageYesDynButton)

    def manageYesDynButton(self):

        if self.dyn_tab.currentWidget().objectName() == 'custom':
            # if self.dyn_tab.currentWidget().objectName():
            if len(self.dyn_line_edit.toPlainText()) > 0:
                self.yes_button.setDisabled(False)
            else:
                self.yes_button.setDisabled(True)
        elif self.dyn_tab.currentWidget().objectName() == 'default':
            if self.dyn_combo_box.currentIndex() == -1:
                self.yes_button.setDisabled(True)
            else:
                self.yes_button.setDisabled(False)

    def setDynamics(self):
        # todo can optimize form
        type_dyn = self.dyn_tab.currentWidget().objectName()

        if type_dyn == 'custom':
            dyn = self.dyn_line_edit.toPlainText()
            self.horizon_receiver.setDynamics(type_dyn, dyn)
            self.display.setText(type_dyn)
        elif type_dyn == 'default':
            dyn = self.dyn_combo_box.currentText()
            self.horizon_receiver.setDynamics(type_dyn, dyn)
            self.display.setText(dyn)

        self.display.setReady(True)

    def manageDisplay(self):
        if not self.horizon_receiver.isDynamicsReady():
            self.display.clear()
            self.display.setReady(False)

if __name__ == '__main__':
    hi = horizonImpl(2)
    app = QApplication(sys.argv)
    gui = DynamicsGui(hi)
    gui.show()

    sys.exit(app.exec_())