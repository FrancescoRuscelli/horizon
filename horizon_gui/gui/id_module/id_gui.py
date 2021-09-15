import os
from horizon_gui.gui.gui_receiver import horizonImpl
from horizon_gui.gui.model_module.model_handler import ModelHandler
from horizon_gui.custom_widgets.generic_display_mask import GenericDisplayMask
from horizon_gui.custom_widgets.display_line import DisplayLine
from horizon_gui.custom_widgets.on_destroy_signal_window import DestroySignalWindow
from PyQt5.QtWidgets import QFileDialog, QApplication, QGridLayout, QLabel, QWidget, QLineEdit, QRadioButton, QPushButton, \
    QButtonGroup, QListWidget
from PyQt5.QtCore import pyqtSignal
import sys
from functools import partial



class InverseDynamicsGui(QWidget):
    idComputed = pyqtSignal(str, str)

    def __init__(self, horizon_receiver: horizonImpl, logger=None, parent=None):
        super().__init__(parent)

        self.horizon_receiver = horizon_receiver
        self.model_handler = self.horizon_receiver.getModelHandler()
        self.logger = logger

        self.variables_name = ['q', 'q_dot', 'q_ddot']
        self.variables = dict(zip(self.variables_name, [None]* len(self.variables_name)))

        self.main_layout = QGridLayout(self)
        self.button = QPushButton('ID')
        self.main_layout.addWidget(self.button)
        self.button.clicked.connect(self.openIDWidget)

        # self._initDummy()

    def _initDummy(self):

        urdf_file = '/home/francesco/catkin_ws/external/casadi_horizon/horizon/examples/urdf/cart_pole.urdf'
        # urdf_file = '/home/francesco/hhcm_workspace/src/horizon/horizon/examples/urdf/cart_pole.urdf'
        urdf = open(urdf_file, 'r').read()
        self.horizon_receiver.getModelHandler().setModel(urdf)
        self.horizon_receiver.createVariable('State', 'x', 2, 0, [1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
        self.horizon_receiver.createVariable('State', 'x_dot', 2, 0, [1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
        self.horizon_receiver.createVariable('State', 'x_ddot', 2, 0, [1, 2, 3, 4, 5, 6, 7, 8, 9, 10])

        self.setVar('q', 'x')
        self.setVar('q_dot', 'x_dot')
        self.setVar('q_ddot', 'x_ddot')


    def openIDWidget(self):
        self.id_widget = DestroySignalWindow()
        self.id_widget_layout = QGridLayout(self.id_widget)


        id_title = QLabel('Variables:')
        self.id_widget_layout.addWidget(id_title, 0, 0)

        self.gen_id_button = QPushButton('Generate ID')
        self.gen_id_button.clicked.connect(self.gen_id_clicked)
        self.gen_id_button.setDisabled(True)

        for i in range(len(self.variables_name)):
            id_var_title = QLabel(f'{self.variables_name[i]}:')
            self.id_widget_layout.addWidget(id_var_title, i + 1, 0)
            display = DisplayLine()
            display.setObjectName(self.variables_name[i])
            self.id_widget_layout.addWidget(display, i + 1, 1)

            open_button = QPushButton('...')
            open_button.clicked.connect(partial(self.openVar, display))
            self.id_widget_layout.addWidget(open_button, i + 1, 2)
            if self.variables[self.variables_name[i]] is not None:
                self.updateDisplay(display, self.variables_name[i])


        self.id_widget_layout.addWidget(self.gen_id_button)


        self.checkReady()

        self.id_widget.show()

    def checkReady(self):
        if self.model_handler.getKinDyn() is not None and None not in self.variables.values():
            self.gen_id_button.setDisabled(False)

    def openVar(self, display):
        self.list_var = QListWidget()

        for elem in self.horizon_receiver.getVar().keys():
            self.list_var.addItem(elem)

        self.list_var.show()

        self.list_var.itemDoubleClicked.connect(partial(self.on_item_double_clicked, display))
        self.list_var.itemDoubleClicked.connect(self.list_var.close)


    def on_item_double_clicked(self, display: DisplayLine, item):
        var_name = item.text()
        self.setVar(display.objectName(), var_name)
        self.updateDisplay(display, var_name)

    def setVar(self, var_type, var_name):
        self.variables[var_type] = self.horizon_receiver.getVar(var_name)

    def updateDisplay(self, display: DisplayLine, text):
        display.setText(text)
        display.setReady(True)
        self.checkReady()

    def gen_id_clicked(self):
        tau = self.model_handler.computeID(self.variables['q']['var'], self.variables['q_dot']['var'], self.variables['q_ddot']['var'])
        self.horizon_receiver.appendFun(tau, 'tau_id', 'tau')
        self.idComputed.emit('tau_id', 'tau')


if __name__ == '__main__':

    app = QApplication(sys.argv)
    hr = horizonImpl(10)


    gui = InverseDynamicsGui(hr)
    gui.show()

    sys.exit(app.exec_())
