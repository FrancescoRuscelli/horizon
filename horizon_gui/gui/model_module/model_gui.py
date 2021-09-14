import os
from horizon_gui.gui.gui_receiver import horizonImpl
from horizon_gui.gui.model_module.model_handler import ModelHandler
from horizon_gui.custom_widgets.generic_display_mask import GenericDisplayMask
from horizon_gui.custom_widgets.display_line import DisplayLine
from horizon_gui.custom_widgets.on_destroy_signal_window import DestroySignalWindow
from PyQt5.QtWidgets import QFileDialog, QApplication, QGridLayout, QLabel, QLineEdit, QRadioButton, QPushButton, QButtonGroup, QListWidget
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtGui import QPalette
from horizon_gui.definitions import ROOT_DIR
import sys
from functools import partial
import ntpath


class ModelGui(GenericDisplayMask):
    modelLoaded = pyqtSignal(bool)

    def __init__(self, horizon_receiver: horizonImpl, logger=None, parent=None):
        super().__init__(parent)

        self.logger = logger
        self.horizon_receiver = horizon_receiver
        self.model_handler = self.horizon_receiver.getModelHandler()

    def openWindow(self):

        super().openWindow()

        self.main_widget_layout = QGridLayout()
        self.main_widget.setLayout(self.main_widget_layout)

        self.populateWindow()
        # self._initVarSelector()

    def populateWindow(self):
        self.yes_button.setDisabled(False)
        self.info_title = QLabel('Info URDF')
        self.main_widget_layout.addWidget(self.info_title, 0, 0)

        self.nq_title = QLabel('nq:')
        self.main_widget_layout.addWidget(self.nq_title, 1, 0)

        self.nq_line = QLineEdit()
        self.nq_line.setDisabled(True)
        palette = QPalette()
        palette.setColor(QPalette.Text, Qt.darkGray)
        self.nq_line.setPalette(palette)
        self.nq_line.setAlignment(Qt.AlignCenter)
        self.main_widget_layout.addWidget(self.nq_line, 1, 1)

        self.nv_title = QLabel('nv:')
        self.main_widget_layout.addWidget(self.nv_title, 2, 0)

        self.nv_line = QLineEdit()
        self.nv_line.setDisabled(True)
        palette = QPalette()
        palette.setColor(QPalette.Text, Qt.darkGray)
        self.nv_line.setPalette(palette)
        self.nv_line.setAlignment(Qt.AlignCenter)
        self.main_widget_layout.addWidget(self.nv_line, 2, 1)

        # self.var_button = QPushButton('Variables')
        # self.main_widget_layout.addWidget(self.var_button, 3, 1)

        if self.model_handler.getKinDyn():
            self.nq_line.setText(str(self.model_handler.getNq()))
            self.nv_line.setText(str(self.model_handler.getNqdot()))

        self.no_button.clicked.connect(self.win.close)
        self.yes_button.clicked.connect(self.openFileFromDialog)

    def _initVarSelector(self):

        self.var_widget = DestroySignalWindow()
        self.var_widget_layout = QGridLayout(self.var_widget)
        self.var_title = QLabel('Variables:')

        '______|__state__|__input__'
        'q     |         |         '
        '______|_________|_________'
        'qdot  |         |         '
        '______|_________|_________'
        'qddot |         |         '
        '______|_________|_________'
        vars = ['q', 'q_dot', 'q_ddot']
        types = ['state', 'input']

        self.var_titles = list()
        for var in vars:
            self.var_titles.append(QLabel(f'{var}:'))

        self.type_titles = list()
        for type in types:
            self.type_titles.append(QLabel(type))

        self.type_buttons = list()
        for n in range(len(types)):
            var_buttons = list()
            for i in range(len(vars)):
                var_buttons.append(QRadioButton())
            self.type_buttons.append(var_buttons)

        button_groups = list()

        for i in range(len(vars)):
            temp_group = QButtonGroup(self)
            for type_button in self.type_buttons:
                temp_group.addButton(type_button[i])
            button_groups.append(temp_group)

        # for type_button in self.type_buttons:
        #     type_button[0].setChecked(True)

        for n in range(len(types)):
            self.var_widget_layout.addWidget(self.type_titles[n], 0, n+1)

        for i in range(len(vars)):
            self.var_widget_layout.addWidget(self.var_titles[i], i+1, 0)

        k = 0
        for type_button in self.type_buttons:
            k = k+1
            for i in range(len(type_button)):
                self.var_widget_layout.addWidget(type_button[i], i+1, k)

        self.var_widget.show()

    def openFileFromDialog(self):

        dir = ROOT_DIR
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        # file_selected = QFileDialog.getExistingDirectory(self, "Open URDF", dir, options=options)
        file_selected, _ = QFileDialog.getOpenFileName(self, "Open URDF", dir, "Urdf Files (*.urdf)", options=options)
        if file_selected:
            self.openFile(file_selected)
        else:
            if self.logger:
                self.logger.warning('File not found. Failed loading urdf file.')

    def openFile(self, file_path):

        # self.updateSettings(file_path) # todo this may be useful later
        if self.logger:
            self.logger.info('Opening urdf file: {}'.format(file_path))

        self.loadModel(file_path)
        self.win.close()

        self.nq_line.setText(str(self.model_handler.getNq()))
        self.nv_line.setText(str(self.model_handler.getNqdot()))
        # self.writeInStatusBar('Opening Horizon problem!')

    def loadModel(self, urdf_file):
        try:
            urdf = open(urdf_file, 'r').read()
            self.model_handler.setModel(urdf)

            # should this go here?
            self.display.setText(ntpath.basename(urdf_file))
            self.display.setReady(True)

        except Exception as e:
            if self.logger:
                self.logger.warning(f'Could not load model in file {urdf_file}: {e}')

    def getKinDin(self):
        return self.model_handler.getKinDyn()

if __name__ == '__main__':

    app = QApplication(sys.argv)
    hr = horizonImpl(10)
    hr.createVariable('State', 'x', 2, 0, [1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
    gui = ModelGui(hr)
    gui.show()

    sys.exit(app.exec_())
