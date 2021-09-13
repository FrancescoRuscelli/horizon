import os
from horizon_gui.gui.model_module.model_handler import ModelHandler
from horizon_gui.custom_widgets.generic_display_mask import GenericDisplayMask
from PyQt5.QtWidgets import QFileDialog, QApplication, QGridLayout, QLabel, QLineEdit, QWidget, QRadioButton, QPushButton
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtGui import QPalette
from horizon_gui.definitions import ROOT_DIR
import sys
import ntpath


class ModelGui(GenericDisplayMask):
    modelLoaded = pyqtSignal(bool)

    def __init__(self, logger=None, parent=None):
        super().__init__(parent)

        self.logger = logger
        self.model_handler = ModelHandler()

    def openWindow(self):

        super().openWindow()

        self.main_widget_layout = QGridLayout()
        self.main_widget.setLayout(self.main_widget_layout)

        self.populateWindow()
        self._addVarSelector()

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

    def _addVarSelector(self):

        self.var_widget = QWidget()
        self.var_widget_layout = QGridLayout(self.var_widget)
        self.var_title = QLabel('Variables:')
        self.q_title = QLabel('q:')
        self.q_dot_title = QLabel('q_dot:')
        self.q_ddot_title = QLabel('q_ddot:')

        self.state_title = QLabel('state')
        self.input_title = QLabel('input')

        self.state_button = [QRadioButton(), QRadioButton(), QRadioButton()]
        self.input_button = [QRadioButton(), QRadioButton(), QRadioButton()]

        self.var_widget_layout.addWidget(self.state_title, 0, 1)
        self.var_widget_layout.addWidget(self.input_title, 0, 2)

        self.var_widget_layout.addWidget(self.q_title, 1, 0)
        self.var_widget_layout.addWidget(self.q_dot_title, 2, 0)
        self.var_widget_layout.addWidget(self.q_ddot_title, 3, 0)

        # QButtonGroup
        self.var_widget_layout.addWidget(self.state_button[0], 1, 1)
        self.var_widget_layout.addWidget(self.state_button[1], 2, 1)
        self.var_widget_layout.addWidget(self.state_button[2], 3, 1)

        self.var_widget_layout.addWidget(self.input_button[0], 1, 2)
        self.var_widget_layout.addWidget(self.input_button[1], 2, 2)
        self.var_widget_layout.addWidget(self.input_button[2], 3, 2)



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
    gui = ModelGui()
    gui.show()

    sys.exit(app.exec_())
