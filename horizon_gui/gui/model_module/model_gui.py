import os
from casadi_kin_dyn import pycasadi_kin_dyn as cas_kin_dyn
from horizon_gui.custom_widgets.generic_display_mask import GenericDisplayMask
from PyQt5.QtWidgets import QFileDialog, QApplication, QGridLayout, QLabel, QLineEdit
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtGui import QPalette
import sys
import ntpath


class ModelGui(GenericDisplayMask):
    modelLoaded = pyqtSignal(bool)

    def __init__(self, logger=None, parent=None):
        super().__init__(parent)

        self.logger = logger
        self.kindyn = None

    def openWindow(self):

        super().openWindow()

        self.main_widget_layout = QGridLayout()
        self.main_widget.setLayout(self.main_widget_layout)

        self.populateWindow()

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

        if self.kindyn:
            self.nq_line.setText(str(self.kindyn.nq()))
            self.nv_line.setText(str(self.kindyn.nv()))

        self.no_button.clicked.connect(self.win.close)
        self.yes_button.clicked.connect(self.openFileFromDialog)

    def openFileFromDialog(self):

        dir = '/home/francesco/hhcm_workspace/src/horizon/horizon'
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

        self.nq_line.setText(str(self.kindyn.nq()))
        self.nv_line.setText(str(self.kindyn.nv()))
        # self.writeInStatusBar('Opening Horizon problem!')

    def loadModel(self, urdf_file):
        try:
            urdf = open(urdf_file, 'r').read()
            self.kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

            # should this go here?
            self.display.setText(ntpath.basename(urdf_file))
            self.display.setReady(True)

        except Exception as e:
            if self.logger:
                self.logger.warning(f'Could not load model in file {urdf_file}: {e}')

    def getKinDin(self):
        return self.kindyn


if __name__ == '__main__':

    app = QApplication(sys.argv)
    gui = ModelGui()
    gui.show()

    sys.exit(app.exec_())
