from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import pyqtSignal, pyqtSlot
from horizon_gui.gui.starting_screen_ui import Ui_StartingScreen

class StartingScreen(QWidget, Ui_StartingScreen):
    start_sig = pyqtSignal()

    def __init__(self):
        super(StartingScreen, self).__init__()

        self.setupUi(self)
        self.setup_button.clicked.connect(self.on_start_sig)

    @pyqtSlot()
    def on_start_sig(self):
        self.start_sig.emit()

