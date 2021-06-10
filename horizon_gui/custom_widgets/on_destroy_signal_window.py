from PyQt5.QtWidgets import QLabel, QVBoxLayout, QWidget
from PyQt5.QtCore import Qt, pyqtSignal

class DestroySignalWindow(QWidget):
    returnDestroyed = pyqtSignal()

    def __init__(self):
        super().__init__()

        self.setAttribute(Qt.WA_DeleteOnClose, False)

    def closeEvent(self, event):
        self.returnDestroyed.emit()
        event.accept()

    def destroyCompletely(self):
        self.deleteLater()
