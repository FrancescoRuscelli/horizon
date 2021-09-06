from PyQt5.QtWidgets import QFrame

class SeparationLine(QFrame):
    def __init__(self, parent=None):
        super(SeparationLine, self).__init__(parent)
        self.setFrameShape(QFrame.HLine)
        self.setFrameShadow(QFrame.Sunken)

