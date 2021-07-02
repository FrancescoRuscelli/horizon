from PyQt5.QtWidgets import QApplication, QSpinBox, QDoubleSpinBox, QWidget, QHBoxLayout
from PyQt5.QtGui import QKeyEvent, QRegExpValidator
from PyQt5.QtCore import Qt, QRegExp

import numpy as np

class InfinitySpinBox(QDoubleSpinBox):
    def __init__(self, *args):
        QSpinBox.__init__(self, *args)
        validator = QRegExpValidator(QRegExp(r"[-+]?inf|\d*\,\d+|\d+"), self)
        self.lineEdit().setValidator(validator)

        self.setMinimum(-np.inf)
        self.setMaximum(np.inf)

    def keyPressEvent(self, e: QKeyEvent):

        if e.key() == Qt.Key_Home:
            self.setValue(self.maximum())
        elif e.key() == Qt.Key_End:
            self.setValue(self.minimum())
        else:
            super(QDoubleSpinBox, self).keyPressEvent(e)

    # def stepBy(self, value):
    #     QSpinBox.stepBy(self, value)
    #     self.lineEdit().deselect()


if (__name__ == "__main__"):
    import sys

    app = QApplication(sys.argv)

    infbox = InfinitySpinBox()
    infbox.show()
    sys.exit(app.exec_())