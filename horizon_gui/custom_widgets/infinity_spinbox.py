from PyQt5.QtWidgets import QApplication, QSpinBox, QDoubleSpinBox
from PyQt5.QtGui import QKeyEvent, QRegExpValidator, QFont, QPalette
from PyQt5.QtCore import Qt, QRegExp, QLocale

import numpy as np

class InfinitySpinBox(QDoubleSpinBox):
    def __init__(self, parent=None, color_base="MediumSeaGreen", color_selected="Crimson"):
        QSpinBox.__init__(self, parent)

        locale = QLocale(QLocale.English)
        self.setLocale(locale)

        self.setDecimals(1)

        self.color_base = color_base
        self.color_selected = color_selected

        palette = QPalette()
        palette.setColor(QPalette.Text, Qt.red)
        self.lineEdit().setPalette(palette)

        font = QFont("Times", 12, QFont.Bold)
        self.lineEdit().setFont(font)


        validator = QRegExpValidator(QRegExp(r"[-+]?\d*[\.,\,]\d+|\d+|[-+]?inf"), self)
        self.lineEdit().setValidator(validator)

        self.selected = False

        self.setStyleSheet("background-color: {}".format(color_base))

        self.setMinimum(-np.inf)
        self.setMaximum(np.inf)

    def keyPressEvent(self, e: QKeyEvent):

        if e.key() == Qt.Key_Home:
            self.setValue(self.maximum())
        elif e.key() == Qt.Key_End:
            self.setValue(self.minimum())
        else:
            super(QDoubleSpinBox, self).keyPressEvent(e)

    def select(self, flag):
        self.selected = flag
        if flag is False:
            self.lineEdit().deselect()
            self.setStyleSheet("background-color: {}".format(self.color_base))
        else:
            self.lineEdit().selectAll()
            self.setStyleSheet("background-color: {}".format(self.color_selected))


    def isSelected(self):
        return self.selected

if (__name__ == "__main__"):
    import sys

    app = QApplication(sys.argv)

    infbox = InfinitySpinBox()
    infbox.show()
    sys.exit(app.exec_())