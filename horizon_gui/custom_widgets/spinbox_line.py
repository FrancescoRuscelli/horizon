from PyQt5.QtWidgets import QApplication, QSpinBox, QDoubleSpinBox, QVBoxLayout, QWidget, QHBoxLayout
from PyQt5.QtGui import QKeyEvent, QValidator, QDoubleValidator, QIntValidator, QRegExpValidator, QRegularExpressionValidator
from PyQt5.QtCore import Qt, QRegExp

import numpy as np

class MyValidator(QDoubleValidator):
    def __init__(self, widget):
        super().__init__(self, widget)
        self.widget = widget
        self.var_list = list()
        self.validator = QRegExpValidator(QRegExp("inf"), self)

    def validate(self, text, pos):
        return self.validator.validate(text, pos)

        # for elem in self.regexp_list:
        #     QRegExpValidator.setRegExp(self, QRegExp(elem))
        #     valid = QRegExpValidator.validate(self, text, pos)
        #
        #     if valid[0] != QValidator.Invalid:
        #         break


        # print(text)
        # print(valid)
        # if valid[0] == QValidator.Intermediate:
        #     print('acWidget.validate result of fixup', text)

        return valid

class InfinitySpinBox(QDoubleSpinBox):
    def __init__(self, *args):
        QSpinBox.__init__(self, *args)
        validator = QRegExpValidator(QRegExp(r"[-+]?inf|\d*\,\d+|\d+"), self)
        self.lineEdit().setValidator(validator)



        self.setMinimum(-np.inf)
        self.setMaximum(np.inf)

        # self.lineEdit().setStyleSheet('color: black; background-color: white;')

        # self.valueChanged[int].connect(self.removeSelectionText)

    def keyPressEvent(self, e: QKeyEvent):

        if e.key() == Qt.Key_Home:
            self.setValue(self.maximum())
        elif e.key() == Qt.Key_End:
            self.setValue(self.minimum())
        else:
            super(QDoubleSpinBox, self).keyPressEvent(e)

    def stepBy(self, value):
        QSpinBox.stepBy(self, value)
        self.lineEdit().deselect()


        # return "%04d" % value
    # def removeSelectionText(self):
    #     print('ciao')
    #     a = self.findChild(QLineEdit)
    #     a.deselect()

class SpinBoxLine(QWidget):
    def __init__(self, nodes):
        super().__init__()

        self.layout = QHBoxLayout(self)

        self.spin_list = list()
        for i in range(nodes):
            temp_spin = InfinitySpinBox()
            # self.spin_list.append(temp_spin)
            self.layout.addWidget(temp_spin)








if (__name__ == "__main__"):
    import sys

    app = QApplication(sys.argv)

    infbox = SpinBoxLine(20)
    # hslider.setEmitWhileMoving(True)
    infbox.show()
    sys.exit(app.exec_())