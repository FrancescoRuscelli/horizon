from PyQt5.QtWidgets import QSpinBox, QLineEdit, QAbstractSpinBox

class NodesSpinBox(QSpinBox):
    def __init__(self, *args):
        QSpinBox.__init__(self, *args)
        # self.lineEdit().setEnabled(False)
        self.lineEdit().setReadOnly(True)

        # self.lineEdit().setStyleSheet('color: black; background-color: white;')

        # self.valueChanged[int].connect(self.removeSelectionText)

    def textFromValue(self, value):
        if value == 0:
            return 'present'
        else:
            return str(value)

    def stepBy(self, value):
        QSpinBox.stepBy(self, value)
        self.lineEdit().deselect()

        # return "%04d" % value
    # def removeSelectionText(self):
    #     print('ciao')
    #     a = self.findChild(QLineEdit)
    #     a.deselect()


