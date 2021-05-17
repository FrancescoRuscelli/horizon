import sys
from functools import partial

from PyQt5.QtWidgets import QApplication, QSpinBox, QDoubleSpinBox, QLineEdit, QPushButton, QVBoxLayout
from PyQt5.QtGui import QKeyEvent, QValidator, QIntValidator, QDoubleValidator, QRegExpValidator, QRegularExpressionValidator
from PyQt5.QtCore import Qt, QRegExp

from widget1_ui import Ui_Form
from custom_functions import highlighter
from custom_widgets import horizon_line, line_edit, on_destroy_signal_window
import numpy as np
import re

class MyValidator(QRegExpValidator):
    def __init__(self, widget):
        QRegExpValidator.__init__(self, widget)
        self.widget = widget
        self.var_list = list()
        self.regexp_list = list()

        self.ignore_text = ''

    def validate(self, text, pos):

        for elem in self.regexp_list:
            QRegExpValidator.setRegExp(self, QRegExp(elem))
            valid = QRegExpValidator.validate(self, text, pos)
            # check if variable is complete
            # if (text != '' and text[-1] == ']') or text in self.var_list:
            #     temp_text = text.replace('[', '\[')
            #     temp_text = temp_text.replace(']', '\]')
            #     temp_regex = '(' + temp_text + ')' + '\+'
            #     print('adding regexp:', temp_regex)
            #     self.addRegex(temp_regex)

            if valid[0] != QValidator.Invalid:
                break


        # print(text)
        # print(valid)
        # if valid[0] == QValidator.Intermediate:
        #     print('acWidget.validate result of fixup', text)
        print(valid)
        return valid

    # def fixup(self, text):
    #     text = '1'
    #     print('diocane')
    #     self.widget.setText('1')
    #     print('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
    # return text
    def addVar(self, var):
        self.var_list.append(var)
        self.regexp_list.append(var + '(\[[0-9]:[0-9]\])')

    def addRegex(self, re):
        self.regexp_list.append(re)

class ValidatorLine(QLineEdit):
    def __init__(self):
        QLineEdit.__init__(self)

        self.keywords = ['x', 'y', 'culo'] #'|'.join(self.keywords)
        # self.layout = QVBoxLayout(self)
        operators = ['\+', '\-', '\*', '\/']
        # todo put also power

        self.setMinimumWidth(300)
        self.validator = MyValidator(self)

        # alone variables
        var_regex = '(' + '|'.join(self.keywords) + ')'
        # variables with slices
        var_slice_regex = '(' + '|'.join(self.keywords) + ')' + '(\[\d+:\d+\])'
        # all the variables (var_regex + var_slice_regex)
        all_var_regex = '(' + var_regex + '|' + var_slice_regex + ')'
        # operators regex
        operator_regex = '(' + '|'.join(operators) + ')'

        # connect everything in the form of an equation
        myregex = '(' + all_var_regex + operator_regex + ')' + '+'

        self.validator.addRegex(myregex)

        # for key in self.keywords:
        #     self.validator.addVar(key)

        self.setValidator(self.validator)


        # self.buttonValidate = QPushButton(self)

        # self.buttonValidate.clicked.connect(self.validate)

    # def validate(self):
    #     print(self.text())
    #     print(self.validator.validate(self.text(), 0))



class InfinitySpinBox(QDoubleSpinBox):
    def __init__(self, *args):
        QSpinBox.__init__(self, *args)
        # self.lineEdit().setEnabled(False)

        self.setMinimum(-np.inf)
        self.setMaximum(np.inf)

        # self.lineEdit().setStyleSheet('color: black; background-color: white;')

        # self.valueChanged[int].connect(self.removeSelectionText)

    # def textFromValue(self, value):
    #     if value == 0:
    #         return 'present'
    #     else:
    #         return str(value)

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

if (__name__ == "__main__"):
    import sys

    app = QApplication(sys.argv)

    infbox = ValidatorLine()
    # hslider.setEmitWhileMoving(True)
    infbox.show()
    sys.exit(app.exec_())