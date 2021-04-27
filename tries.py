#!/usr/bin/env python
#-*- coding:utf-8 -*-

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

class widgetB(QWidget):
    procDone = pyqtSignal(str)

    def __init__(self, parent=None):
        super(widgetB, self).__init__(parent)

        self.lineEdit = QLineEdit(self)
        self.button = QPushButton("Send Message to A", self)
        self.layout = QHBoxLayout(self)
        self.layout.addWidget(self.lineEdit)
        self.layout.addWidget(self.button)

        self.button.clicked.connect(self.on_button_clicked)

    @pyqtSlot()
    def on_button_clicked(self):
        self.procDone.emit(self.lineEdit.text())

    @pyqtSlot(str)
    def on_procStart(self, message):
        self.lineEdit.setText("From A: " + message)

        self.raise_()

class widgetA(QWidget):
    procStart = pyqtSignal(str)

    def __init__(self, parent=None):
        super(widgetA, self).__init__(parent)

        self.lineEdit = QLineEdit(self)
        self.lineEdit.setText("Hello!")

        self.button = QPushButton("Send Message to B", self)
        self.button.clicked.connect(self.on_button_clicked)

        self.layout = QHBoxLayout(self)
        self.layout.addWidget(self.lineEdit)
        self.layout.addWidget(self.button)

    @pyqtSlot()
    def on_button_clicked(self):
        self.procStart.emit(self.lineEdit.text())

    @pyqtSlot(str)
    def on_widgetB_procDone(self, message):
        self.lineEdit.setText("From B: " + message)

        self.raise_()


class mainwindow(QMainWindow):
    def __init__(self, parent=None):
        super(mainwindow, self).__init__(parent)

        self.button = QPushButton("Click Me", self)
        self.button.clicked.connect(self.on_button_clicked)

        self.setCentralWidget(self.button)

        self.widgetA = widgetA()
        self.widgetB = widgetB()

        self.widgetA.procStart.connect(self.widgetB.on_procStart)
        self.widgetB.procDone.connect(self.widgetA.on_widgetB_procDone)

    @pyqtSlot()
    def on_button_clicked(self):
        self.widgetA.show()
        self.widgetB.show()

        self.widgetA.raise_()


if __name__ == "__main__":
    import sys

    app = QApplication(sys.argv)
    main = mainwindow()
    main.show()
    sys.exit(app.exec_())