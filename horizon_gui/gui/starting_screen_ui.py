# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui/starting_screen.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_StartingScreen(object):
    def setupUi(self, StartingScreen):
        StartingScreen.setObjectName("StartingScreen")
        StartingScreen.resize(1305, 926)
        self.verticalLayoutWidget = QtWidgets.QWidget(StartingScreen)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(450, 400, 333, 108))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.setup_label = QtWidgets.QLabel(self.verticalLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.setup_label.sizePolicy().hasHeightForWidth())
        self.setup_label.setSizePolicy(sizePolicy)
        self.setup_label.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.setup_label.setAlignment(QtCore.Qt.AlignCenter)
        self.setup_label.setObjectName("setup_label")
        self.verticalLayout.addWidget(self.setup_label)
        self.setup_button = QtWidgets.QPushButton(self.verticalLayoutWidget)
        self.setup_button.setObjectName("setup_button")
        self.verticalLayout.addWidget(self.setup_button)

        self.retranslateUi(StartingScreen)
        QtCore.QMetaObject.connectSlotsByName(StartingScreen)

    def retranslateUi(self, StartingScreen):
        _translate = QtCore.QCoreApplication.translate
        StartingScreen.setWindowTitle(_translate("StartingScreen", "Setup"))
        self.setup_label.setText(_translate("StartingScreen", "Click to start the Horizon Problem"))
        self.setup_button.setText(_translate("StartingScreen", "Start!"))
