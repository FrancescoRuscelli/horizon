# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui/widget1.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_HorizonGUI(object):
    def setupUi(self, HorizonGUI):
        HorizonGUI.setObjectName("HorizonGUI")
        HorizonGUI.resize(1485, 804)
        HorizonGUI.setMouseTracking(False)
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(HorizonGUI)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.splitter = QtWidgets.QSplitter(HorizonGUI)
        self.splitter.setOrientation(QtCore.Qt.Vertical)
        self.splitter.setObjectName("splitter")
        self.layoutWidget = QtWidgets.QWidget(self.splitter)
        self.layoutWidget.setObjectName("layoutWidget")
        self.gridLayout_4 = QtWidgets.QGridLayout(self.layoutWidget)
        self.gridLayout_4.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.funTable = BoxFunction(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.funTable.sizePolicy().hasHeightForWidth())
        self.funTable.setSizePolicy(sizePolicy)
        self.funTable.setObjectName("funTable")
        self.funTable.setColumnCount(2)
        self.funTable.setRowCount(0)
        item = QtWidgets.QTableWidgetItem()
        self.funTable.setHorizontalHeaderItem(0, item)
        item = QtWidgets.QTableWidgetItem()
        self.funTable.setHorizontalHeaderItem(1, item)
        self.gridLayout_4.addWidget(self.funTable, 0, 2, 1, 1)
        self.verticalLayout_4 = QtWidgets.QVBoxLayout()
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.DynamicsBox = QtWidgets.QGroupBox(self.layoutWidget)
        self.DynamicsBox.setAlignment(QtCore.Qt.AlignCenter)
        self.DynamicsBox.setObjectName("DynamicsBox")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.DynamicsBox)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.dynDisplay = DisplayLine(self.DynamicsBox)
        self.dynDisplay.setEnabled(False)
        self.dynDisplay.setObjectName("dynDisplay")
        self.horizontalLayout_2.addWidget(self.dynDisplay)
        self.dynButton = QtWidgets.QPushButton(self.DynamicsBox)
        self.dynButton.setObjectName("dynButton")
        self.horizontalLayout_2.addWidget(self.dynButton)
        self.verticalLayout_4.addWidget(self.DynamicsBox)
        self.TranscriptionBox = QtWidgets.QGroupBox(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.TranscriptionBox.sizePolicy().hasHeightForWidth())
        self.TranscriptionBox.setSizePolicy(sizePolicy)
        self.TranscriptionBox.setAlignment(QtCore.Qt.AlignCenter)
        self.TranscriptionBox.setObjectName("TranscriptionBox")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.TranscriptionBox)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.transcriptionDisplay = DisplayLine(self.TranscriptionBox)
        self.transcriptionDisplay.setEnabled(False)
        self.transcriptionDisplay.setReadOnly(True)
        self.transcriptionDisplay.setClearButtonEnabled(False)
        self.transcriptionDisplay.setObjectName("transcriptionDisplay")
        self.horizontalLayout.addWidget(self.transcriptionDisplay)
        self.transButton = QtWidgets.QPushButton(self.TranscriptionBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.transButton.sizePolicy().hasHeightForWidth())
        self.transButton.setSizePolicy(sizePolicy)
        self.transButton.setObjectName("transButton")
        self.horizontalLayout.addWidget(self.transButton)
        self.verticalLayout_4.addWidget(self.TranscriptionBox)
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_4.addItem(spacerItem)
        self.gridLayout_4.addLayout(self.verticalLayout_4, 0, 3, 1, 1)
        self.funBox = QtWidgets.QGroupBox(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.funBox.sizePolicy().hasHeightForWidth())
        self.funBox.setSizePolicy(sizePolicy)
        self.funBox.setMinimumSize(QtCore.QSize(0, 131))
        self.funBox.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.funBox.setAlignment(QtCore.Qt.AlignCenter)
        self.funBox.setObjectName("funBox")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.funBox)
        self.gridLayout_3.setSizeConstraint(QtWidgets.QLayout.SetMinAndMaxSize)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.funDefaultButton = QtWidgets.QPushButton(self.funBox)
        self.funDefaultButton.setObjectName("funDefaultButton")
        self.gridLayout_3.addWidget(self.funDefaultButton, 7, 0, 1, 2)
        self.funNameInput = QtWidgets.QLineEdit(self.funBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.funNameInput.sizePolicy().hasHeightForWidth())
        self.funNameInput.setSizePolicy(sizePolicy)
        self.funNameInput.setObjectName("funNameInput")
        self.gridLayout_3.addWidget(self.funNameInput, 1, 1, 1, 1)
        self.funLabel = QtWidgets.QLabel(self.funBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.funLabel.sizePolicy().hasHeightForWidth())
        self.funLabel.setSizePolicy(sizePolicy)
        self.funLabel.setObjectName("funLabel")
        self.gridLayout_3.addWidget(self.funLabel, 2, 0, 1, 1)
        self.line_3 = QtWidgets.QFrame(self.funBox)
        self.line_3.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_3.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_3.setObjectName("line_3")
        self.gridLayout_3.addWidget(self.line_3, 4, 0, 1, 2)
        self.funComboBox = QtWidgets.QComboBox(self.funBox)
        self.funComboBox.setObjectName("funComboBox")
        self.gridLayout_3.addWidget(self.funComboBox, 6, 0, 1, 2)
        self.funNameLabel = QtWidgets.QLabel(self.funBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.funNameLabel.sizePolicy().hasHeightForWidth())
        self.funNameLabel.setSizePolicy(sizePolicy)
        self.funNameLabel.setObjectName("funNameLabel")
        self.gridLayout_3.addWidget(self.funNameLabel, 1, 0, 1, 1)
        self.label_4 = QtWidgets.QLabel(self.funBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_4.sizePolicy().hasHeightForWidth())
        self.label_4.setSizePolicy(sizePolicy)
        self.label_4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_4.setObjectName("label_4")
        self.gridLayout_3.addWidget(self.label_4, 0, 0, 1, 2)
        self.label_5 = QtWidgets.QLabel(self.funBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_5.sizePolicy().hasHeightForWidth())
        self.label_5.setSizePolicy(sizePolicy)
        self.label_5.setAlignment(QtCore.Qt.AlignCenter)
        self.label_5.setObjectName("label_5")
        self.gridLayout_3.addWidget(self.label_5, 5, 0, 1, 2)
        spacerItem1 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.gridLayout_3.addItem(spacerItem1, 8, 0, 1, 1)
        self.funCustomButton = QtWidgets.QPushButton(self.funBox)
        self.funCustomButton.setObjectName("funCustomButton")
        self.gridLayout_3.addWidget(self.funCustomButton, 3, 1, 1, 1)
        self.funInput = LineEdit(self.funBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.funInput.sizePolicy().hasHeightForWidth())
        self.funInput.setSizePolicy(sizePolicy)
        self.funInput.setObjectName("funInput")
        self.gridLayout_3.addWidget(self.funInput, 2, 1, 1, 1)
        self.gridLayout_4.addWidget(self.funBox, 0, 1, 1, 1)
        self.varBox = QtWidgets.QGroupBox(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.varBox.sizePolicy().hasHeightForWidth())
        self.varBox.setSizePolicy(sizePolicy)
        self.varBox.setMinimumSize(QtCore.QSize(0, 131))
        self.varBox.setAlignment(QtCore.Qt.AlignCenter)
        self.varBox.setFlat(False)
        self.varBox.setCheckable(False)
        self.varBox.setObjectName("varBox")
        self.gridLayout = QtWidgets.QGridLayout(self.varBox)
        self.gridLayout.setObjectName("gridLayout")
        self.stateVarAddButton = QtWidgets.QPushButton(self.varBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.stateVarAddButton.sizePolicy().hasHeightForWidth())
        self.stateVarAddButton.setSizePolicy(sizePolicy)
        self.stateVarAddButton.setObjectName("stateVarAddButton")
        self.gridLayout.addWidget(self.stateVarAddButton, 4, 2, 1, 1)
        self.varOptionLayout = QtWidgets.QHBoxLayout()
        self.varOptionLayout.setObjectName("varOptionLayout")
        self.varDim = QtWidgets.QLabel(self.varBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.varDim.sizePolicy().hasHeightForWidth())
        self.varDim.setSizePolicy(sizePolicy)
        self.varDim.setObjectName("varDim")
        self.varOptionLayout.addWidget(self.varDim)
        self.varDimInput = QtWidgets.QSpinBox(self.varBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.varDimInput.sizePolicy().hasHeightForWidth())
        self.varDimInput.setSizePolicy(sizePolicy)
        self.varDimInput.setMinimum(1)
        self.varDimInput.setObjectName("varDimInput")
        self.varOptionLayout.addWidget(self.varDimInput)
        self.varOffset = QtWidgets.QLabel(self.varBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.varOffset.sizePolicy().hasHeightForWidth())
        self.varOffset.setSizePolicy(sizePolicy)
        self.varOffset.setObjectName("varOffset")
        self.varOptionLayout.addWidget(self.varOffset)
        self.varOffsetInput = NodesSpinBox(self.varBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.varOffsetInput.sizePolicy().hasHeightForWidth())
        self.varOffsetInput.setSizePolicy(sizePolicy)
        palette = QtGui.QPalette()
        brush = QtGui.QBrush(QtGui.QColor(87, 49, 198, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Highlight, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.HighlightedText, brush)
        brush = QtGui.QBrush(QtGui.QColor(87, 49, 198, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Highlight, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.HighlightedText, brush)
        brush = QtGui.QBrush(QtGui.QColor(145, 141, 126))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Highlight, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.HighlightedText, brush)
        self.varOffsetInput.setPalette(palette)
        self.varOffsetInput.setMaximum(102)
        self.varOffsetInput.setProperty("value", 0)
        self.varOffsetInput.setObjectName("varOffsetInput")
        self.varOptionLayout.addWidget(self.varOffsetInput)
        self.gridLayout.addLayout(self.varOptionLayout, 1, 2, 1, 1)
        self.singleVarAddButton = QtWidgets.QPushButton(self.varBox)
        self.singleVarAddButton.setObjectName("singleVarAddButton")
        self.gridLayout.addWidget(self.singleVarAddButton, 6, 2, 1, 1)
        self.varTable = BoxStateVar(self.varBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.varTable.sizePolicy().hasHeightForWidth())
        self.varTable.setSizePolicy(sizePolicy)
        self.varTable.setObjectName("varTable")
        self.varTable.setColumnCount(3)
        self.varTable.setRowCount(0)
        item = QtWidgets.QTableWidgetItem()
        self.varTable.setHorizontalHeaderItem(0, item)
        item = QtWidgets.QTableWidgetItem()
        self.varTable.setHorizontalHeaderItem(1, item)
        item = QtWidgets.QTableWidgetItem()
        self.varTable.setHorizontalHeaderItem(2, item)
        self.varTable.horizontalHeader().setDefaultSectionSize(57)
        self.varTable.horizontalHeader().setMinimumSectionSize(20)
        self.varTable.horizontalHeader().setSortIndicatorShown(False)
        self.varTable.verticalHeader().setDefaultSectionSize(21)
        self.gridLayout.addWidget(self.varTable, 0, 6, 12, 1)
        self.inputVarAddButton = QtWidgets.QPushButton(self.varBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.inputVarAddButton.sizePolicy().hasHeightForWidth())
        self.inputVarAddButton.setSizePolicy(sizePolicy)
        self.inputVarAddButton.setObjectName("inputVarAddButton")
        self.gridLayout.addWidget(self.inputVarAddButton, 5, 2, 1, 1)
        spacerItem2 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.gridLayout.addItem(spacerItem2, 9, 2, 1, 1)
        self.customVarAddButton = QtWidgets.QPushButton(self.varBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.customVarAddButton.sizePolicy().hasHeightForWidth())
        self.customVarAddButton.setSizePolicy(sizePolicy)
        self.customVarAddButton.setObjectName("customVarAddButton")
        self.gridLayout.addWidget(self.customVarAddButton, 7, 2, 1, 1)
        self.varNameLayout = QtWidgets.QHBoxLayout()
        self.varNameLayout.setObjectName("varNameLayout")
        self.SVName = QtWidgets.QLabel(self.varBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.SVName.sizePolicy().hasHeightForWidth())
        self.SVName.setSizePolicy(sizePolicy)
        self.SVName.setObjectName("SVName")
        self.varNameLayout.addWidget(self.SVName)
        self.varNameInput = QtWidgets.QLineEdit(self.varBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.varNameInput.sizePolicy().hasHeightForWidth())
        self.varNameInput.setSizePolicy(sizePolicy)
        self.varNameInput.setObjectName("varNameInput")
        self.varNameLayout.addWidget(self.varNameInput)
        self.gridLayout.addLayout(self.varNameLayout, 0, 2, 1, 1)
        self.gridLayout_4.addWidget(self.varBox, 0, 0, 1, 1)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.SolverBox = QtWidgets.QGroupBox(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.SolverBox.sizePolicy().hasHeightForWidth())
        self.SolverBox.setSizePolicy(sizePolicy)
        self.SolverBox.setAlignment(QtCore.Qt.AlignCenter)
        self.SolverBox.setObjectName("SolverBox")
        self.gridLayout_5 = QtWidgets.QGridLayout(self.SolverBox)
        self.gridLayout_5.setObjectName("gridLayout_5")
        self.ledCreate = LedIndicator(self.SolverBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.ledCreate.sizePolicy().hasHeightForWidth())
        self.ledCreate.setSizePolicy(sizePolicy)
        self.ledCreate.setObjectName("ledCreate")
        self.gridLayout_5.addWidget(self.ledCreate, 3, 1, 1, 1)
        self.ledSolve = LedIndicator(self.SolverBox)
        self.ledSolve.setObjectName("ledSolve")
        self.gridLayout_5.addWidget(self.ledSolve, 4, 1, 1, 1)
        self.SolveButton = QtWidgets.QPushButton(self.SolverBox)
        self.SolveButton.setEnabled(False)
        self.SolveButton.setObjectName("SolveButton")
        self.gridLayout_5.addWidget(self.SolveButton, 4, 0, 1, 1)
        self.CreateButton = QtWidgets.QPushButton(self.SolverBox)
        self.CreateButton.setObjectName("CreateButton")
        self.gridLayout_5.addWidget(self.CreateButton, 3, 0, 1, 1)
        self.label_3 = QtWidgets.QLabel(self.SolverBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_3.sizePolicy().hasHeightForWidth())
        self.label_3.setSizePolicy(sizePolicy)
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName("label_3")
        self.gridLayout_5.addWidget(self.label_3, 1, 1, 1, 1)
        self.PlotButton = QtWidgets.QPushButton(self.SolverBox)
        self.PlotButton.setEnabled(False)
        self.PlotButton.setFlat(False)
        self.PlotButton.setObjectName("PlotButton")
        self.gridLayout_5.addWidget(self.PlotButton, 5, 0, 1, 1)
        self.SolverOptionMenu = QtWidgets.QComboBox(self.SolverBox)
        self.SolverOptionMenu.setObjectName("SolverOptionMenu")
        self.SolverOptionMenu.addItem("")
        self.SolverOptionMenu.addItem("")
        self.SolverOptionMenu.addItem("")
        self.SolverOptionMenu.addItem("")
        self.SolverOptionMenu.addItem("")
        self.gridLayout_5.addWidget(self.SolverOptionMenu, 1, 0, 1, 1)
        self.line = QtWidgets.QFrame(self.SolverBox)
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.gridLayout_5.addWidget(self.line, 2, 0, 1, 1)
        self.label_2 = QtWidgets.QLabel(self.SolverBox)
        self.label_2.setObjectName("label_2")
        self.gridLayout_5.addWidget(self.label_2, 0, 0, 1, 1)
        self.verticalLayout_2.addWidget(self.SolverBox)
        spacerItem3 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_2.addItem(spacerItem3)
        self.groupBox = QtWidgets.QGroupBox(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.groupBox.sizePolicy().hasHeightForWidth())
        self.groupBox.setSizePolicy(sizePolicy)
        self.groupBox.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox.setObjectName("groupBox")
        self.gridLayout_6 = QtWidgets.QGridLayout(self.groupBox)
        self.gridLayout_6.setObjectName("gridLayout_6")
        self.NodesLabel = QtWidgets.QLabel(self.groupBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.NodesLabel.sizePolicy().hasHeightForWidth())
        self.NodesLabel.setSizePolicy(sizePolicy)
        self.NodesLabel.setMinimumSize(QtCore.QSize(138, 17))
        self.NodesLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.NodesLabel.setObjectName("NodesLabel")
        self.gridLayout_6.addWidget(self.NodesLabel, 0, 0, 1, 1)
        self.NodesSpinBox = QtWidgets.QSpinBox(self.groupBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.NodesSpinBox.sizePolicy().hasHeightForWidth())
        self.NodesSpinBox.setSizePolicy(sizePolicy)
        self.NodesSpinBox.setMinimumSize(QtCore.QSize(0, 26))
        self.NodesSpinBox.setAlignment(QtCore.Qt.AlignCenter)
        self.NodesSpinBox.setObjectName("NodesSpinBox")
        self.gridLayout_6.addWidget(self.NodesSpinBox, 0, 1, 1, 1)
        self.verticalLayout_2.addWidget(self.groupBox)
        self.gridLayout_4.addLayout(self.verticalLayout_2, 0, 5, 1, 1)
        self.PRB = QtWidgets.QGroupBox(self.splitter)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.PRB.sizePolicy().hasHeightForWidth())
        self.PRB.setSizePolicy(sizePolicy)
        self.PRB.setObjectName("PRB")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.PRB)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.LineView = QtWidgets.QHBoxLayout()
        self.LineView.setObjectName("LineView")
        self.label = QtWidgets.QLabel(self.PRB)
        self.label.setObjectName("label")
        self.LineView.addWidget(self.label)
        self.MultipleLineButton = QtWidgets.QRadioButton(self.PRB)
        self.MultipleLineButton.setChecked(True)
        self.MultipleLineButton.setObjectName("MultipleLineButton")
        self.LineView.addWidget(self.MultipleLineButton)
        self.SingleLineButton = QtWidgets.QRadioButton(self.PRB)
        self.SingleLineButton.setChecked(False)
        self.SingleLineButton.setObjectName("SingleLineButton")
        self.LineView.addWidget(self.SingleLineButton)
        self.gridLayout_2.addLayout(self.LineView, 2, 2, 1, 1)
        spacerItem4 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_2.addItem(spacerItem4, 2, 0, 1, 1)
        self.switchPageButton = QtWidgets.QPushButton(self.PRB)
        self.switchPageButton.setObjectName("switchPageButton")
        self.gridLayout_2.addWidget(self.switchPageButton, 2, 1, 1, 1)
        self.ProblemMain = QtWidgets.QStackedWidget(self.PRB)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.ProblemMain.sizePolicy().hasHeightForWidth())
        self.ProblemMain.setSizePolicy(sizePolicy)
        self.ProblemMain.setObjectName("ProblemMain")
        self.CTPage = QtWidgets.QWidget()
        self.CTPage.setObjectName("CTPage")
        self.ProblemMain.addWidget(self.CTPage)
        self.CFPage = QtWidgets.QWidget()
        self.CFPage.setObjectName("CFPage")
        self.ProblemMain.addWidget(self.CFPage)
        self.gridLayout_2.addWidget(self.ProblemMain, 0, 0, 1, 3)
        self.consoleLogger = ConsoleLogger(self.splitter)
        self.consoleLogger.setEnabled(True)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.consoleLogger.sizePolicy().hasHeightForWidth())
        self.consoleLogger.setSizePolicy(sizePolicy)
        self.consoleLogger.setMaximumSize(QtCore.QSize(16777215, 200))
        font = QtGui.QFont()
        font.setFamily("Monospace")
        self.consoleLogger.setFont(font)
        self.consoleLogger.setStyleSheet("QFrame {background-color: rgb(46, 52, 54)}\n"
"QTextEdit {color: rgb(238, 238, 236)}")
        self.consoleLogger.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.consoleLogger.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.consoleLogger.setSizeAdjustPolicy(QtWidgets.QAbstractScrollArea.AdjustToContents)
        self.consoleLogger.setReadOnly(True)
        self.consoleLogger.setObjectName("consoleLogger")
        self.verticalLayout_3.addWidget(self.splitter)
        self.actionExit = QtWidgets.QAction(HorizonGUI)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("ui/../../../../../../../../.designer/resources/rocket.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionExit.setIcon(icon)
        self.actionExit.setObjectName("actionExit")
        self.actionAbout = QtWidgets.QAction(HorizonGUI)
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap("ui/../../../../../../../../.designer/resources/info.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionAbout.setIcon(icon1)
        self.actionAbout.setObjectName("actionAbout")

        self.retranslateUi(HorizonGUI)
        self.ProblemMain.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(HorizonGUI)

    def retranslateUi(self, HorizonGUI):
        _translate = QtCore.QCoreApplication.translate
        HorizonGUI.setWindowTitle(_translate("HorizonGUI", "Form"))
        item = self.funTable.horizontalHeaderItem(0)
        item.setText(_translate("HorizonGUI", "Name"))
        item = self.funTable.horizontalHeaderItem(1)
        item.setText(_translate("HorizonGUI", "Function"))
        self.DynamicsBox.setTitle(_translate("HorizonGUI", "Dynamics"))
        self.dynButton.setText(_translate("HorizonGUI", "..."))
        self.TranscriptionBox.setTitle(_translate("HorizonGUI", "Transcriptions"))
        self.transButton.setText(_translate("HorizonGUI", "..."))
        self.funBox.setTitle(_translate("HorizonGUI", "Functions"))
        self.funDefaultButton.setText(_translate("HorizonGUI", "Add Default"))
        self.funLabel.setText(_translate("HorizonGUI", "function:"))
        self.funNameLabel.setText(_translate("HorizonGUI", "name:"))
        self.label_4.setText(_translate("HorizonGUI", "Custom:"))
        self.label_5.setText(_translate("HorizonGUI", "Default:"))
        self.funCustomButton.setText(_translate("HorizonGUI", "Add Custom"))
        self.varBox.setTitle(_translate("HorizonGUI", "Variables"))
        self.stateVarAddButton.setText(_translate("HorizonGUI", "Add State"))
        self.varDim.setText(_translate("HorizonGUI", "Dim:"))
        self.varOffset.setText(_translate("HorizonGUI", "Offset:"))
        self.singleVarAddButton.setText(_translate("HorizonGUI", "Add Single"))
        item = self.varTable.horizontalHeaderItem(0)
        item.setText(_translate("HorizonGUI", "Var"))
        item = self.varTable.horizontalHeaderItem(1)
        item.setText(_translate("HorizonGUI", "Dim"))
        item = self.varTable.horizontalHeaderItem(2)
        item.setText(_translate("HorizonGUI", "Type"))
        self.inputVarAddButton.setText(_translate("HorizonGUI", "Add Input"))
        self.customVarAddButton.setText(_translate("HorizonGUI", "Add Custom"))
        self.SVName.setText(_translate("HorizonGUI", "Name:"))
        self.SolverBox.setTitle(_translate("HorizonGUI", "Solver"))
        self.SolveButton.setText(_translate("HorizonGUI", "Solve!"))
        self.CreateButton.setText(_translate("HorizonGUI", "Create Problem"))
        self.label_3.setText(_translate("HorizonGUI", "updated:"))
        self.PlotButton.setText(_translate("HorizonGUI", "Plot Solution"))
        self.SolverOptionMenu.setItemText(0, _translate("HorizonGUI", "ipopt"))
        self.SolverOptionMenu.setItemText(1, _translate("HorizonGUI", "doesn\'t"))
        self.SolverOptionMenu.setItemText(2, _translate("HorizonGUI", "really"))
        self.SolverOptionMenu.setItemText(3, _translate("HorizonGUI", "works"))
        self.SolverOptionMenu.setItemText(4, _translate("HorizonGUI", "lol sorry"))
        self.label_2.setText(_translate("HorizonGUI", "Options:"))
        self.groupBox.setTitle(_translate("HorizonGUI", "Horizon Nodes"))
        self.NodesLabel.setText(_translate("HorizonGUI", "N. nodes:"))
        self.PRB.setTitle(_translate("HorizonGUI", "Problem"))
        self.label.setText(_translate("HorizonGUI", "View:"))
        self.MultipleLineButton.setText(_translate("HorizonGUI", "Multiple"))
        self.SingleLineButton.setText(_translate("HorizonGUI", "Single"))
        self.switchPageButton.setText(_translate("HorizonGUI", "Switch to Cost Functions "))
        self.actionExit.setText(_translate("HorizonGUI", "Exit"))
        self.actionExit.setToolTip(_translate("HorizonGUI", "Exit Horizon GUI"))
        self.actionExit.setShortcut(_translate("HorizonGUI", "Ctrl+E"))
        self.actionAbout.setText(_translate("HorizonGUI", "About"))
        self.actionAbout.setToolTip(_translate("HorizonGUI", "Info about Horizon GUI"))
        self.actionAbout.setShortcut(_translate("HorizonGUI", "Ctrl+I"))
from horizon_gui.custom_widgets.box_function import BoxFunction
from horizon_gui.custom_widgets.box_state_var import BoxStateVar
from horizon_gui.custom_widgets.console_logger import ConsoleLogger
from horizon_gui.custom_widgets.display_line import DisplayLine
from horizon_gui.custom_widgets.led_indicator import LedIndicator
from horizon_gui.custom_widgets.line_edit import LineEdit
from horizon_gui.custom_widgets.nodes_spinbox import NodesSpinBox
