import sys
from functools import partial

from PyQt5.QtWidgets import (QGridLayout, QLabel, QPushButton, QRadioButton, QVBoxLayout, QHBoxLayout, QWidget,
                             QLineEdit, QTableWidgetItem, QTableWidget, QCompleter, QHeaderView, QDialogButtonBox,
                             QComboBox, QSpinBox, QFrame, QToolBox, QTabWidget)

from PyQt5.QtGui import QPalette, QFont, QColor
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot

from horizon_gui.gui.widget1_ui import Ui_HorizonGUI
from horizon_gui.gui.txt_to_fun import TxtToFun
from horizon_gui.gui.variables_module.variables_gui import VariablesGui
from horizon_gui.gui.transcriptions_module.transcriptions_gui import TranscriptionGui
from horizon_gui.gui.dynamics_module.dynamics_gui import DynamicsGui
from horizon_gui.gui.model_module.model_gui import ModelGui
from horizon_gui.custom_functions import highlighter
from horizon_gui.custom_widgets import horizon_line, line_edit, on_destroy_signal_window, highlight_delegate, multi_slider, option_container, separation_line, bounds_line
from horizon_gui.definitions import CSS_DIR
from horizon_gui.gui.gui_receiver import horizonImpl
class MainInterface(QWidget, Ui_HorizonGUI):
    generic_sig = pyqtSignal(str)

    def __init__(self, horizon_receiver: horizonImpl, logger=None):
        super(MainInterface, self).__init__()

        self.setupUi(self)
        # #todo logger! use it everywhere!

        self.logger = logger
        self.logger.addHandler(self.consoleLogger)
        # emitting stream from python terminal
        # sys.stdout = EmittingStream()
        # sys.stdout.textWritten.connect(self.normalOutputWritten)

        # todo put it somewhere else?
        # self._init()
        # self.horizon_receiver = None
        self.horizon_receiver = horizon_receiver
        self.nodes = horizon_receiver.getNodes()

        self.fun_keywords = list()

        self.variables_gui = VariablesGui(self.horizon_receiver, self.logger, self)
        var_box_layout = QVBoxLayout()
        self.varBox.setLayout(var_box_layout)
        var_box_layout.addWidget(self.variables_gui)

        # self._connectActions()
        self.trans_gui = TranscriptionGui(self.horizon_receiver, self.logger, self)
        transcription_box_layout = QVBoxLayout()
        self.TranscriptionBox.setLayout(transcription_box_layout)
        transcription_box_layout.addWidget(self.trans_gui)
        # ===============================
        self.dyn_gui = DynamicsGui(self.horizon_receiver, self.logger, self)
        dynamics_box_layout = QVBoxLayout()
        self.DynamicsBox.setLayout(dynamics_box_layout)
        dynamics_box_layout.addWidget(self.dyn_gui)
        # ===============================
        self.model_gui = ModelGui(self.logger, self)
        model_box_layout = QVBoxLayout()
        self.ModelBox.setLayout(model_box_layout)
        model_box_layout.addWidget(self.model_gui)

        # spinbox to set offset variables
        # self.varOffsetInput.setRange(-N, 0)


        self.layout_ct = QVBoxLayout(self.CTPage)

        ct_title = QLabel('Constraints')
        ct_title.setFont(QFont("Times", 12, QFont.Bold))
        ct_title.setAlignment(Qt.AlignCenter)
        self.layout_ct.addWidget(ct_title)

        # self.constraintLine = horizon_multi_line.HorizonMultiLine(self.horizon_receiver, 'constraint', nodes=N, logger=self.logger)
        self.constraintLine = horizon_line.HorizonLine(self.horizon_receiver, 'constraint', nodes=self.nodes, logger=self.logger)
        self.constraintLine.setContentsMargins(0, 40, 0, 0)
        self.layout_ct.addWidget(self.constraintLine)

        self.layout_cf = QVBoxLayout(self.CFPage)

        cf_title = QLabel('Cost Functions')
        cf_title.setFont(QFont("Times", 12, QFont.Bold))
        cf_title.setAlignment(Qt.AlignCenter)
        self.layout_cf.addWidget(cf_title)

        self.costfunctionLine = horizon_line.HorizonLine(self.horizon_receiver, 'costfunction', nodes=self.nodes, logger=self.logger)
        self.costfunctionLine.setContentsMargins(0, 40, 0, 0)
        self.layout_cf.addWidget(self.costfunctionLine)
        self.ct_entry = self.setFunEditor(self.funBox)
        #
        # with open(CSS_DIR + '/button_old.css', 'r') as f:
        #     self.SolveButton.setStyleSheet(f.read())
        #
        # with open(CSS_DIR + '/button_old.css', 'r') as f:
        #     self.PlotButton.setStyleSheet(f.read())

        self.funCustomButton.clicked.connect(self.generateCustomFunction)
        self.funDefaultButton.clicked.connect(self.generateDefaultFunction)
        self.funTable.itemDoubleClicked.connect(self.openFunction)

        self.switchPageButton.clicked.connect(self.switchPage)
        # self.NodesSpinBox.editingFinished.connect(self.setBoxNodes)
        self.NodesSpinBox.valueChanged.connect(self.setBoxNodes)
        self.SingleLineButton.toggled.connect(partial(self.constraintLine.switchPage, self.constraintLine.Single))
        self.SingleLineButton.toggled.connect(partial(self.costfunctionLine.switchPage, self.constraintLine.Single))
        self.MultipleLineButton.toggled.connect(partial(self.constraintLine.switchPage, self.costfunctionLine.Multi))
        self.MultipleLineButton.toggled.connect(partial(self.costfunctionLine.switchPage, self.costfunctionLine.Multi))
        self.CreateButton.clicked.connect(self.createButtonPushed)
        self.SolveButton.clicked.connect(self.solveButtonPushed)
        self.PlotButton.clicked.connect(self.plotButtonPushed)


        # these set to NOT READY the create/solve problem buttons if something in the horizon problem is changed
        self.constraintLine.active_fun_horizon.connect(partial(self.ledCreate.setReady, False))
        self.constraintLine.active_fun_horizon.connect(partial(self.ledSolve.setReady, False))
        self.constraintLine.bounds_changed.connect(partial(self.ledCreate.setReady, False))
        self.constraintLine.bounds_changed.connect(partial(self.ledSolve.setReady, False))
        self.costfunctionLine.active_fun_horizon.connect(partial(self.ledCreate.setReady, False))
        self.costfunctionLine.active_fun_horizon.connect(partial(self.ledSolve.setReady, False))
        self.costfunctionLine.bounds_changed.connect(partial(self.ledCreate.setReady, False))
        self.costfunctionLine.bounds_changed.connect(partial(self.ledSolve.setReady, False))
        self.NodesSpinBox.valueChanged.connect(partial(self.ledCreate.setReady, False))
        self.NodesSpinBox.valueChanged.connect(partial(self.ledSolve.setReady, False))
        self.constraintLine.function_nodes_changed.connect(partial(self.ledCreate.setReady, False))
        self.constraintLine.function_nodes_changed.connect(partial(self.ledSolve.setReady, False))

        self.ledSolve.setEnabled(False)

        # TRANSCRIPTION STUFF
        # the transcription need to be updated if nodes changed
        self.NodesSpinBox.valueChanged.connect(self.trans_gui.updateTranscriptionMethod)

        # MODEL STUFF
        # self.model_gui.modelLoaded.connect(self.writeInStatusBar('Opening Horizon problem!'))

        # VARIABLES STUFF
        # the dynamics becomes not ready if new state or input variables are inserted
        # self.variables_gui.stateVarAdded.connect(self.dyn_gui.manageDisplay())
        # self.variables_gui.stateVarAdded.connect(self.updateHighlighter)
        # self.variables_gui.genericSignal.connect(self.on_generic_sig)
        # self.variables_gui.stateVarAdded.connect(partial(self.ledCreate, False))
        # self.variables_gui.stateVarAdded.connect(partial(self.ledSolve, False))




        # when opening horizon, fill the GUI
        for name, data in horizon_receiver.getVarDict().items():
            self.addStateVariableToGUI(name)

        for name, data in horizon_receiver.getFunctionDict().items():
            self.addFunctionToGUI(name, data['str'])
            if data['active'] is not None:
                if data['active'].getType() == 'constraint':
                    line = self.constraintLine
                    # todo getDim() only taking the row, what if it's a matrix?
                    line.addFunctionToSingleLine(name, data['active'].getDim()[0])
                    line.addFunctionToMultiLine(name)
                elif data['active'].getType() == 'costfunction':
                    line = self.costfunctionLine
                    line.addFunctionToSingleLine(name, data['active'].getDim()[0])
                    line.addFunctionToMultiLine(name)

        # fill function combo box
        self._fillFunComboBox()

        # self.initDummyStuff()

    def initDummyStuff(self):
        import os
        urdf_file = '/home/francesco/hhcm_workspace/src/horizon/horizon/examples/urdf/cart_pole.urdf'
        self.model_gui.loadModel(urdf_file)

        vars = dict(q=2, q_dot=2)

        for name, dim in vars.items():
            flag, signal = self.horizon_receiver.createVariable('State', name, dim, 0)
            if flag:
                self.addStateVariableToGUI(name)



    def updateHighlighter(self, var_name):
        # add variable to highlighter and to completer
        self.highlighter.addKeyword(var_name)
        self.fun_keywords.append('{}'.format(var_name))
        model = self.completer.model()
        model.setStringList(self.fun_keywords)


    def createButtonPushed(self):
        if self.horizon_receiver.generate():
            self.SolveButton.setEnabled(True)
            self.ledSolve.setEnabled(True)
            self.ledCreate.setReady(True)
        else:
            self.logger.warning('Failed to generate problem.')
            # with open(CSS_DIR + '/button_new.css', 'r') as f:
            #     self.SolveButton.setStyleSheet(f.read())

    def solveButtonPushed(self):
        if self.horizon_receiver.solve():
            self.PlotButton.setEnabled(True)
            self.ledSolve.setReady(True)
            # with open(CSS_DIR + '/button_new.css', 'r') as f:
            #     self.PlotButton.setStyleSheet(f.read())


    def plotButtonPushed(self):
        self.horizon_receiver.plot()

    @pyqtSlot()
    def on_generic_sig(self, str):
        self.generic_sig.emit(str)

    def setBoxNodes(self):
        n_nodes = self.NodesSpinBox.value()
        self.nodes = n_nodes
        self.horizon_receiver.setHorizonNodes(n_nodes)  # setting to casadi the new number of nodes
        self.constraintLine.setHorizonNodes(n_nodes + 1)  # n_nodes+1 to account for the final node
        self.costfunctionLine.setHorizonNodes(n_nodes + 1)  # n_nodes+1 to account for the final node
        # self.ledCreate.setReady(False)
        # self.ledSolve.setReady(False)

    def switchPage(self):
        index = self.ProblemMain.currentIndex()
        if index == 0:
            self.switchPageButton.setText('Switch to Constraints')
        else:
            self.switchPageButton.setText('Switch to Cost Functions')

        self.ProblemMain.setCurrentIndex(abs(index-1))

    def _fillFunComboBox(self):
        self.funComboBox.addItem('nothing')
        self.funComboBox.addItem('yet')
        self.funComboBox.addItem('implemented')
        self.funComboBox.setCurrentIndex(-1)


    def openFunction(self, item):

        # todo ADD USAGES: if active and where is active!!
        # regardless of where I click, get the first item of the table
        item = self.funTable.item(self.funTable.row(item), 0)

        # create window that emit a signal when closed
        self.fun_temp_win = on_destroy_signal_window.DestroySignalWindow()

        # create a layout for the window
        # widget, fromRow, int fromColumn, int rowSpan, int columnSpan, Qt::Alignment alignment = Qt::Alignment()
        self.window_layout = QGridLayout()

        #populate it
        self.label = QLabel("Function:")
        self.window_layout.addWidget(self.label, 0, 0, 1, 1)

        # todo if i UNWRAP and then edit, I lose the dependency on functions (I cannot return to WRAPPED functions)
        self.vis_fun_buttons = QWidget()
        self.vis_fun_buttons_layout = QHBoxLayout()
        self.wrapped_fun_button = QRadioButton('Wrapped')
        self.unwrapped_fun_button = QRadioButton('Unwrapped')

        self.wrapped_fun_button.setChecked(True)

        self.wrapped_fun_button.toggled.connect(partial(self.showWrappedFun, item))
        self.unwrapped_fun_button.toggled.connect(partial(self.showUnwrappedFun, item))

        self.vis_fun_buttons_layout.addWidget(self.wrapped_fun_button)
        self.vis_fun_buttons_layout.addWidget(self.unwrapped_fun_button)
        self.vis_fun_buttons.setLayout(self.vis_fun_buttons_layout)

        self.window_layout.addWidget(self.vis_fun_buttons, 0, 1, 1, 1)


        self.temp_line_edit = line_edit.LineEdit()
        str_fun = self.horizon_receiver.getFunction(item.text())['str']
        self.temp_line_edit.setText(str_fun)
        self.temp_line_edit.setDisabled(True)

        palette = QPalette()
        # palette.setColor(QPalette.Base, Qt.white)
        palette.setColor(QPalette.Text, Qt.black)
        self.temp_line_edit.setPalette(palette)
        self.window_layout.addWidget(self.temp_line_edit, 1, 0, 1, 2)

        self.edit_button = QPushButton('Edit Function')
        self.edit_button.setCheckable(True)
        # self.edit_button.setStyleSheet("background-color : lightblue")
        self.edit_button.clicked.connect(partial(self.enableFunEdit, item))
        self.window_layout.addWidget(self.edit_button, 2, 0, 1, 1)

        self.close_button = QPushButton('Ok')
        # self.edit_button.setCheckable(True)
        self.close_button.clicked.connect(self.fun_temp_win.close)
        self.window_layout.addWidget(self.close_button, 2, 1, 1, 1)

        self.fun_temp_win.setWindowTitle(item.text())
        self.fun_temp_win.setLayout(self.window_layout)

        # show window
        self.fun_temp_win.show()

        # temp_win emits a message when it is closed, before being destroyed.
        # remember that I have to manually destroy it
        self.fun_temp_win.returnDestroyed.connect(self.yieldHighlighter)
    # GUI

    def showWrappedFun(self, item):
        str_fun = self.horizon_receiver.getFunction(item.text())['str']
        self.temp_line_edit.setText(str_fun)

    def showUnwrappedFun(self, item):
        str_fun = str(self.horizon_receiver.getFunction(item.text())['fun'])
        self.temp_line_edit.setText(str_fun)

    # GUI
    def enableFunEdit(self, item):
        if self.edit_button.isChecked():
            # self.edit_button.setStyleSheet("background-color : lightblue")
            self.highlighter.setDocument(self.temp_line_edit.document())
            self.temp_line_edit.setDisabled(False)
            self.edit_button.setText('Done')
        else:
            # self.edit_button.setStyleSheet("background-color : lightgrey")
            self.highlighter.setDocument(self.funInput.document())

            # update ct_dict with edited function
            str_fun = self.temp_line_edit.toPlainText()
            flag, signal = self.horizon_receiver.editFunction(item.text(), str_fun)

            if flag:
                # update text in table of functions if function is updated.
                fun_displayed = self.funTable.item(self.funTable.row(item), 1)
                fun_displayed.setText(str_fun)
                self.logger.info(signal)
                self.on_generic_sig(signal)
            else:
                self.logger.warning('main_interface.py'.format(signal))
                self.on_generic_sig(signal)

            self.temp_line_edit.setDisabled(True)
            self.edit_button.setText('Edit Function')

    # GUI
    def yieldHighlighter(self):
        # return the highlighter to the funInput
        self.fun_temp_win.destroyCompletely()
        self.highlighter.setDocument(self.funInput.document())

    def addFunctionToGUI(self, name, str_fun):

        # adding function to highlighter
        self.highlighter.addKeyword(name)
        self.fun_keywords.append('{}'.format(name))
        model = self.completer.model()
        model.setStringList(self.fun_keywords)

        row_pos = self.funTable.rowCount()
        self.funTable.insertRow(row_pos)

        name_table = QTableWidgetItem(name)
        name_table.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable | Qt.ItemIsDragEnabled)
        name_table.setData(Qt.UserRole, str_fun)

        str_table = QTableWidgetItem(str_fun)
        str_table.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable | Qt.ItemIsDragEnabled)

        self.funTable.setItem(row_pos, 0, name_table)
        self.funTable.setItem(row_pos, 1, str_table)


    def generateCustomFunction(self):

        name = self.funNameInput.text()
        str_fun = self.funInput.toPlainText()
        flag, signal = self.horizon_receiver.addFunction(dict(name=name, str=str_fun, active=None))

        if flag:

            self.addFunctionToGUI(name, str_fun)

            # clear mask
            self.funNameInput.clear()
            self.funInput.clear()

            self.logger.info(signal)

        else:
            # todo signal mean something: 1 is nameinput missing, 2 is ..., not only info
            self.funNameInput.clear()
            self.funNameInput.setFocus()
            self.funInput.clear()

            self.logger.warning('main_inteface.py: {}'.format(signal))
            self.on_generic_sig(signal)

    def generateDefaultFunction(self):
        print('not yet implemented')

    # GUI
    def setFunEditor(self, parent):
        font = QFont()
        font.setFamily('Courier')
        font.setFixedPitch(True)
        font.setPointSize(10)

        # with QLineEdit doesn't work, so I had to override QTextEdit
        self.highlighter = highlighter.Highlighter(self.funInput.document())
        self.highlighter.addOperators(TxtToFun.getValidOperators())


        self.completer = QCompleter(self.fun_keywords)
        self.completer.setCaseSensitivity(Qt.CaseInsensitive)
        self.completer.setWrapAround(False)
        self.funInput.setCompleter(self.completer)

    # GUI



    # GUI
    def _connectActions(self):
        pass
        # todo PUTT ALL OTHER CONNECT
        # self.SVAddButton.clicked.connect(self.generateStateVariable)

    # GUI


    def __del__(self):
        # Restore sys.stdout
        self.logger.removeHandler(self.consoleLogger)
        sys.stdout = sys.__stdout__
