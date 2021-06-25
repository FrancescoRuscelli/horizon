import sys
from functools import partial

from PyQt5.QtWidgets import (QGridLayout, QLabel, QPushButton, QRadioButton, QVBoxLayout, QHBoxLayout, QWidget,
                             QLineEdit, QTableWidgetItem, QTableWidget, QCompleter, QHeaderView)

from PyQt5.QtGui import QPalette, QFont
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot

from horizon_gui.gui.widget1_ui import Ui_HorizonGUI
from horizon_gui.custom_functions import highlighter
from horizon_gui.custom_widgets import horizon_line, node_box_line, line_edit, on_destroy_signal_window, highlight_delegate


class MainInterface(QWidget, Ui_HorizonGUI):
    generic_sig = pyqtSignal(str)

    def __init__(self, horizon_receiver, logger=None):
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
        N = horizon_receiver.getNodes()

        self.fun_keywords = list()

        self._connectActions()

        # spinbox to set old variables
        self.SVNodeInput.setRange(-N, 0)

        self.layout_ct = QVBoxLayout(self.CTPage)
        # self.constraintLine = horizon_multi_line.HorizonMultiLine(self.horizon_receiver, 'constraint', nodes=N, logger=self.logger)
        self.constraintLine = horizon_line.HorizonLine(self.horizon_receiver, 'constraint', nodes=N, logger=self.logger)
        self.constraintLine.setContentsMargins(0, 40, 0, 0)
        self.layout_ct.addWidget(self.constraintLine)

        self.layout_cf = QVBoxLayout(self.CFPage)
        self.costfunctionLine = horizon_line.HorizonLine(self.horizon_receiver, 'costfunction', nodes=N, logger=self.logger)
        self.costfunctionLine.setContentsMargins(0, 40, 0, 0)
        self.layout_cf.addWidget(self.costfunctionLine)

        self.ct_layout = QHBoxLayout(self.funBox)
        self.ct_entry = self.setFunEditor(self.funBox)


        self.funButton.clicked.connect(self.generateFunction)
        self.funTable.itemDoubleClicked.connect(self.openFunction)
        self.SVTable.itemDoubleClicked.connect(self.openSV)
        self.switchPageButton.clicked.connect(self.switchPage)
        self.constraintLine.add_fun_horizon.connect(self.horizon_receiver.addFunction)
        self.constraintLine.function_tab.funNodesChanged.connect(self.horizon_receiver.updateFunctionNodes)
        self.costfunctionLine.function_tab.funNodesChanged.connect(self.horizon_receiver.updateFunctionNodes)
        self.NodesSpinBox.valueChanged.connect(self.setBoxNodes)
        self.SingleLineButton.toggled.connect(partial(self.constraintLine.switchPage, self.constraintLine.Single))
        self.SingleLineButton.toggled.connect(partial(self.costfunctionLine.switchPage, self.constraintLine.Single))
        self.MultipleLineButton.toggled.connect(partial(self.constraintLine.switchPage, self.costfunctionLine.Multi))
        self.MultipleLineButton.toggled.connect(partial(self.costfunctionLine.switchPage, self.costfunctionLine.Multi))

        # when opening horizon, fill the GUI
        for name, data in horizon_receiver.getVarDict().items():
            self.addStateVariableToGUI(name)

        for name, data in horizon_receiver.getFunctionDict().items():
            self.addFunctionToGUI(name, data['str'])
            if data['active'] is not None:
                if data['active'].getType() == 'constraint':
                    line = self.constraintLine
                    line.addFunctionToSingleLine(name)
                    line.addFunctionToMultiLine(name)
                elif data['active'].getType() == 'costfunction':
                    line = self.costfunctionLine
                    line.addFunctionToSingleLine(name)
                    line.addFunctionToMultiLine(name)




    @pyqtSlot()
    def on_generic_sig(self, str):
        self.generic_sig.emit(str)

    def setBoxNodes(self, n_nodes):
        self.horizon_receiver.setHorizonNodes(n_nodes)
        self.constraintLine.setHorizonNodes(n_nodes)
        self.costfunctionLine.setHorizonNodes(n_nodes)

    def switchPage(self):
        index = self.ProblemMain.currentIndex()
        if index == 0:
            self.switchPageButton.setText('Switch to Constraints')
        else:
            self.switchPageButton.setText('Switch to Cost Functions')

        self.ProblemMain.setCurrentIndex(abs(index-1))

    def openFunction(self, item):

        # todo ADD USAGES: if active and where is active!!


        # regardless of where I click, get the first item of the table
        item = self.funTable.item(self.funTable.row(item), 0)

        # create window that emit a signal when closed
        self.temp_win = on_destroy_signal_window.DestroySignalWindow()

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
        self.close_button.clicked.connect(self.temp_win.close)
        self.window_layout.addWidget(self.close_button, 2, 1, 1, 1)

        self.temp_win.setWindowTitle(item.text())
        self.temp_win.setLayout(self.window_layout)

        # show window
        self.temp_win.show()

        # temp_win emits a message when it is closed, before being destroyed.
        # remember that I have to manually destroy it
        self.temp_win.returnDestroyed.connect(self.yieldHighlighter)
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
                self.logger.warning(signal)
                self.on_generic_sig(signal)

            self.temp_line_edit.setDisabled(True)
            self.edit_button.setText('Edit Function')

    # GUI
    def yieldHighlighter(self):
        # return the highlighter to the funInput
        self.temp_win.destroyCompletely()
        self.highlighter.setDocument(self.funInput.document())

    # GUI
    def openSV(self, item):

        # regardless of where I click, get the first item of the table
        item = self.SVTable.item(self.SVTable.row(item), 0)
        # create window that emit a signal when closed
        self.temp_win = on_destroy_signal_window.DestroySignalWindow()

        # create a layout for the window
        #widget, fromRow, int fromColumn, int rowSpan, int columnSpan, Qt::Alignment alignment = Qt::Alignment()
        self.sv_window_layout = QGridLayout()

        # populate it
        self.label_name = QLabel("name:")
        self.sv_window_layout.addWidget(self.label_name, 0, 0, 1, 1)

        palette = QPalette()
        palette.setColor(QPalette.Text, Qt.black)

        self.display_name = QLineEdit()
        self.display_name.setFixedWidth(50)
        self.display_name.setText(item.text())
        self.display_name.setDisabled(True)
        self.display_name.setPalette(palette)


        self.display_name.setPalette(palette)

        self.sv_window_layout.addWidget(self.display_name, 0, 1, 1, 1)

        self.label_dim = QLabel("dimension:")
        self.sv_window_layout.addWidget(self.label_dim, 0, 2, 1, 1)

        self.display_dim = QLineEdit()
        self.display_dim.setFixedWidth(20)
        self.display_dim.setText(str(self.horizon_receiver.getVar(item.text())['dim']))
        self.display_dim.setDisabled(True)
        self.display_dim.setPalette(palette)
        self.sv_window_layout.addWidget(self.display_dim, 0, 3, 1, 1)

        self.label_fun = QLabel("var:")
        self.sv_window_layout.addWidget(self.label_fun, 1, 0, 1, 1)

        self.display_fun = QLineEdit()
        width = self.display_fun.fontMetrics().width(str(self.horizon_receiver.getVar(item.text())['var']))
        self.display_fun.setText(str(self.horizon_receiver.getVar(item.text())['var']))
        self.display_fun.setDisabled(True)
        # todo magic number?
        self.display_fun.setMinimumWidth(width+5)
        self.display_fun.setAlignment(Qt.AlignCenter)
        self.display_fun.setPalette(palette)
        self.sv_window_layout.addWidget(self.display_fun, 1, 1, 1, 3)

        self.label_usages = QLabel("usages:")
        self.sv_window_layout.addWidget(self.label_usages, 2, 0, 1, 4)

        self.usages_table = QTableWidget()
        self.usages_table.setColumnCount(3)
        self.usages_table.setHorizontalHeaderLabels(['Name', 'Function', 'Type'])
        self._delegate = highlight_delegate.HighlightDelegate(self.usages_table)
        self.usages_table.setItemDelegateForColumn(1, self._delegate)
        self._delegate.setFilters(item.text())

        header = self.usages_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.Stretch)
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents)

        self.sv_window_layout.addWidget(self.usages_table, 3, 0, 1, 4)
        # # todo add usages (add constraint logic and check if constraint depends on variable)
        # TODO REMOVE CT ADD FUNCTION
        # get only active function
        active_fun_list = [i for i in [(elem['str'], elem['active']) for name, elem in self.horizon_receiver.getFunctionDict().items() if 'active' in elem] if i[1]]

        for str_function, function in active_fun_list:
            if item.text() in function.getVariables():
                row_pos = self.usages_table.rowCount()
                self.usages_table.insertRow(row_pos)
                fun_name = QTableWidgetItem(function.getName())
                fun_name.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)

                fun_str = QTableWidgetItem(str_function)
                fun_str.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)

                fun_type = QTableWidgetItem(function.getType())
                fun_type.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)

                self.usages_table.setItem(row_pos, 0, fun_name)
                self.usages_table.setItem(row_pos, 1, fun_str)
                self.usages_table.setItem(row_pos, 2, fun_type)


        self.remove_button = QPushButton('Remove Variable')
        # self.edit_button.setStyleSheet("background-color : lightblue")
        self.remove_button.clicked.connect(partial(self.horizon_receiver.removeStateVariable, item))

        self.sv_window_layout.addWidget(self.remove_button)

        self.temp_win.setWindowTitle(item.text())
        self.temp_win.setLayout(self.sv_window_layout)

        # show window
        self.temp_win.show()

        # temp_win emits a message when it is closed, before being destroyed.
        # remember that I have to manually destroy it
        # self.temp_win.returnDestroyed.connect(self.yieldHighlighter)

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


    def generateFunction(self):

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

            self.logger.warning(signal)
            self.on_generic_sig(signal)

    # GUI
    def setFunEditor(self, parent):
        font = QFont()
        font.setFamily('Courier')
        font.setFixedPitch(True)
        font.setPointSize(10)

        # with QLineEdit doesn't work, so I had to override QTextEdit
        self.highlighter = highlighter.Highlighter(self.funInput.document())
        self.highlighter.addOperators(self.horizon_receiver.getValidOperators())


        self.completer = QCompleter(self.fun_keywords)
        self.completer.setCaseSensitivity(Qt.CaseInsensitive)
        self.completer.setWrapAround(False)
        self.funInput.setCompleter(self.completer)

    # GUI
    def addStateVariableToGUI(self, sv_name):

        self._addRowToSVTable(sv_name)
        # add variable to highlighter and to completer
        self.highlighter.addKeyword(sv_name)
        self.fun_keywords.append('{}'.format(sv_name))
        model = self.completer.model()
        model.setStringList(self.fun_keywords)

    def generateStateVariable(self):

        default_dim = 1
        default_past_node = 0

        sv_name = self.SVNameInput.text()
        sv_dim = self.SVDimInput.value()
        sv_nodes = self.SVNodeInput.value()

        # todo put in gui_receiver?

        flag, signal = self.horizon_receiver.addStateVariable(dict(name=sv_name, dim=sv_dim, prev=sv_nodes))

        if flag:

            self.addStateVariableToGUI(sv_name)
            self.SVNameInput.clear()
            self.SVDimInput.setValue(default_dim)
            self.SVNodeInput.setValue(default_past_node)

            self.logger.info(signal)
            self.on_generic_sig(signal)

        else:
            self.SVNameInput.setFocus()
            self.logger.warning(signal)
            self.on_generic_sig(signal)

    # GUI
    def _connectActions(self):

        # todo PUTT ALL OTHER CONNECT
        self.SVAddButton.clicked.connect(self.generateStateVariable)

    # GUI
    def _addRowToSVTable(self, name):

        row_pos = self.SVTable.rowCount()
        self.SVTable.insertRow(row_pos)

        self.SVTable.setItem(row_pos, 0, QTableWidgetItem(name))
        self.SVTable.setItem(row_pos, 1, QTableWidgetItem(str(self.horizon_receiver.getVar(name)['dim'])))

        # self.SVTable.setCellWidget(row_pos, 2, scroll)

    # def normalOutputWritten(self, text):
    #
    #     cursor = self.codeStream.textCursor()
    #     cursor.movePosition(QTextCursor.End)
    #     cursor.insertText(text)
    #     self.codeStream.setTextCursor(cursor)
    #     self.codeStream.ensureCursorVisible()
    #
    # def closeEvent(self, event):
    #     """Shuts down application on close."""
    #     # Return stdout to defaults.
    #     sys.stdout = sys.__stdout__
    #     super().closeEvent(event)

    def __del__(self):
        # Restore sys.stdout
        self.logger.removeHandler(self.consoleLogger)
        sys.stdout = sys.__stdout__
