from PyQt5.QtWidgets import QGridLayout, QLabel, QPushButton, QHBoxLayout, QWidget, QTableWidgetItem, QRadioButton, QCompleter, QApplication
from PyQt5.QtGui import QPalette, QFont
from PyQt5.QtCore import Qt, pyqtSignal
from horizon_gui.custom_widgets import on_destroy_signal_window, line_edit
from horizon_gui.gui.gui_receiver import horizonImpl
from horizon_gui.gui.functions_module.functions_module_ui import Ui_FunctionsGUI
from functools import partial
import sys


class FunctionsGui(QWidget, Ui_FunctionsGUI):
    funAdded = pyqtSignal(str)

    def __init__(self, horizon_receiver: horizonImpl, highlighter, completer, logger=None, parent=None):
        super().__init__(parent)

        self.setupUi(self)
        self.horizon_receiver = horizon_receiver
        self.logger = logger

        self.highlighter = highlighter
        self.completer = completer

        self.fun_keywords = list()

        self.setFunEditor()

        self._connectActions()
        self._fillFunComboBox()

        # self._dummyInit()
        self._initFunctionList()


    def _dummyInit(self):

        self.horizon_receiver.createVariable('State', 'x', 2, 0)
        self.horizon_receiver.createVariable('State', 'xe', 2, 0)
        self.horizon_receiver.createVariable('State', 'e', 2, 0)
        self.horizon_receiver.createVariable('State', 'danie', 2, 0)

        fun = self.horizon_receiver.getVar('x')['var'] + self.horizon_receiver.getVar('e')['var']
        self.horizon_receiver.appendFun('ciao', fun, 'x+e')


        self.highlighter.addKeyword('x')
        self.highlighter.addKeyword('danie')
        self.highlighter.addKeyword('xe')
        self.highlighter.addKeyword('e')

    def _initFunctionList(self):
        for fun_name, fun in self.horizon_receiver.getFunction().items():
            self.addFunctionToGui(fun_name, fun['str'])

    # connect actions
    def _connectActions(self):
        self.funCustomButton.clicked.connect(self.generateCustomFunction)
        self.funDefaultButton.clicked.connect(self.generateDefaultFunction)
        self.funTable.itemDoubleClicked.connect(self.openFunction)

    def openFunction(self, item):
        # todo ADD USAGES: if active and where is active!!
        # regardless of where I click, get the first item of the table
        item = self.funTable.item(self.funTable.row(item), 0)

        # create window that emit a signal when closed
        self.fun_temp_win = on_destroy_signal_window.DestroySignalWindow()

        # create a layout for the window
        # widget, fromRow, int fromColumn, int rowSpan, int columnSpan, Qt::Alignment alignment = Qt::Alignment()
        self.window_layout = QGridLayout()

        # populate it
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


    def showWrappedFun(self, item):
        str_fun = self.horizon_receiver.getFunction(item.text())['str']
        self.temp_line_edit.setText(str_fun)


    def showUnwrappedFun(self, item):
        str_fun = str(self.horizon_receiver.getFunction(item.text())['fun'])
        self.temp_line_edit.setText(str_fun)


    def enableFunEdit(self, item):
        if self.edit_button.isChecked():
            # self.edit_button.setStyleSheet("background-color : lightblue")
            self.highlighter.setDocument(self.temp_line_edit.document())
            self.temp_line_edit.setDisabled(False)
            self.edit_button.setText('Done')
        else:
            # self.edit_button.setStyleSheet("background-color : lightgrey")
            self.highlighter.setDocument(self.funInput.document())

            # update horizon with edited function
            str_fun = self.temp_line_edit.toPlainText()
            flag, signal = self.horizon_receiver.editFunction(item.text(), str_fun)

            if flag:
                # update text in table of functions if function is updated.
                fun_displayed = self.funTable.item(self.funTable.row(item), 1)
                fun_displayed.setText(str_fun)
                if self.logger:
                    self.logger.info(signal)
            else:
                if self.logger:
                    self.logger.warning('main_interface.py'.format(signal))


            self.temp_line_edit.setDisabled(True)
            self.edit_button.setText('Edit Function')

    def _fillFunComboBox(self):
        self.funComboBox.addItem('nothing')
        self.funComboBox.addItem('yet')
        self.funComboBox.addItem('implemented')
        self.funComboBox.setCurrentIndex(-1)

    def addFunctionToGui(self, name, str_fun):

        # adding function to highlighter
        self.highlighter.addKeyword(name)
        # todo this can be moved outside of this class
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

        self.funAdded.emit(name)

    def generateCustomFunction(self):

        name = self.funNameInput.text()
        str_fun = self.funInput.toPlainText()
        flag, signal = self.horizon_receiver.addFunction(dict(name=name, str=str_fun, active=None))

        if flag:

            self.addFunctionToGui(name, str_fun)

            # clear mask
            self.funNameInput.clear()
            self.funInput.clear()
            if self.logger:
                self.logger.info(signal)

        else:
            # todo signal mean something: 1 is nameinput missing, 2 is ..., not only info
            self.funNameInput.clear()
            self.funNameInput.setFocus()
            self.funInput.clear()
            if self.logger:
                self.logger.warning('main_inteface.py: {}'.format(signal))

    def generateDefaultFunction(self):
        print('not yet implemented')

    def setFunEditor(self):
        font = QFont()
        font.setFamily('Courier')
        font.setFixedPitch(True)
        font.setPointSize(10)

        self.highlighter.setDocument(self.funInput.document())
        self.funInput.setCompleter(self.completer)

    def yieldHighlighter(self):
        # return the highlighter to the funInput
        self.fun_temp_win.destroyCompletely()
        self.highlighter.setDocument(self.funInput.document())

if __name__ == '__main__':

    hr = horizonImpl(10)
    from horizon_gui.custom_functions.highlighter import Highlighter
    from horizon_gui.custom_functions.txt_to_fun import TxtToFun
    # create highlighter
    highlighter = Highlighter()
    # adding math operators to highlighter
    math_operators = TxtToFun.getValidOperators()
    highlighter.addOperators(math_operators)

    # create completer
    fun_keywords = []
    completer = QCompleter(fun_keywords)
    completer.setCaseSensitivity(Qt.CaseInsensitive)
    completer.setWrapAround(False)

    app = QApplication(sys.argv)
    gui = FunctionsGui(hr, highlighter, completer)
    gui.show()

    sys.exit(app.exec_())
