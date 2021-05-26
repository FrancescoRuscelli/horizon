import sys
from functools import partial
from classes import gui_receiver
import logging

from PyQt5.QtWidgets import (QMainWindow, QApplication, QSplitter, QCheckBox, QGridLayout, QGroupBox, QLabel, QToolBar, QAction, qApp,
                             QMenu, QPushButton, QRadioButton, QVBoxLayout, QHBoxLayout, QWidget, QSlider, QDesktopWidget, QMenuBar, QStyle,
                             QSpinBox, QStyleFactory, QFrame, QTabWidget, QProgressBar, QDialog, QButtonGroup, QLayout, QLineEdit, QTextEdit, QTableWidgetItem,
                             QRubberBand, QComboBox, QScrollArea, QTableWidget, QCompleter, QToolButton, QMessageBox, QListWidgetItem, QHeaderView, QAbstractItemView)

from PyQt5.QtGui import QIcon, QPixmap, QKeyEvent, QPalette, QColor, QDrag, QTextCursor, QPainter, QCursor, QBrush, QTextCharFormat, QSyntaxHighlighter, QFont, QFontMetrics
from PyQt5.QtCore import Qt, pyqtSignal, QRect, QMimeData, QPoint, QSize, QRegExp, pyqtSlot, QObject

from widget1_ui import Ui_Form
from custom_functions import highlighter
from custom_widgets import horizon_line, line_edit, on_destroy_signal_window, box_state_var, highlight_delegate
import qrc_resources

# todo:
# SCREEN WITH ERRORS from casadi and python                           DONE
# EDIT CONSTRAINT NOW IS WRONG (visualize function and not string)    DONE
# EMBELLISH USAGE TABLE                                               DONE
# make option for showing all constraints_line together or one for each tab
# REMOVE CONSTRAINT (FROM LIST)
# ADD CONSISTENCY CHECK FOR CASADI VARIABLES
# ADD PRINT CASADI FUNCTION TO CHECK
# ADD LOG OF EVERYTHING DONE (HISTORY)
# REMOVE STATE VARIABLE (FROM TABLE) CHECK USAGES IN CONSTRAINTS
# ADD A VARIABLE EDITOR
# ADD DIALOG BUTTON TO CONSTRAINT (YES AND NO)
# ADD BUTTON FOR ADDING HANDLES TO MULTI-SLIDERS

# class EmittingStream(QObject):
#
#     textWritten = pyqtSignal(str)
#
#     def write(self, text):
#         now = datetime.now()
#         # current_time = now.strftime("%H:%M:%S")
#         self.textWritten.emit('{}'.format(text))
#
#     def flush(self):
#         pass

class Widget1(QWidget, Ui_Form):
    invalid_sv = pyqtSignal(str)
    invalid_fun = pyqtSignal(str)
    generic_sig = pyqtSignal(str)

    def __init__(self):
        super(Widget1, self).__init__()

        self.setupUi(self)
        # #todo logger! use it everywhere!
        self.logger = logging.getLogger('logger')
        self.logger.addHandler(self.consoleLogger)
        self.logger.setLevel(logging.DEBUG)

        # emitting stream from python terminal
        # sys.stdout = EmittingStream()
        # sys.stdout.textWritten.connect(self.normalOutputWritten)

        N = 20
        # todo put it somewhere else?
        self.horizon_receiver = gui_receiver.horizonImpl(N, self.logger)


        self.fun_keywords = list()


        self._connectActions()
        self.SVNodeInput.setRange(-N, 0)

        # self.layout_problem = QHBoxLayout(self.PRB)
        # self.horizonLine = horizon_line.HorizonLine()
        # self.layout_problem.addWidget(self.horizonLine)

        self.layout_ct = QVBoxLayout(self.CTPage)
        self.constraintLine = horizon_line.HorizonLine(self.horizon_receiver, 'constraint', logger=self.logger)
        self.constraintLine.setContentsMargins(0, 40, 0, 0)
        self.layout_ct.addWidget(self.constraintLine)

        self.layout_cf = QVBoxLayout(self.CFPage)
        self.costfunctionLine = horizon_line.HorizonLine(self.horizon_receiver, 'costfunction', logger=self.logger)
        self.costfunctionLine.setContentsMargins(0, 40, 0, 0)
        self.layout_cf.addWidget(self.costfunctionLine)

        self.ct_layout = QHBoxLayout(self.funBox)
        self.ct_entry = self.setFunEditor(self.funBox)

        # self.fun_type_input = 'generic'
        # self.constraintTypeButton.data = 'constraint'
        # self.costfunctionTypeButton.data = 'costfunction'
        # self.genericTypeButton.data = 'generic'
        #
        # self.constraintTypeButton.toggled.connect(self.updateInputFunctionType)
        # self.costfunctionTypeButton.toggled.connect(self.updateInputFunctionType)
        # self.genericTypeButton.toggled.connect(self.updateInputFunctionType)

        self.funButton.clicked.connect(self.generateFunction)
        self.funTable.itemDoubleClicked.connect(self.openFunction)
        self.SVTable.itemDoubleClicked.connect(self.openSV)
        self.switchPageButton.clicked.connect(self.switchPage)
        self.constraintLine.add_fun_horizon.connect(self.horizon_receiver.addFunction)

    @pyqtSlot()
    def on_invalid_sv(self, str):
        self.invalid_sv.emit(str)

    @pyqtSlot()
    def on_invalid_fun(self, str):
        self.invalid_fun.emit(str)

    def on_generic_sig(self, str):
        self.generic_sig.emit(str)
    # GUI
    def switchPage(self):
        index = self.ProblemMain.currentIndex()
        if index == 0:
            self.switchPageButton.setText('Switch to Constraints')
        else:
            self.switchPageButton.setText('Switch to Cost Functions')

        self.ProblemMain.setCurrentIndex(abs(index-1))

    # GUI
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
        self.display_dim.setText(str(self.horizon_receiver.sv_dict[item.text()]['dim']))
        self.display_dim.setDisabled(True)
        self.display_dim.setPalette(palette)
        self.sv_window_layout.addWidget(self.display_dim, 0, 3, 1, 1)

        self.label_fun = QLabel("var:")
        self.sv_window_layout.addWidget(self.label_fun, 1, 0, 1, 1)

        self.display_fun = QLineEdit()
        width = self.display_fun.fontMetrics().width(str(self.horizon_receiver.sv_dict[item.text()]['var']))
        self.display_fun.setText(str(self.horizon_receiver.sv_dict[item.text()]['var']))
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
        active_fun_list = [i for i in [(elem['str'], elem['active']) for name, elem in self.horizon_receiver.fun_dict.items() if 'active' in elem] if i[1]]

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


    def generateFunction(self):
        name = self.funNameInput.text()
        str_fun = self.funInput.toPlainText()
        flag, signal = self.horizon_receiver.addFunction(dict(name=name, str=str_fun, active=None))

        if flag:

            # todo add FUNCTION to highlighter and to completer
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


        self.completer = QCompleter(self.fun_keywords)
        self.completer.setCaseSensitivity(Qt.CaseInsensitive)
        self.completer.setWrapAround(False)
        self.funInput.setCompleter(self.completer)

    # GUI
    def generateStateVariable(self):

        default_dim = 1
        default_past_node = 0

        sv_name = self.SVNameInput.text()
        sv_dim = self.SVDimInput.value()
        sv_nodes = self.SVNodeInput.value()

        # todo put in gui_receiver?

        flag, signal = self.horizon_receiver.addStateVariable(dict(name=sv_name, dim=sv_dim, prev=sv_nodes))

        if flag:
            self._addRowToSVTable(sv_name)
            # add variable to highlighter and to completer
            self.highlighter.addKeyword(sv_name)
            self.fun_keywords.append('{}'.format(sv_name))
            model = self.completer.model()
            model.setStringList(self.fun_keywords)

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
        self.SVTable.setItem(row_pos, 1, QTableWidgetItem(str(self.horizon_receiver.sv_dict[name]['dim'])))

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
        sys.stdout = sys.__stdout__

class HorizonGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        # widget generated with qt designer
        widget1 = Widget1()

        # another qwidget
        # self.red_widget = QWidget(self)
        # self.red_widget.setAutoFillBackground(True)
        #
        # palette = self.red_widget.palette()
        # palette.setColor(QPalette.Window, QColor('red'))
        # self.red_widget.setPalette(palette)

        # yet another qwidget
        self.text_widget = QLabel("Hello, World")
        self.text_widget.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)

        # generate a layout for a widget, I can populate the layout with more widget
        self.main_layout = QVBoxLayout()
        # self.main_layout.addWidget(self.red_widget,1)
        self.widget1 = widget1
        self.main_layout.addWidget(self.widget1, 4)
        # self.main_layout.addWidget(self.text_widget,1)

        # generate a main widget to assign to centralwidget
        self.main_widget = QWidget()
        self.main_widget.setLayout(self.main_layout)
        self.setCentralWidget(self.main_widget)
        self.setWindowTitle("Horizon GUI")
        # setting geometry
        # self.setGeometry(1000, 200, 300, 200)
        self.showMaximized()
        # miniwindow = TimeLine()

        # self.setCentralWidget(self.centralWidget)


        self._createActions()
        self._createMenuBar()
        self._createToolBars()
        # self._createContextMenu()
        # self._createButton()
        self._connectActions()
        self._createStatusBar()

    def newFile(self):
        # Logic for creating a new file goes here...
        self.text_widget.setText("<b>File > New</b> clicked")

    def openFile(self):
        # Logic for opening an existing file goes here...
        self.text_widget.setText("<b>File > Open... </b> clicked")

    def openRecentFile(self, filename):
        # Logic for opening a recent file goes here...
        self.text_widget.setText(f"<b>{filename}</b> opened")

    def saveFile(self):
        # Logic for saving a file goes here...
        self.text_widget.setText("<b>File > Save</b> clicked")

    def copyContent(self):
        # Logic for copying content goes here...
        self.text_widget.setText("<b>Edit > Copy</b> clicked")

    def pasteContent(self):
        # Logic for pasting content goes here...
        self.text_widget.setText("<b>Edit > Paste</b> clicked")

    def cutContent(self):
        # Logic for cutting content goes here...
        self.text_widget.setText("<b>Edit > Cut</b> clicked")

    def helpContent(self):
        # Logic for launching help goes here...
        self.text_widget.setText("<b>Help > Help Content...</b> clicked")

    def about(self):
        # Logic for showing an about dialog content goes here...
        self.text_widget.setText("<b>Help > About...</b> clicked")

    def _connectActions(self):
        # Connect File actions
        self.newAction.triggered.connect(self.newFile)
        self.openAction.triggered.connect(self.openFile)
        self.saveAction.triggered.connect(self.saveFile)
        self.exitAction.triggered.connect(self.close)
        # Connect Edit actions
        self.copyAction.triggered.connect(self.copyContent)
        self.pasteAction.triggered.connect(self.pasteContent)
        self.cutAction.triggered.connect(self.cutContent)
        # Connect Help actions
        self.helpContentAction.triggered.connect(self.helpContent)
        self.aboutAction.triggered.connect(self.about)
        # Connect Open Recent to dynamically populate it
        self.openRecentMenu.aboutToShow.connect(self.populateOpenRecent)

        # This is amazing, I can connect a custom signal from a widget to the main window here
        # basically horizonLine in widget1 sends a signal with a msg, which I connect to the status bar
        self.widget1.constraintLine.repeated_fun.connect(self.writeInStatusBar)
        self.widget1.costfunctionLine.repeated_fun.connect(self.writeInStatusBar)
        self.widget1.invalid_fun.connect(self.writeInStatusBar)
        self.widget1.invalid_sv.connect(self.writeInStatusBar)
        self.widget1.generic_sig.connect(self.writeInStatusBar)

    def _createContextMenu(self):
        # Setting contextMenuPolicy
        self.text_widget.setContextMenuPolicy(Qt.ActionsContextMenu)
        # Populating the widget with actions
        self.text_widget.addAction(self.newAction)
        self.text_widget.addAction(self.openAction)
        self.text_widget.addAction(self.saveAction)
        self.text_widget.addAction(self.copyAction)
        self.text_widget.addAction(self.pasteAction)
        self.text_widget.addAction(self.cutAction)

    def _createActions(self):
        # Creating action using the first constructor
        self.newAction = QAction(self)
        self.newAction.setText("&New")
        self.newAction.setShortcut("Ctrl+N")
        newTip = "Create a new file"
        self.newAction.setStatusTip(newTip)
        self.newAction.setToolTip(newTip)

        self.openAction = QAction("&Open...", self)
        self.openAction.setShortcut("Ctrl+O")

        self.saveAction = QAction("&Save", self)
        self.saveAction.setShortcut("Ctrl+S")

        # exit action
        self.exitAction = QAction(self.style().standardIcon(QStyle.SP_DialogCancelButton), '&Exit', self)
        self.exitAction.setShortcut('Ctrl+Q')
        self.exitAction.setStatusTip('Exit application')
        self.exitAction.triggered.connect(qApp.quit)

        self.copyAction = QAction("&Copy", self)
        self.pasteAction = QAction("&Paste", self)
        self.cutAction = QAction("C&ut", self)
        self.helpContentAction = QAction("&Help Content", self)

        self.aboutAction = QAction("&About", self)

    def writeInStatusBar(self, msg):
        self.statusbar.showMessage(msg, 10000)

    def populateOpenRecent(self):
        # Step 1. Remove the old options from the menu
        self.openRecentMenu.clear()
        # Step 2. Dynamically create the actions
        actions = []
        filenames = [f"File-{n}" for n in range(5)]
        for filename in filenames:
            action = QAction(filename, self)
            action.triggered.connect(partial(self.openRecentFile, filename))
            actions.append(action)
        # Step 3. Add the actions to the menu
        self.openRecentMenu.addActions(actions)

    def _createStatusBar(self):
        self.statusbar = self.statusBar()
        self.statusbar.showMessage("Ready", 3000)
        self.wcLabel = QLabel("Words")
        self.wcProgressBar = QProgressBar(self)
        self.statusbar.addPermanentWidget(self.wcProgressBar)

    def _createMenuBar(self):

        menuBar = QMenuBar(self)
        # menuBar.setNativeMenuBar(False) # if true, the menu is in the native menubar, if false it is in the created window

        self.setMenuBar(menuBar)

        fileMenu = QMenu("&Maledetto", self)
        menuBar.addMenu(fileMenu)
        # Creating menus using a title
        editMenu = menuBar.addMenu("&Porco")
        helpMenu = menuBar.addMenu(QIcon(":/resources/info.svg"), "&Dio")
        saveMenu = menuBar.addMenu(self.style().standardIcon(QStyle.SP_DialogSaveButton), "&Save")
        aboutMenu = menuBar.addMenu("&Menu")
        newMenu = menuBar.addMenu('&New')

        newMenu.addAction(self.newAction)

        fileMenu.addAction(self.openAction)
        # Adding an Open Recent submenu
        self.openRecentMenu = fileMenu.addMenu("Open Recent")
        fileMenu.addAction(self.saveAction)

        fileMenu.addAction(self.exitAction)
        fileMenu.addSeparator()
        fileMenu.addAction(self.aboutAction)

        porcoMenu = aboutMenu.addMenu("Porco")
        porcoMenu.addAction("il cristo")
        porcoMenu.addAction("dio")
        porcoMenu.addAction("gesu")


    def _createButton(self):

        self.button = QPushButton("My button")
        icon = QIcon(":/resources/rocket.svg")
        self.button.setIcon(icon)
        self.button.clicked.connect(self.change_icon)

        self.setCentralWidget(self.button)
    def change_icon(self):
        icon = QIcon(":/resources/info.svg")
        self.button.setIcon(icon)

    def _createToolBars(self):
        # Using a title
        fileToolBar = self.addToolBar("File")
        fileToolBar.addAction(self.newAction)
        fileToolBar.addAction(self.openAction)
        fileToolBar.addAction(self.saveAction)
        fileToolBar.setMovable(False)


        # Using a QToolBar object
        editToolBar = QToolBar("Edit", self)
        self.addToolBar(editToolBar)
        self.fontSizeSpinBox = QSpinBox()
        self.fontSizeSpinBox.setFocusPolicy(Qt.NoFocus)
        editToolBar.addWidget(self.fontSizeSpinBox)

        # Using a QToolBar object and a toolbar area
        helpToolBar = QToolBar("Help", self)
        self.addToolBar(Qt.LeftToolBarArea, helpToolBar)

class TimeLine(QWidget):
    def __init__(self):
        super().__init__()

        self.groupBox = QGroupBox(self.centralwidget)
        self.groupBox.setGeometry(QRect(40, 20, 341, 391))
        self.groupBox.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.groupBox.setObjectName("groupBox")

        return self.groupBox




class Window_1(QWidget):
    def __init__(self):
        super().__init__()

        grid = QGridLayout()
        grid.addWidget(self.createExampleGroup(), 0, 0)
        grid.addWidget(self.createExampleGroup(), 1, 0)
        grid.addWidget(self.createExampleGroup(), 0, 1)
        grid.addWidget(self.createExampleGroup(), 1, 1)
        self.setLayout(grid)

        self.setWindowTitle("PyQt5 Sliders")
        self.resize(400, 300)

    def createExampleGroup(self):
        groupBox = QGroupBox("Slider Example")

        radio1 = QRadioButton("&Radio horizontal slider")

        slider = QSlider(Qt.Horizontal)
        slider.setFocusPolicy(Qt.StrongFocus)
        slider.setTickPosition(QSlider.TicksBothSides)
        slider.setTickInterval(10)
        slider.setSingleStep(1)

        radio1.setChecked(True)

        vbox = QVBoxLayout()
        vbox.addWidget(radio1)
        vbox.addWidget(slider)
        vbox.addStretch(1)
        groupBox.setLayout(vbox)

        return groupBox

class Window(QMainWindow):
    def __init__(self):
        super().__init__()

        # setting title
        self.setWindowTitle("Python")

        # setting geometry
        self.setGeometry(1000, 200, 300, 200)

        # calling method
        self.UiComponents()

        # showing all the widgets
        self.show()

    # method for widgets
    def UiComponents(self):
        # creating a push button
        button = QPushButton("CLICK", self)

        # setting geometry of button
        button.setGeometry(100, 100, 100, 30)

        # adding action to a button
        button.clicked.connect(self.clickme)

    # action method
    def clickme(self):
        # printing pressed
        print("pressed")


# create pyqt5 app
# app = QApplication(sys.argv)
# window = Window()
# sys.exit(app.exec_())

app = QApplication(sys.argv)
gui = HorizonGUI()
# gui = Widget1()

gui.show()
sys.exit(app.exec_())

# window = QWidget()
# window.setWindowTitle('PyQt5 App')
# window.setGeometry(100, 100, 280, 80)
# window.move(60, 15)
# helloMsg = QLabel('<h1>Hello World!</h1>', parent=window)
# helloMsg.move(60, 15)
#
#
# # 4. Show your application's GUI
# window.show()
#
# # 5. Run your application's event loop (or main