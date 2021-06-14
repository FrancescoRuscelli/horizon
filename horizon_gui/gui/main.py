import sys
from functools import partial
import pickle
from PyQt5.QtWidgets import (QMainWindow, QApplication, QLabel, QToolBar, QAction, qApp, QLineEdit,
                             QMenu, QPushButton, QVBoxLayout, QGridLayout, QWidget, QMenuBar, QStyle,
                             QSpinBox, QProgressBar)

from PyQt5.QtGui import QIcon
from PyQt5.QtCore import Qt

from horizon_gui.custom_widgets import starting_screen, main_interface, box_state_var, on_destroy_signal_window

# todo:
# SCREEN WITH ERRORS from casadi and python                           DONE
# EDIT CONSTRAINT NOW IS WRONG (visualize function and not string)    DONE
# EMBELLISH USAGE TABLE                                               DONE
# make so that, when nodes number changes, the slices adapt (basically they don't have to exceed the maximum and the minimum)
# make option for showing all constraints_line together or one for each tab
# REMOVE CONSTRAINT (FROM LIST)
# ADD CONSISTENCY CHECK FOR CASADI VARIABLES
# ADD PRINT CASADI FUNCTION TO CHECK
# ADD LOG OF EVERYTHING DONE (HISTORY)
# REMOVE STATE VARIABLE (FROM TABLE) CHECK USAGES IN CONSTRAINTS
# ADD A VARIABLE EDITOR
# ADD DIALOG BUTTON TO CONSTRAINT (YES AND NO)
# ADD BUTTON FOR ADDING HANDLES TO MULTI-SLIDERS
# ADD INPUT/STATE VARIABLE SELECTOR

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

class HorizonGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        # initial screen for Horizon GUI

        self.starting_screen = starting_screen.StartingScreen()
        self.setCentralWidget(self.starting_screen)

        self.nodes_input = QSpinBox()
        self._createActions()
        self._createMenuBar()
        # self._createContextMenu()
        # self._createButton()
        self._connectActions()
        self._createStatusBar()

        self.showMaximized()


    def initializeGui(self):

        setup_info = dict()

        setup_info['nodes'] = self.nodes_input.value()
        # generate a layout for a widget, I can populate the layout with more widget
        self.main_layout = QVBoxLayout()
        # self.main_layout.addWidget(self.red_widget,1)
        self.widget_gui = main_interface.MainInterface(setup_info)
        self.main_layout.addWidget(self.widget_gui)

        # generate a main widget to assign to centralwidget
        self.main_widget = QWidget()
        self.main_widget.setLayout(self.main_layout)
        self.setCentralWidget(self.main_widget)
        self.setWindowTitle("Horizon GUI")

        self._createToolBars()
        self._connectGuiActions()


    def newFile(self):
        # Logic for creating a new file goes here...
        self.text_widget.setText("<b>File > New</b> clicked")

    def openFile(self, file):

        with open(file, 'rb') as f:
            loaded_horizon = pickle.load(f)

        loaded_horizon._deserialize()
        self.widget_gui.loadProblem(loaded_horizon)

        self.text_widget.setText("<b>File > Open... </b> clicked")

    def openRecentFile(self, filename):
        # Logic for opening a recent file goes here...
        self.text_widget.setText(f"<b>{filename}</b> opened")

    def saveFile(self):
        # Logic for saving a file goes here...
        self.widget_gui.horizon_receiver.save()
        self.text_widget.setText("<b>File > Save</b> clicked")

    def helpContent(self):
        # Logic for launching help goes here...
        self.text_widget.setText("<b>Help > Help Content...</b> clicked")

    def about(self):
        # Logic for showing an about dialog content goes here...
        self.text_widget.setText("<b>Help > About...</b> clicked")

    def changeSpinBoxNodesSettings(self):
        # getting current value of spin box
        current = self.nodes_input.value()

        # setting this value to second spin box
        self.nodes_input_setting.setValue(current)

        # method called after editing finished

    def changeSpinBoxNodes(self):
        # getting current value of spin box
        current = self.nodes_input_setting.value()

        # setting this value to the first spin box
        self.nodes_input.setValue(current)

    def openSettings(self):

        self.settings_widget = on_destroy_signal_window.DestroySignalWindow()
        self.settings_layout = QGridLayout()

        nodes_input_label = QLabel("Number of nodes:")
        # state_variables_label = QLabel("Add state variable:")
        self.nodes_input_setting = QSpinBox()
        self.nodes_input_setting.setRange(0,999)
        self.nodes_input_setting.setValue(self.nodes_input.value())
        # state_variables = QLineEdit()
        # state_variables_button = QPushButton("Add")
        # initial_state_var = box_state_var.BoxStateVar()
        self.settings_layout.addWidget(nodes_input_label, 0, 0)
        # self.settings_layout.addWidget(state_variables_label, 1, 0)
        self.settings_layout.addWidget(self.nodes_input_setting, 0, 1)
        # self.settings_layout.addWidget(state_variables, 1, 1)
        # self.settings_layout.addWidget(state_variables_button, 1, 2)
        # self.settings_layout.addWidget(initial_state_var, 2, 0, 3, 1)

        self.nodes_input_setting.valueChanged.connect(self.changeSpinBoxNodes)
        self.nodes_input.setDisabled(True)

        self.settings_widget.setLayout(self.settings_layout)
        self.settings_widget.show()
        self.settings_widget.returnDestroyed.connect(partial(self.nodes_input.setDisabled, False))

    def _connectActions(self):
        # Connect File actions
        self.newAction.triggered.connect(self.newFile)
        self.openAction.triggered.connect(self.openFile)
        self.saveAction.triggered.connect(self.saveFile)
        self.exitAction.triggered.connect(self.close)
        # Connect Edit actions
        self.settingAction.triggered.connect(self.openSettings)
        # Connect Help actions
        self.helpContentAction.triggered.connect(self.helpContent)
        self.aboutAction.triggered.connect(self.about)
        # Connect Open Recent to dynamically populate it
        self.openRecentMenu.aboutToShow.connect(self.populateOpenRecent)

        self.starting_screen.start_sig.connect(self.initializeGui)

    def _connectGuiActions(self):
        # This is amazing, I can connect a custom signal from a widget to the main window here
        # basically horizonLine in widget1 sends a signal with a msg, which I connect to the status bar
        self.widget_gui.constraintLine.repeated_fun.connect(self.writeInStatusBar)
        self.widget_gui.costfunctionLine.repeated_fun.connect(self.writeInStatusBar)
        self.widget_gui.generic_sig.connect(self.writeInStatusBar)

    def _createContextMenu(self):
        # Setting contextMenuPolicy
        self.text_widget.setContextMenuPolicy(Qt.ActionsContextMenu)
        # Populating the widget with actions
        self.text_widget.addAction(self.newAction)
        self.text_widget.addAction(self.openAction)
        self.text_widget.addAction(self.saveAction)

    def _createActions(self):
        # Creating action using the first constructor
        self.newAction = QAction(self)
        self.newAction.setText("&New")
        self.newAction.setShortcut("Ctrl+N")
        newTip = "Create a new horizon"
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

        self.helpContentAction = QAction("&Help Content", self)

        self.aboutAction = QAction("&About", self)

        self.settingAction = QAction("@Settings", self)

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
        editMenu = menuBar.addMenu("&Settings")
        # helpMenu = menuBar.addMenu(QIcon("../resources/info.svg"), "&Dio")
        saveMenu = menuBar.addMenu(self.style().standardIcon(QStyle.SP_DialogSaveButton), "&Save")
        # aboutMenu = menuBar.addMenu("&Menu")
        newMenu = menuBar.addMenu('&New')

        newMenu.addAction(self.newAction)

        fileMenu.addAction(self.openAction)
        # Adding an Open Recent submenu
        self.openRecentMenu = fileMenu.addMenu("Open Recent")
        fileMenu.addAction(self.saveAction)

        fileMenu.addAction(self.exitAction)
        fileMenu.addSeparator()
        fileMenu.addAction(self.aboutAction)

        editMenu.addAction(self.settingAction)
        # porcoMenu = aboutMenu.addMenu("Porco")
        # porcoMenu.addAction("il cristo")
        # porcoMenu.addAction("dio")
        # porcoMenu.addAction("gesu")


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
        # fileToolBar = self.addToolBar("File")
        # fileToolBar.addAction(self.newAction)
        # fileToolBar.addAction(self.openAction)
        # fileToolBar.addAction(self.saveAction)
        # fileToolBar.setMovable(False)


        # Using a QToolBar object
        editToolBar = QToolBar("Edit", self)
        self.addToolBar(editToolBar)
        self.nodes_label = QLabel("Nodes in Horizon: ")
        self.nodes_input = QSpinBox()
        self.nodes_input.setRange(0, 999)
        editToolBar.addWidget(self.nodes_label)
        editToolBar.addWidget(self.nodes_input)


        self._connectToolBars()
        # Using a QToolBar object and a toolbar area
        # helpToolBar = QToolBar("Help", self)
        # self.addToolBar(Qt.LeftToolBarArea, helpToolBar)

    def _connectToolBars(self):
        self.nodes_input.valueChanged.connect(self.widget_gui.setBoxNodes)

if __name__ == '__main__':

    app = QApplication(sys.argv)
    gui = HorizonGUI()

    gui.show()
    sys.exit(app.exec_())


    # with open(ROOT_DIR + '/try.pkl', 'rb') as f:
    #     nodes = pickle.load(f)
    #
    # print(nodes)
