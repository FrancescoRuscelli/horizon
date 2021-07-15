import sys
from functools import partial
import pickle
from PyQt5.QtWidgets import (QMainWindow, QApplication, QLabel, QToolBar, QAction, qApp, QLineEdit,
                             QMenu, QPushButton, QVBoxLayout, QGridLayout, QWidget, QMenuBar, QStyle,
                             QSpinBox, QProgressBar, QFileDialog, QDialog)

from PyQt5.QtGui import QIcon
from PyQt5.QtCore import Qt, QSettings, QDir

from horizon_gui.custom_widgets import starting_screen, main_interface, on_destroy_signal_window
from horizon_gui.gui import gui_receiver
import logging
from horizon_gui.definitions import ROOT_DIR

# todo:
# SCREEN WITH ERRORS from casadi and python                           DONE
# EDIT CONSTRAINT NOW IS WRONG (visualize function and not string)    DONE
# make so that, when nodes number changes, the slices adapt (basically they don't have to exceed the maximum and the minimum) DONE
# in state variable box: UNWRAP function! and check highlighter
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
# STRANGE BEHAVIOUR WHEN LOADING A HORIZON_PROBLEM WITH MANY NODES: LINE LOSING ITS WIDTH
# WHAT IF NODES ARE 0 ---> WHAT TO DO???
# STRANGE BEHAVIOUR WHEN ADDING REMOVING AND ADDING AGAIN SAME FUNCTION TO HORIZONLINE
# STRANGE BEHAVIOUR WHEN ADDING FUNCTION (DEPENDS_ON ERROR)
# WHEN RELOADING, MARGINS OF MULTI_SLIDER IS WRONG


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
        self.logger = logging.getLogger('logger')
        self.logger.setLevel(logging.DEBUG)

        # settings
        self.current_save_file = None
        self.recent_files = list()
        self.max_recent_files = 3

        self.starting_screen = starting_screen.StartingScreen()
        self.createSetup()

        self.setCentralWidget(self.starting_screen)

        self._createActions()
        self._createMenuBar()
        # self._createContextMenu()
        # self._createButton()
        self._connectActions()
        self._createStatusBar()

        self.showMaximized()

        self.loadSettings()

    def createSetup(self):

        self.setup_opt = dict()
        self.setup_opt['nodes'] = 0 # todo make it 0

    def setNodes(self, nodes):
        self.setup_opt['nodes'] = nodes

    def initializeGui(self, horizon_receiver):

        self.current_horizon = horizon_receiver
        # generate a layout for a widget, I can populate the layout with more widget
        self.main_layout = QVBoxLayout()

        # self.main_layout.addWidget(self.red_widget,1)

        self.widget_gui = main_interface.MainInterface(horizon_receiver, logger=self.logger)
        self.main_layout.addWidget(self.widget_gui)
        self.widget_gui.splitter.setStretchFactor(1, 2)

        # generate a main widget to assign to centralwidget
        self.main_widget = QWidget()
        self.main_widget.setLayout(self.main_layout)
        self.main_widget.adjustSize()
        self.setCentralWidget(self.main_widget)
        self.setWindowTitle("Horizon GUI")

        self._createToolBars()
        self._connectGuiActions()


    def newFile(self):
        # Logic for creating a new file goes here...
        horizon_receiver = gui_receiver.horizonImpl(self.setup_opt['nodes'], logger=self.logger)
        self.initializeGui(horizon_receiver)

    def updateSettings(self, recent_file):

        if recent_file not in self.recent_files:
            self.settings = QSettings(ROOT_DIR + "/settings/gui_settings.ini", QSettings.IniFormat)
            # for saving a value to the ini
            self.settings.beginGroup('recent_files')
            # todo mechanics for ignoring file if already in recent_files
            if len(self.settings.allKeys()):
                i = len(self.settings.allKeys())
                if i < self.max_recent_files:
                    self.settings.setValue('file' + str(i + 1), recent_file)
                else:

                    for k in range(1, i):
                        self.settings.setValue('file' + str(k), self.settings.value('file' + str(k + 1)))
                    self.settings.setValue('file' + str(i), recent_file)
            else:
                self.settings.setValue('file1', recent_file)
            self.settings.endGroup()



    def loadSettings(self):

        self.settings = QSettings(ROOT_DIR + "/settings/gui_settings.ini", QSettings.IniFormat)
        # for saving a value to the ini
        self.settings.beginGroup('recent_files')

        self.settings.allKeys()
        for elem in self.settings.allKeys():
            self.recent_files.append(self.settings.value(elem))

        self.settings.endGroup()

    def openFileFromDialog(self):

        options = QFileDialog.Options()
        # options |= QFileDialog.DontUseNativeDialog
        file_horizon, _ = QFileDialog.getOpenFileName(self, "QFileDialog.getOpenFileName()", "",
                                                "Pickle Files (*.pickle)", options=options)
        if file_horizon:
            self.openFile(file_horizon)
        else:
            self.logger.warning('File not found. Failed loading horizon problem.')

    def openFile(self, file_path):

        self.updateSettings(file_path)
        self.logger.info('Opening file: {}'.format(file_path))
        self.loadHorizon(file_path)
        self.writeInStatusBar('Opening Horizon problem!')

    def openRecentFile(self, filename):
        # Logic for opening a recent file goes here...
        self.openFile(filename)

    def saveFile(self):

        if self.current_save_file is None:
            self.saveAsFile()
        else:
            self.saveHorizon(self.current_save_file, self.current_horizon)

        self.updateSettings(self.current_save_file)
        self.writeInStatusBar('SAVING!')

    def saveAsFile(self):


        dlg = QFileDialog()
        dlg.setDefaultSuffix('pickle')
        dlg.setFilter(dlg.filter() | QDir.Hidden)
        dlg.setNameFilters(['Pickle Files (*.pickle)'])

        dlg.setAcceptMode(QFileDialog.AcceptSave)

        if dlg.exec_() == QDialog.Accepted:
            file_path = dlg.selectedFiles()[0]
        else:
            file_path = None


        if file_path:

            self.logger.info('Saving to File: {}'.format(file_path))
            self.current_save_file = file_path
            self.saveHorizon(self.current_save_file, self.current_horizon)

            self.writeInStatusBar('SAVING!')

    def helpContent(self):
        # Logic for launching help goes here...
        self.text_widget.setText("<b>Help > Help Content...</b> clicked")

    def about(self):
        # Logic for showing an about dialog content goes here...
        self.text_widget.setText("<b>Help > About...</b> clicked")

    def loadHorizon(self, file_path):

        with open(file_path, 'rb') as f:
            loaded_horizon = pickle.load(f)

        loaded_horizon.deserialize()

        self.logger.info('Loaded Horizon: {}'.format(loaded_horizon))
        self.initializeGui(loaded_horizon)

    def saveHorizon(self, file_path, horizon_to_save):

        # print('way_before:', horizon_to_save.casadi_prb.state_var_container.state_var)
        # print('way_before:', horizon_to_save.casadi_prb.state_var_container.state_var_prev)
        # print('way_before:', horizon_to_save.casadi_prb.state_var_container.state_var_impl)
        # print('way_before:', [elem.getFunction() for elem in horizon_to_save.casadi_prb.function_container.cnstr_container.values()])
        # print('way_before:', horizon_to_save.casadi_prb.function_container.cnstr_impl)
        # print('way_before:', [elem.getFunction() for elem in horizon_to_save.casadi_prb.function_container.costfun_container.values()])
        # print('way_before:', horizon_to_save.casadi_prb.function_container.costfun_impl)
        # print('way_before:', horizon_to_save.casadi_prb.function_container.state_vars.state_var)
        # print('way_before:', horizon_to_save.casadi_prb.prob['f'])
        # print('way_before:', horizon_to_save.casadi_prb.prob['x'])
        # print('way_before:', horizon_to_save.casadi_prb.prob['g'])
        # print('way_before', horizon_to_save.sv_dict)
        # print('way_before', horizon_to_save.fun_dict)

        # todo is this intelligent?
        # serializing and deserializing everytime is stupid right?
        # make a copy of itself and serialize that one!
        horizon_to_save.serialize()

        # print('before:', horizon_to_save.casadi_prb.state_var_container.state_var)
        # print('before:', horizon_to_save.casadi_prb.state_var_container.state_var_prev)
        # print('before:', horizon_to_save.casadi_prb.state_var_container.state_var_impl)
        # print('before:', [elem.getFunction() for elem in horizon_to_save.casadi_prb.function_container.cnstr_container.values()])
        # print('before:', horizon_to_save.casadi_prb.function_container.cnstr_impl)
        # print('before:', [elem.getFunction() for elem in horizon_to_save.casadi_prb.function_container.costfun_container.values()])
        # print('before:', horizon_to_save.casadi_prb.function_container.costfun_impl)
        # print('before:', horizon_to_save.casadi_prb.function_container.state_vars.state_var)
        # print('before:', horizon_to_save.casadi_prb.prob['f'])
        # print('before:', horizon_to_save.casadi_prb.prob['x'])
        # print('before:', horizon_to_save.casadi_prb.prob['g'])
        # print('before', horizon_to_save.sv_dict)
        # print('before', horizon_to_save.fun_dict)

        # print('var_container is pickleable?', self.pickleable(horizon_to_save.casadi_prb.var_container))
        # print('state_var_container is pickleable?', self.pickleable(horizon_to_save.casadi_prb.state_var_container))
        # print('function_container is pickleable?', self.pickleable(horizon_to_save.casadi_prb.function_container))
        # print('nodes is pickleable?', self.pickleable(horizon_to_save.casadi_prb.nodes))
        # print('solver is pickleable?', self.pickleable(horizon_to_save.casadi_prb.solver))
        # print('prob is pickleable?', self.pickleable(horizon_to_save.casadi_prb.prob))
        # print('sv_dict is pickleable?', self.pickleable(horizon_to_save.sv_dict))
        # print('fun_dict is pickleable?', self.pickleable(horizon_to_save.fun_dict))


        with open(file_path, 'wb') as f:
            pickle.dump(horizon_to_save, f)

        horizon_to_save.deserialize()

        # print('after', horizon_to_save.casadi_prb.state_var_container.state_var)
        # print('after', horizon_to_save.casadi_prb.state_var_container.state_var_prev)
        # print('after', horizon_to_save.casadi_prb.state_var_container.state_var_impl)
        # print('after:', [elem.getFunction() for elem in horizon_to_save.casadi_prb.function_container.cnstr_container.values()])
        # print('after:', horizon_to_save.casadi_prb.function_container.cnstr_impl)
        # print('after:', [elem.getFunction() for elem in horizon_to_save.casadi_prb.function_container.costfun_container.values()])
        # print('after:', horizon_to_save.casadi_prb.function_container.costfun_impl)
        # print('after:', horizon_to_save.casadi_prb.function_container.state_vars.state_var)
        # print('after:', horizon_to_save.casadi_prb.prob['f'])
        # print('after:', horizon_to_save.casadi_prb.prob['x'])
        # print('after:', horizon_to_save.casadi_prb.prob['g'])
        # print('after', horizon_to_save.sv_dict)
        # print('after', horizon_to_save.fun_dict)

    def openSettings(self):

        # todo save also the settings of the GUI
        self.settings_widget = on_destroy_signal_window.DestroySignalWindow()
        self.settings_layout = QGridLayout()

        nodes_input_label = QLabel("Number of nodes:")
        # state_variables_label = QLabel("Add state variable:")
        nodes_input = QSpinBox()
        nodes_input.setRange(0, 999)
        nodes_input.setValue(self.setup_opt['nodes'])
        # state_variables = QLineEdit()
        # state_variables_button = QPushButton("Add")
        # initial_state_var = box_state_var.BoxStateVar()
        self.settings_layout.addWidget(nodes_input_label, 0, 0)
        # self.settings_layout.addWidget(state_variables_label, 1, 0)
        self.settings_layout.addWidget(nodes_input, 0, 1)
        # self.settings_layout.addWidget(state_variables, 1, 1)
        # self.settings_layout.addWidget(state_variables_button, 1, 2)
        # self.settings_layout.addWidget(initial_state_var, 2, 0, 3, 1)
        nodes_input.valueChanged.connect(self.setNodes)
        self.settings_widget.setLayout(self.settings_layout)
        self.settings_widget.show()

    def _connectActions(self):
        # Connect File actions
        self.newAction.triggered.connect(self.newFile)
        self.openAction.triggered.connect(self.openFileFromDialog)
        self.saveAction.triggered.connect(self.saveFile)
        self.saveAsAction.triggered.connect(self.saveAsFile)
        self.exitAction.triggered.connect(self.close)
        # Connect Edit actions
        self.settingAction.triggered.connect(self.openSettings)
        # Connect Help actions
        self.helpContentAction.triggered.connect(self.helpContent)
        self.aboutAction.triggered.connect(self.about)
        # Connect Open Recent to dynamically populate it
        self.openRecentMenu.aboutToShow.connect(self.populateOpenRecent)

        self.starting_screen.start_sig.connect(self.newFile)

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
        self.text_widget.addAction(self.saveAsAction)

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

        self.saveAsAction = QAction("&Save As", self)
        self.saveAsAction.setShortcut("Ctrl+A")

        # exit action
        self.exitAction = QAction(self.style().standardIcon(QStyle.SP_DialogCancelButton), '&Exit', self)
        self.exitAction.setShortcut('Ctrl+Q')
        self.exitAction.setStatusTip('Exit application')
        self.exitAction.triggered.connect(qApp.quit)

        self.helpContentAction = QAction("&Help Content", self)

        self.aboutAction = QAction("&About", self)

        self.settingAction = QAction("&Settings", self)

    def writeInStatusBar(self, msg):
        self.statusbar.showMessage(msg, 10000)

    def populateOpenRecent(self):
        # Step 1. Remove the old options from the menu
        self.openRecentMenu.clear()
        # Step 2. Dynamically create the actions
        actions = []
        for filename in self.recent_files:
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

        fileMenu = QMenu("&File", self)
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
        fileMenu.addAction(self.saveAsAction)
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



        # self._connectToolBars()
        # Using a QToolBar object and a toolbar area
        # helpToolBar = QToolBar("Help", self)
        # self.addToolBar(Qt.LeftToolBarArea, helpToolBar)

    def _connectToolBars(self):
        print('_connectToolBars yet to implement')

    def pickleable(self, obj):
        try:
            pickle.dumps(obj)
        except pickle.PicklingError:
            return False
        return True

if __name__ == '__main__':

    app = QApplication(sys.argv)
    gui = HorizonGUI()

    gui.show()
    sys.exit(app.exec_())


    # with open(ROOT_DIR + '/try.pkl', 'rb') as f:
    #     nodes = pickle.load(f)
    #
    # print(nodes)
