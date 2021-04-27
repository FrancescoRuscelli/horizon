import sys
from functools import partial

from PyQt5.QtWidgets import (QMainWindow, QApplication, QSplitter, QCheckBox, QGridLayout, QGroupBox, QLabel, QToolBar, QAction, qApp,
                             QMenu, QPushButton, QRadioButton, QVBoxLayout, QHBoxLayout, QWidget, QSlider, QDesktopWidget, QMenuBar, QStyle,
                             QSpinBox, QStyleFactory, QFrame, QTabWidget, QProgressBar, QDialog, QButtonGroup, QLayout, QLineEdit, QTextEdit, QTableWidgetItem,
                             QRubberBand, QComboBox, QScrollArea, QTableWidget, QCompleter)

from PyQt5.QtGui import QIcon, QPixmap, QKeyEvent, QPalette, QColor, QDrag, QTextCursor, QPainter, QCursor, QBrush, QTextCharFormat, QSyntaxHighlighter, QFont
from PyQt5.QtCore import Qt, pyqtSignal, QRect, QMimeData, QPoint, QSize, QRegExp, pyqtSlot

from widget1_ui import Ui_Form
from custom_functions import highlighter
from custom_widgets import horizon_line
import qrc_resources

class DraggableLabel(QLabel):

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.drag_start_position = event.pos()

    def mouseMoveEvent(self, event):
        if not (event.buttons() & Qt.LeftButton):
            return
        if (event.pos() - self.drag_start_position).manhattanLength() < QApplication.startDragDistance():
            return

        drag = QDrag(self)
        mimedata = QMimeData()
        mimedata.setText(self.text())
        drag.setMimeData(mimedata)
        pixmap = QPixmap(self.size())
        painter = QPainter(pixmap)
        painter.drawPixmap(self.rect(), self.grab())
        painter.end()
        drag.setPixmap(pixmap)
        drag.setHotSpot(event.pos())
        drag.exec_(Qt.CopyAction | Qt.MoveAction)

class DropLabel(QLabel):
    def __init__(self, *args, **kwargs):
        QLabel.__init__(self, *args, **kwargs)
        self.setAcceptDrops(True)

    def dragEnterEvent(self, event):
        if event.mimeData().hasText():
            event.acceptProposedAction()

    def dropEvent(self, event):
        pos = event.pos()
        text = event.mimeData().text()
        self.setText(text)
        event.acceptProposedAction()

    def dragEnterEvent(self,event):
        if event.mimeData().hasFormat("text/plain"):
            event.accept()
        else:
            event.ignore()

    def dropEvent(self,event):
        self.setText(event.mimeData().text())

# class AnotherWindow(QWidget):
#     """
#     This "window" is a QWidget. If it has no parent, it
#     will appear as a free-floating window as we want.
#     """
#     def __init__(self):
#         super().__init__()
#         layout = QVBoxLayout()
#         self.label = QLabel("Another Window")
#         layout.addWidget(self.label)
#         self.setLayout(layout)

class ConstraintBar(QWidget):
    def __init__(self, *args, **kwargs):
        QWidget.__init__(self, *args, **kwargs)

        hbox = QHBoxLayout(self)
        main = QFrame(self)
        main.setFrameShape(QFrame.StyledPanel)

        self.splitter = QSplitter(Qt.Horizontal)
        # self.splitter.setChildrenCollapsible(False)
        self.splitter.addWidget(main)
        self.splitter.setStretchFactor(1, 1)
        self.splitter.setSizes([50, 50])
        hbox.addWidget(self.splitter)
        self.setLayout(hbox)
        QApplication.setStyle(QStyleFactory.create('Cleanlooks'))
        self.setGeometry(100, 100, 300, 50)
        self.setWindowTitle('QtGui.QSplitter')

    def addBar(self):
        bar = QFrame(self)
        bar.setFrameShape(QFrame.StyledPanel)
        bar.setMinimumWidth(100)
        self.splitter.addWidget(bar)

class Widget1(QWidget, Ui_Form):
    invalid_sv = pyqtSignal(str)

    def __init__(self):
        super(Widget1, self).__init__()

        N = 20

        self.setupUi(self)
        self._connectActions()
        self.SVNodeInput.setRange(-N, 0)
        self.TableSV.setEditTriggers(QTableWidget.NoEditTriggers)

        # drag and drop stuff
        # label = DropLabel("drop there", self)
        # label.setGeometry(190, 65, 100, 100)
        # label_to_drag = DraggableLabel("drag this", self)

        self.horizonLine = horizon_line.HorizonLine(self.PRB)

        # self.gridLayoutWidget = QtWidgets.QWidget(self.PRB)

        self.ct_layout = QHBoxLayout(self.CT)
        self.ct_entry = self.setuCTEditor(self.PRB)

        self.CTButton.clicked.connect(partial(self.addConstraint))

    @pyqtSlot()
    def on_invalid_sv(self, str):
        self.invalid_sv.emit(str)

    def addConstraint(self):
        name = self.CTNameInput.text()
        self.horizonLine.addConstraintToHorizon(name)

    def setuCTEditor(self, parent):
        font = QFont()
        font.setFamily('Courier')
        font.setFixedPitch(True)
        font.setPointSize(10)

        self.highlighter = highlighter.Highlighter(self.CTFunctionInput.document()) #with QLineEdit doesn't work, so I had to override QTextEdit

        self.ct_keywords = list()
        self.completer = QCompleter(self.ct_keywords)
        self.completer.setCaseSensitivity(Qt.CaseInsensitive)
        self.completer.setWrapAround(False)
        self.CTFunctionInput.setCompleter(self.completer)

    def addStateVariable(self):

        sv_list = [] # todo horizon.getStateVariables()
        self.sv_name = self.SVNameInput.text()
        self.sv_dim = self.SVDimInput.value()
        self.sv_nodes = self.SVNodeInput.value()

        if self.sv_name == "":
            self.on_invalid_sv("Empty Value Not Allowed")
            self.SVNameInput.setFocus()
        elif self.sv_name in sv_list:
            self.on_invalid_sv("State Variable Already Inserted")
            self.SVNameInput.setFocus()
        else:
            print('state variable {} added.'.format(self.sv_name))
            print('dimension: {}'.format(self.sv_dim))
            print('previous node: {}'.format(self.sv_nodes))
            self._addRow()
            self.highlighter.addKeyword(self.sv_name)
            self.ct_keywords.append('{}'.format(self.sv_name))
            model = self.completer.model()
            model.setStringList(self.ct_keywords)

            # todo horizon.setStateVariable()

    def _connectActions(self):

        self.SVAddButton.clicked.connect(self.addStateVariable)

    def _addRow(self):
        row_pos = self.TableSV.rowCount()
        self.TableSV.insertRow(row_pos)

        combo_box = QComboBox()
        scroll = QScrollArea()

        widget = QWidget()  # Widget that contains the collection of Vertical Box
        vbox = QVBoxLayout()  # The Vertical Box that contains the Horizontal Boxes of  labels and buttons

        widget.setLayout(vbox)

        for i in range(1, 50):
            object = QLabel("TextLabel")
            vbox.addWidget(object)

        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setWidgetResizable(True)
        scroll.setWidget(widget)

        self.TableSV.setItem(row_pos, 0, QTableWidgetItem(self.sv_name))
        self.TableSV.setItem(row_pos, 1, QTableWidgetItem(str(self.sv_dim)))
        self.TableSV.setCellWidget(row_pos, 2, scroll)

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
        self.widget1.horizonLine.invalid_ct.connect(self.writeInStatusBar)
        self.widget1.invalid_sv.connect(self.writeInStatusBar)

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