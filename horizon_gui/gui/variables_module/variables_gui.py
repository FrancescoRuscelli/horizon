from PyQt5.QtWidgets import (QGridLayout, QLabel, QPushButton, QHBoxLayout, QWidget, QSizePolicy, QApplication, \
                                QLineEdit, QTableWidget, QHeaderView, QTableWidgetItem, QVBoxLayout)
from PyQt5.QtGui import QPalette
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot
from horizon_gui.custom_widgets import on_destroy_signal_window, highlight_delegate, bounds_line, multi_slider_v1, spinbox_line
from horizon_gui.gui.gui_receiver import horizonImpl
from horizon_gui.gui.variables_module.variables_module_ui import Ui_VariablesGUI
from functools import partial
import sys
import numpy as np

class VariablesGui(QWidget, Ui_VariablesGUI):
    stateVarAdded = pyqtSignal(str)
    genericSignal = pyqtSignal(str)

    def __init__(self, horizon_receiver: horizonImpl, logger=None, parent=None):
        super().__init__(parent)

        self.setupUi(self)
        self.horizon_receiver = horizon_receiver
        self.logger = logger

        self.connectButtons()

    def connectButtons(self):
        self.stateVarAddButton.clicked.connect(partial(self.generateVariable, 'State'))
        self.inputVarAddButton.clicked.connect(partial(self.generateVariable, 'Input'))
        self.singleVarAddButton.clicked.connect(partial(self.generateVariable, 'Single'))
        self.customVarAddButton.clicked.connect(self.openCustomVarOptions)

        self.varTable.itemDoubleClicked.connect(self.openVar)



    def openVar(self, item):

        # regardless of where I click, get the first item of the table
        item = self.varTable.item(self.varTable.row(item), 0)
        n_row = 0
        # create window that emit a signal when closed
        self.var_temp_win = on_destroy_signal_window.DestroySignalWindow()

        # create a layout for the window
        # widget, fromRow, int fromColumn, int rowSpan, int columnSpan, Qt::Alignment alignment = Qt::Alignment()
        self.sv_window_layout = QGridLayout()

        # populate it
        self.label_name = QLabel("name:")
        self.sv_window_layout.addWidget(self.label_name, n_row, 0, 1, 1)

        palette = QPalette()
        palette.setColor(QPalette.Text, Qt.black)

        self.display_name = QLineEdit()
        self.display_name.setFixedWidth(50)
        self.display_name.setText(item.text())
        self.display_name.setDisabled(True)
        self.display_name.setPalette(palette)

        self.sv_window_layout.addWidget(self.display_name, n_row, 1, 1, 1)

        self.label_dim = QLabel("dimension:")
        self.sv_window_layout.addWidget(self.label_dim, n_row, 2, 1, 1)

        self.display_dim = QLineEdit()
        self.display_dim.setFixedWidth(20)
        self.display_dim.setText(str(self.horizon_receiver.getVar(item.text())['dim']))
        self.display_dim.setDisabled(True)
        self.display_dim.setPalette(palette)
        self.sv_window_layout.addWidget(self.display_dim, n_row, 3, 1, 1)

        # next row
        n_row = n_row + 1
        self.label_fun = QLabel("var:")
        self.sv_window_layout.addWidget(self.label_fun, n_row, 0, 1, 1)

        self.display_fun = QLineEdit()
        # as width as the vari text
        width = self.display_fun.fontMetrics().width(str(self.horizon_receiver.getVar(item.text())['var']))
        self.display_fun.setText(str(self.horizon_receiver.getVar(item.text())['var']))
        self.display_fun.setDisabled(True)
        # todo magic number?
        self.display_fun.setMinimumWidth(width + 5)
        self.display_fun.setAlignment(Qt.AlignCenter)
        self.display_fun.setPalette(palette)
        self.sv_window_layout.addWidget(self.display_fun, n_row, 1, 1, 3)

        # next row
        n_row = n_row + 1
        self.label_type = QLabel("type:")
        self.sv_window_layout.addWidget(self.label_type, n_row, 0, 1, 1)

        self.type_var = QLineEdit()
        self.type_var.setText(str(self.horizon_receiver.getVar(item.text())['type']))
        self.type_var.setDisabled(True)
        self.type_var.setAlignment(Qt.AlignCenter)
        self.type_var.setPalette(palette)
        self.sv_window_layout.addWidget(self.type_var, n_row, 1, 1, 3)

        if self.horizon_receiver.getVar(item.text())['type'] == 'Custom':
            # next row
            n_row = n_row + 1
            self.label_nodes = QLabel("active nodes:")
            self.sv_window_layout.addWidget(self.label_nodes, n_row, 0, 1, 1)

            node_list = str(self.horizon_receiver.getVar(item.text())['var'].getNodes())
            self.nodes_var = QLineEdit("active nodes:")
            width = self.display_fun.fontMetrics().width(node_list)
            self.nodes_var.setMinimumWidth(width + 5)
            self.nodes_var.setText(node_list)
            self.nodes_var.setDisabled(True)
            self.nodes_var.setAlignment(Qt.AlignCenter)
            self.nodes_var.setPalette(palette)
            self.sv_window_layout.addWidget(self.nodes_var, n_row, 1, 1, 3)

        # next row
        n_row = n_row + 1
        self.label_usages = QLabel("usages:")
        self.sv_window_layout.addWidget(self.label_usages, n_row, 0, 1, 4)

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

        # next row
        n_row = n_row + 1
        self.sv_window_layout.addWidget(self.usages_table, n_row, 0, 1, 4)
        # # todo add usages (add constraint logic and check if constraint depends on variable)
        # TODO REMOVE CT ADD FUNCTION
        # get only active function
        active_fun_list = [i for i in [(elem['str'], elem['active']) for name, elem in
                                       self.horizon_receiver.getFunctionDict().items() if 'active' in elem] if i[1]]

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

        n_row = n_row + 1
        var = self.horizon_receiver.getVar(item.text())['var']
        var_dim = self.horizon_receiver.getVar(item.text())['dim']
        active_nodes = var.getNodes()

        n_nodes = self.horizon_receiver.getNodes()+1
        lb = var.getLowerBounds()
        ub = var.getUpperBounds()

        lb_matrix = np.reshape(lb, (var_dim, len(active_nodes)), order='F')
        ub_matrix = np.reshape(ub, (var_dim, len(active_nodes)), order='F')

        disabled_nodes = [node for node in range(n_nodes) if node not in active_nodes]

        # adding bounds
        n_row = n_row + 1
        self.var_bounds = bounds_line.BoundsLine(item.text(), n_nodes, var_dim, disabled_nodes, [lb_matrix, ub_matrix])
        self.var_bounds.lbChanged.connect(partial(self.on_var_lb_changed, item.text()))
        self.var_bounds.ubChanged.connect(partial(self.on_var_ub_changed, item.text()))

        self.sv_window_layout.addWidget(self.var_bounds, n_row, 0, 1, 4)

        # adding intial guess
        n_row = n_row + 1
        initial_guess_colors = dict(base='lightblue', selected='blue', background='blue', border='blue')
        initial_guess = var.getInitialGuess()
        # reshape it in a matrix with row (dim) and columns (nodes)
        ig_matrix = np.reshape(initial_guess, (var_dim, len(active_nodes)), order='F')

        self.var_initial_guess = spinbox_line.SpinboxLine('Initial Guess', n_nodes, var_dim, disabled_nodes, initial_values=ig_matrix, colors=initial_guess_colors)
        self.var_initial_guess.valueChanged.connect(partial(self.on_var_ig_changed, item.text()))
        self.sv_window_layout.addWidget(self.var_initial_guess, n_row, 0, 1, 4)

        n_row = n_row + 1
        self.remove_var_button = QPushButton('Remove Variable')
        # self.edit_button.setStyleSheet("background-color : lightblue")
        self.remove_var_button.clicked.connect(partial(self.removeVar, item))

        self.sv_window_layout.addWidget(self.remove_var_button, n_row, 0)

        self.var_temp_win.setWindowTitle(item.text())
        self.var_temp_win.setLayout(self.sv_window_layout)

        # show window
        self.var_temp_win.show()

        # temp_win emits a message when it is closed, before being destroyed.
        # remember that I have to manually destroy it
        # self.temp_win.returnDestroyed.connect(self.yieldHighlighter)

    def removeVar(self, item):
        flag = self.horizon_receiver.removeStateVariable(item.text())

        if flag:
            self.var_temp_win.close()
            self.varTable.removeRow(self.varTable.row(item))

    def generateVariable(self, var_type, nodes=None):

        default_dim = 1
        default_past_node = 0

        var_name = self.varNameInput.text()
        var_dim = self.varDimInput.value()
        var_offset = self.varOffsetInput.value()

        flag, signal = self.horizon_receiver.createVariable(var_type, var_name, var_dim, var_offset, nodes)

        if flag:

            self.addVariableToGui(var_name)
            self.varNameInput.clear()
            self.varDimInput.setValue(default_dim)
            self.varOffsetInput.setValue(default_past_node)

            self.stateVarAdded.emit(var_name)
            self.genericSignal.emit(signal)

            if self.logger:
                self.logger.info(signal)

        else:
            self.varNameInput.setFocus()
            self.genericSignal.emit(signal)
            if self.logger:
                self.logger.warning('main_interface {}'.format(signal))

    def openCustomVarOptions(self):

        self.box = QWidget()
        layout_box = QVBoxLayout(self.box)

        options = dict()
        options['background_color'] = Qt.darkCyan
        options['slice_color'] = Qt.darkMagenta
        options['minmax_color'] = Qt.darkRed
        options['ticks_color'] = Qt.darkGreen
        node_selector = multi_slider_v1.QMultiSlider(slider_range=[0, self.horizon_receiver.getNodes(), 1], options=options, number_bar=True)
        layout_box.addWidget(node_selector)

        widget_dialog = QWidget()
        widget_dialog_layout = QHBoxLayout(widget_dialog)
        no_button = QPushButton('Cancel')
        yes_button = QPushButton('Create')


        widget_dialog_layout.addWidget(no_button)
        widget_dialog_layout.addWidget(yes_button)

        layout_box.addWidget(widget_dialog)

        no_button.clicked.connect(self.box.close)
        yes_button.clicked.connect(partial(self.assembleCustomVar, node_selector))
        yes_button.clicked.connect(self.box.close)
        self.box.show()

    def assembleCustomVar(self, node_selector):

        nodes = node_selector.getRanges()
        self.generateVariable('Custom', nodes)

    def addVariableToGui(self, sv_name):

        self._addRowToSVTable(sv_name)

    def _addRowToSVTable(self, name):

        row_pos = self.varTable.rowCount()
        self.varTable.insertRow(row_pos)

        self.varTable.setItem(row_pos, 0, QTableWidgetItem(name))
        self.varTable.setItem(row_pos, 1, QTableWidgetItem(str(self.horizon_receiver.getVar(name)['dim'])))

        item_type = QTableWidgetItem(str(self.horizon_receiver.getVar(name)['type']))
        item_type.setTextAlignment(Qt.AlignCenter)
        self.varTable.setItem(row_pos, 2, item_type)

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

    @pyqtSlot(str, int, list)
    def on_var_lb_changed(self, var_name, node, lims):
        self.horizon_receiver.updateVarLb(var_name, lims, node)

    @pyqtSlot(str, int, list)
    def on_var_ub_changed(self, var_name, node, lims):
        self.horizon_receiver.updateVarUb(var_name, lims, node)

    @pyqtSlot(str, int, list)
    def on_var_ig_changed(self, var_name, node, lims):
        self.horizon_receiver.updateVarIg(var_name, lims, node)


if __name__ == '__main__':

    hr = horizonImpl(10)
    app = QApplication(sys.argv)
    gui = VariablesGui(hr)
    gui.show()

    sys.exit(app.exec_())
