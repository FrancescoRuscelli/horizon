from PyQt5.QtWidgets import QWidget, QPushButton, QHBoxLayout, QApplication, QGridLayout, QLabel, QDialog, QTableWidget, QTableWidgetItem
from PyQt5.QtCore import Qt
from functools import partial
import sys
from horizon_gui.custom_widgets.on_destroy_signal_window import DestroySignalWindow

class NodeBoxLine(QWidget):
    def __init__(self, horizon_receiver, nodes=0, parent=None):
        super().__init__(parent)

        self.horizon_receiver = horizon_receiver
        self.node_box_layout = QHBoxLayout(self)
        margins = self.node_box_layout.contentsMargins()
        margins.setBottom(0)
        self.node_box_layout.setContentsMargins(margins)
        self.node_box_layout.setSpacing(0)

        self.setBoxNodes(nodes)

    def setBoxNodes(self, nodes):
        self.n_nodes = nodes
        self._updateBoxNodes()

    def getBoxWidth(self):
        # simply the width of the window divided by nodes
        return self.width()/self.n_nodes

    def _updateBoxNodes(self):

        # Add nodes to the layout
        for i in reversed(range(self.node_box_layout.count())):
            self.node_box_layout.itemAt(i).widget().deleteLater()
            self.node_box_layout.removeItem(self.node_box_layout.itemAt(i))

        for i in range(self.n_nodes):
            n_i = QPushButton("{}".format(i))
            n_i.setStyleSheet('background-color: orange;')
            n_i.setObjectName('node_{}'.format(i))
            n_i.clicked.connect(partial(self.openNodeBox, i))
            self.node_box_layout.addWidget(n_i)

    def openNodeBox(self, node):

        self.info_box = DestroySignalWindow() #on_destroy_signal_window.DestroySignalWindow()
        self.info_box.show()
        info_box_layout = QGridLayout(self.info_box)

        label_node = QLabel('Node {}'.format(node))
        label_node.setAlignment(Qt.AlignCenter)
        info_box_layout.addWidget(label_node, 0, 0)

        label_title_state_var = QLabel('State Variables:')
        info_box_layout.addWidget(label_title_state_var, 1, 0)
        table_vars = QTableWidget()
        table_vars.setColumnCount(4)
        table_vars.setHorizontalHeaderLabels(['Var Impl', 'Lower Bounds', 'Upper Bounds', 'Initial Guess'])

        info_box_layout.addWidget(table_vars, 1, 1)

        for elem in self.horizon_receiver.getInfoAtNodes(node):
            rowPosition = table_vars.rowCount()
            table_vars.insertRow(rowPosition)
            table_vars.setItem(rowPosition, 0, QTableWidgetItem((str(elem['var']))))
            table_vars.setItem(rowPosition, 1, QTableWidgetItem((str(elem['lb']))))
            table_vars.setItem(rowPosition, 2, QTableWidgetItem((str(elem['ub']))))
            table_vars.setItem(rowPosition, 3, QTableWidgetItem((str(elem['w0']))))


        label_title_constraint = QLabel('Constraint:')
        info_box_layout.addWidget(label_title_constraint, 2, 0)
        table_constr = QTableWidget()
        table_constr.setColumnCount(3)
        table_constr.setHorizontalHeaderLabels(['Constraint', 'Lower Bounds', 'Upper Bounds'])
        info_box_layout.addWidget(table_constr, 2, 1)




if __name__ == '__main__':


    app = QApplication(sys.argv)
    gui = NodeBoxLine()
    gui.setBoxNodes(4)
    # print(gui.getBoxWidth())
    gui.show()
    sys.exit(app.exec_())