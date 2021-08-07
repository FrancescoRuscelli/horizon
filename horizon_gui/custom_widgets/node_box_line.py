from PyQt5.QtWidgets import QWidget, QPushButton, QHBoxLayout, QApplication, QGridLayout, QLabel, QDialog, QTableWidget, QTableWidgetItem
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot
from functools import partial
import sys


class NodeBoxLine(QWidget):
    buttonPressed = pyqtSignal(int)

    def __init__(self, nodes=0, parent=None):
        super().__init__(parent)

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
            if i != self.n_nodes-1:
                n_i.setStyleSheet('background-color: orange;')
            else:
                n_i.setStyleSheet('background-color: red;')
            n_i.setObjectName('node_{}'.format(i))
            n_i.clicked.connect(partial(self.buttonPressed.emit, i))
            self.node_box_layout.addWidget(n_i)


if __name__ == '__main__':


    app = QApplication(sys.argv)
    gui = NodeBoxLine()
    gui.setBoxNodes(4)
    # print(gui.getBoxWidth())
    gui.show()
    sys.exit(app.exec_())