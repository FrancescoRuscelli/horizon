from PyQt5 import QtCore, QtGui, QtWidgets, Qt

class BoxFunction(QtWidgets.QTableWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.setShowGrid(False)
        # tableWidget.setStyleSheet('QTableView::item {border-right: 1px solid #d6d9dc;}')
        self.verticalHeader().setVisible(False)

        self.setColumnCount(2)
        self.setRowCount(0)


        self.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        # self.setDragDropMode(QtWidgets.QAbstractItemView.)
        self.setDefaultDropAction(QtCore.Qt.CopyAction)
        self.setDragEnabled(True)

        header = self.horizontalHeader()
        header.setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
