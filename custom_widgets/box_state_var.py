from PyQt5.QtWidgets import QTableWidget, QTableWidgetItem, QAbstractItemView, QHeaderView
from PyQt5.QtCore import QRect


class BoxStateVar(QTableWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        # self.setGeometry(QRect(170, 750, 461, 181))
        # self.setObjectName("TableSV")
        self.setShowGrid(False)
        self.setSelectionBehavior(QAbstractItemView.SelectRows)
        # tableWidget.setStyleSheet('QTableView::item {border-right: 1px solid #d6d9dc;}')
        self.setColumnCount(2)
        self.setRowCount(0)

        self.verticalHeader().setVisible(False)

        header = self.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.Stretch)
        header.setSectionResizeMode(1, QHeaderView.ResizeToContents)

        # row_pos = self.rowCount()
        # self.insertRow(row_pos)
        # self.setItem(row_pos, 0, QTableWidgetItem('penis'))

        # rowPosition = self.rowCount()
        # self.addRow(rowPosition)
        # self.setItem(rowPosition, 0, QTableWidgetItem("text1"))

        # var_name_item = QTableWidgetItem()
        # var_name_item.setText('State Variable')
        # self.setHorizontalHeaderItem(0, var_name_item)
        # dim_item = QTableWidgetItem()
        # self.setHorizontalHeaderItem(1, dim_item)
        # dim_item.setText('Dimension')
        # usage_item = QTableWidgetItem()
        # self.setHorizontalHeaderItem(2, usage_item)
        # usage_item.setText('Usage')
