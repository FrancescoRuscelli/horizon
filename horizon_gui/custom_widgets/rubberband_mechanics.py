# def mousePressEvent(self, event):
#     self.origin = event.pos()
#     self.rubberband.setGeometry(QRect(self.origin, QSize()))
#     self.rubberband.show()
#     QWidget.mousePressEvent(self, event)
#
# def mouseMoveEvent(self, event):
#     if self.rubberband.isVisible():
#         self.rubberband.setGeometry(QRect(self.origin, event.pos()).normalized())
#     QWidget.mouseMoveEvent(self, event)
#
# def mouseReleaseEvent(self, event):
#         if self.rubberband.isVisible():
#             self.rubberband.hide()
#             selected = []
#             rect = self.rubberband.geometry()
#             if self.ct_tab.widget(self.ct_tab.currentIndex()) is not None:
#                 for child in self.ct_tab.widget(self.ct_tab.currentIndex()).findChildren(QCheckBox):
#                     # get absolute geometry and then map it to self (HorizonLine)
#                     # the problem was that .geometry() returns the position w.r.t. the parent:
#                     # I was comparing the geometry of the rubberband taken in the HorizonLine's coordinates with
#                     # the geometry of the QCheckBox taken in the QGridLayout's coordinates.
#
#                     gp = child.mapToGlobal(QPoint(0, 0))
#                     b_a = self.mapFromGlobal(gp)
#
#                     # todo check if possible, right now im subtracting a small value to make the geometry of the checkbox smaller
#                     if rect.intersects(QRect(b_a.x(), b_a.y(), child.geometry().width()-20, child.geometry().height())):
#                         selected.append(child)
#
#             for elem in selected:
#                 if event.button() == Qt.RightButton:
#                     elem.setChecked(False)
#                 elif event.button() == Qt.LeftButton:
#                     elem.setChecked(True)
#
#         QWidget.mouseReleaseEvent(self, event)