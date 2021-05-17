from PyQt5.QtWidgets import QTextEdit, QCompleter
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QTextCursor


class LineEdit(QTextEdit):
    returnPressed = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        # textEdit but shaped as a LineEdit. This is to use the properties of TextEdit (QSyntaxHighlighter)
        # single line configuration
        self.setLineWrapMode(QTextEdit.NoWrap)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setFixedHeight(self.document().documentLayout().documentSize().height() + self.height() - self.viewport().height())
        self.setTabChangesFocus(True)
        self._completer = None

    def setCompleter(self, completer):

        if self._completer is not None:
            self._completer.disconnect()

        completer.setWidget(self)
        completer.setCompletionMode(QCompleter.PopupCompletion)
        completer.setCaseSensitivity(Qt.CaseInsensitive)
        self._completer = completer

        self._completer.activated.connect(self.insertCompletion)

    def insertCompletion(self, completion):

        tc = self.textCursor()
        extra = (len(completion) - len(self._completer.completionPrefix()))
        tc.movePosition(QTextCursor.Left)
        tc.movePosition(QTextCursor.EndOfWord)
        if extra > 0:
            tc.insertText(completion[-extra:])
        self.setTextCursor(tc)

    def textUnderCursor(self):

        tc = self.textCursor()
        tc.select(QTextCursor.WordUnderCursor)
        return tc.selectedText()

    def focusInEvent(self, e):
        if self._completer is not None:
            self._completer.setWidget(self)

        super(LineEdit, self).focusInEvent(e)

    def selfClosing(self, e):

        char_list = {'[': ']', '(': ')'}
        closing_char = char_list.get(e.text())

        if closing_char:
            tc = self.textCursor()
            char_pos = tc.position()

            self.insertPlainText(closing_char)
            tc.setPosition(char_pos)
            self.setTextCursor(tc)

    def keyPressEvent(self, e):

        # close parentheses
        self.selfClosing(e)

        if self._completer is not None and self._completer.popup().isVisible():
            # The following keys are forwarded by the completer to the widget.
            if e.key() in (Qt.Key_Enter, Qt.Key_Return, Qt.Key_Escape, Qt.Key_Tab, Qt.Key_Backtab):
                e.ignore()
                # Let the completer do default behavior.
                return

        # this is to prevent Return or Enter key to create new line
        # (since this is a QText disguised as QLine)
        if e.key() in (Qt.Key_Return, Qt.Key_Enter):
            # self.returnPressed.emit() I can also connect it to my custom Signal!!!!
            return

        isShortcut = (e.modifiers() == Qt.ShiftModifier and e.key() == Qt.Key_E)
        if self._completer is None or not isShortcut:
            # Do not process the shortcut when we have a completer.
            super(LineEdit, self).keyPressEvent(e)

        ctrlOrShift = (e.modifiers() == Qt.ControlModifier or e.modifiers() == Qt.ShiftModifier)
        if self._completer is None or (ctrlOrShift and len(e.text()) == 0):
            return

        eow = "~!@#$%^&*()_+{}|:\"<>?,./;'[]\\-="
        hasModifier = (e.modifiers() != Qt.NoModifier) # and not ctrlOrShift
        completionPrefix = self.textUnderCursor()

        if not isShortcut and (hasModifier or len(completionPrefix) < 1): #or len(e.text()) == 0 # or len(completionPrefix) < 1 or e.text()[-1] in eow
            self._completer.popup().hide()
            return

        if completionPrefix != self._completer.completionPrefix():
            self._completer.setCompletionPrefix(completionPrefix)
            self._completer.popup().setCurrentIndex(self._completer.completionModel().index(0, 0))

        cr = self.cursorRect()
        cr.setWidth(self._completer.popup().sizeHintForColumn(0) + self._completer.popup().verticalScrollBar().sizeHint().width())
        self._completer.complete(cr)