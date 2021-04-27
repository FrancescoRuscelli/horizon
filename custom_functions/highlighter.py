from PyQt5.QtCore import Qt, QRegExp
from PyQt5.QtGui import QFont, QSyntaxHighlighter, QTextCharFormat

class Highlighter(QSyntaxHighlighter):
    def __init__(self, parent=None):
        super(Highlighter, self).__init__(parent)

        self.keywordFormat = QTextCharFormat()
        self.keywordFormat.setForeground(Qt.darkGreen)
        self.keywordFormat.setFontWeight(QFont.Bold)

        self.keywordPatterns = list()
        self.highlightingRules = [(QRegExp(pattern), self.keywordFormat) for pattern in self.keywordPatterns]

        # classFormat = QTextCharFormat()
        # classFormat.setFontWeight(QFont.Bold)
        # classFormat.setForeground(Qt.darkMagenta)
        # self.highlightingRules.append((QRegExp("\\bQ[A-Za-z]+\\b"), classFormat))

        # singleLineCommentFormat = QTextCharFormat()
        # singleLineCommentFormat.setForeground(Qt.red)
        # self.highlightingRules.append((QRegExp("//[^\n]*"), singleLineCommentFormat))

        # quotationFormat = QTextCharFormat()
        # quotationFormat.setForeground(Qt.darkGreen)
        # self.highlightingRules.append((QRegExp("\".*\""), quotationFormat))

        self.numberFormat = QTextCharFormat()
        self.numberFormat.setFontWeight(QFont.Bold)
        self.numberFormat.setForeground(Qt.darkBlue)
        # self.highlightingRules.append((QRegExp("(\[[0-9]:[0-9]\])"), self.numberFormat))

        # functionFormat = QTextCharFormat()
        # functionFormat.setFontItalic(True)
        # functionFormat.setForeground(Qt.blue)
        # self.highlightingRules.append((QRegExp("\\b[A-Za-z0-9_]+(?=\\()"), functionFormat))

    def addKeyword(self, keyword):
        # todo if a variable 'x' is in the ct function and a new state variable matching it ('x') is inserted, it does NOT hightlight right now. FIX!
        self.highlightingRules.append((keyword + "(\[[0-9]:[0-9]\])", self.numberFormat))
        self.highlightingRules.append(("\\b" + keyword + "\\b", self.keywordFormat))


    def highlightBlock(self, text):
        for pattern, format in self.highlightingRules:
            expression = QRegExp(pattern)
            index = expression.indexIn(text)
            while index >= 0:
                length = expression.matchedLength()
                self.setFormat(index, length, format)
                index = expression.indexIn(text, index + length)