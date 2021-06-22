from PyQt5.QtCore import Qt, QRegExp
from PyQt5.QtGui import QFont, QSyntaxHighlighter, QTextCharFormat

class Highlighter(QSyntaxHighlighter):
    def __init__(self, parent=None):
        super(Highlighter, self).__init__(parent)

        self.keywordFormat = QTextCharFormat()
        self.keywordFormat.setForeground(Qt.darkGreen)
        self.keywordFormat.setFontWeight(QFont.Bold)

        self.keywordPatterns = list()
        print(list((QRegExp(pattern), self.keywordFormat) for pattern in self.keywordPatterns))
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

        self.sliceFormat = QTextCharFormat()
        self.sliceFormat.setFontWeight(QFont.Bold)
        self.sliceFormat.setForeground(Qt.darkBlue)

        self.baseOperatorFormat = QTextCharFormat()
        self.baseOperatorFormat.setFontItalic(True)
        self.baseOperatorFormat.setFontWeight(QFont.Bold)
        self.baseOperatorFormat.setForeground(Qt.darkCyan)
        base_op = ['sin', 'cos', 'sqrt']
        # dir(__builtin__)
        # dir (casadi)
        # dir (numpy)

        # todo find a way to give priority to functions, otherwise variable names are highlighted inside name of variables, if any
        self.highlightingRules.append(('|'.join(base_op), self.baseOperatorFormat))


        # functionFormat = QTextCharFormat()
        # functionFormat.setFontItalic(True)
        # functionFormat.setForeground(Qt.blue)
        # self.highlightingRules.append((QRegExp("\\b[A-Za-z0-9_]+(?=\\()"), functionFormat))

    def addKeyword(self, keyword):
        # todo if a variable 'x' is in the ct function and a new state variable matching it ('x') is inserted, it does NOT hightlight right now. FIX!
        self.highlightingRules.append((keyword + '(\[\d+(:\d+)?\])', self.sliceFormat))
        # self.highlightingRules.append(("\\b" + keyword + "\\b", self.keywordFormat))
        self.highlightingRules.append((keyword, self.keywordFormat))


    def highlightBlock(self, text):
        for pattern, format in self.highlightingRules:
            expression = QRegExp(pattern)
            index = expression.indexIn(text)
            while index >= 0:
                length = expression.matchedLength()
                self.setFormat(index, length, format)
                index = expression.indexIn(text, index + length)