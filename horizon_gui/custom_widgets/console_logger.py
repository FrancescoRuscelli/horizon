import logging
from PyQt5.QtWidgets import QTextEdit

class ConsoleLogger(logging.Handler, QTextEdit):
    def __init__(self, parent):
        super().__init__()
        QTextEdit.__init__(self, parent)

        self.setReadOnly(True)
        self.setFormatter(logging.Formatter(fmt='%(asctime)s - %(levelname)s {%(pathname)s:%(lineno)d} %(levelname)s - %(message)s', datefmt='[%H:%M:%S]'))

    def emit(self, record):
        msg = self.format(record)
        # self.textCursor().insertText(msg) # no new line
        self.append(msg) # new line

    def write(self, m):
        pass