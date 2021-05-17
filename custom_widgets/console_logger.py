import logging
from PyQt5.QtWidgets import QTextEdit

class ConsoleLogger(logging.Handler, QTextEdit):
    def __init__(self, parent):
        super().__init__()
        QTextEdit.__init__(self, parent)

        self.setReadOnly(True)
        self.setFormatter(logging.Formatter(fmt='%(asctime)s - %(levelname)s - %(message)s', datefmt='[%H:%M:%S]'))

    def emit(self, record):
        msg = self.format(record)
        self.textCursor().insertText(msg)

    def write(self, m):
        pass