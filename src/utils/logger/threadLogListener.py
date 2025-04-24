import logging.handlers
from src.templates.threadwithstop import ThreadWithStop
from src.utils.logger.setupLogger import configLogger
from src.utils.logger.setupLogger import LoggingHandler, LoggerConfigs

class threadLogListener(ThreadWithStop):

    def __init__(self, loggingQ):
        self.loggingQ = loggingQ
        super(threadLogListener, self).__init__()

    def run(self):
        configLogger(LoggerConfigs.LISTENER)
        self.listener = logging.handlers.QueueListener(self.loggingQ, LoggingHandler())
        self.listener.start()
        while self._running:
            pass
        self.listener.stop()

    def start(self):
        super(threadLogListener, self).start()

    def stop(self):
        super(threadLogListener, self).stop()