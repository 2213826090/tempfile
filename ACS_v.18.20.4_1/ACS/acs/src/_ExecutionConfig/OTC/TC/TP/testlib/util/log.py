import threading
import logging
import logging.handlers
import sys
import os



class Logger:
    LOGLEVELS = {"CRITICAL":45,
                 "ERROR":35,
                 "WARNING":25,
                 "INFO":15,
                 "DEBUG":5,
                 "NOTSET":0
                }

    _loginst = None
    _loglock = threading.Lock()

    def __init__(self, module, format, loglevel="DEBUG", tag=None):
        """Log module init method"""
        self._formatter = logging.Formatter(format,'%Y-%m-%d %H:%M:%S')
        self._logobj = logging.getLogger(module)
        self._level = Logger.LOGLEVELS[loglevel]
        self._logobj.setLevel(Logger.LOGLEVELS[loglevel])
        writer = logging.StreamHandler(sys.stdout)
        writer.setLevel(Logger.LOGLEVELS[loglevel])
        writer.setFormatter(self._formatter)
        self._logobj.addHandler(writer)

    @staticmethod
    def getlogger(module="OAT-L", format="[%(asctime)s - %(levelname)s] %(message)s"):
        """Get logger instance"""
        if Logger._loginst is None:
            Logger._loglock.acquire()
            if Logger._loginst is None:
                Logger._loginst = Logger(module, format)
            else:
                pass
            Logger._loglock.release()
        return Logger._loginst

    def debug(self, message=None):
        """Log message of Debug"""
        if message is not None:
            self._logobj.debug(message)

    def info(self, message=None):
        """Log message of user info"""
        if message is not None:
            self._logobj.info(message)

    def warning(self, message=None):
        """Log message of warning"""
        if message is not None:
            self._logobj.warning(message)

    def error(self, message=None):
        """Log message of error"""
        if message is not None:
            self._logobj.error(message)

    def critical(self, message=None):
        """Log message of critical error"""
        if message is not None:
            self._logobj.critical(message)
