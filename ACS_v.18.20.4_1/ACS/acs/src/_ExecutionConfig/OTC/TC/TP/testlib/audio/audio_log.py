import logging
import os
import threading
#import threading
from testlib.util.log import Logger
from testlib.util.common import g_common_obj

#class AudioLogger(object):
class AudioLogger(object):
    LOG_FILE = "audio_log.log"
    _log_instance = None
    _log_lock = threading.Lock()
    handlers = []

    def __init__(self):
        self.format = None
        self.fileHandler = None
        self.logger = Logger.getlogger()

    def addFileHandler(self, mode = "a+", filename = None, loglevel = 0):
        filename = filename if filename else \
                os.path.join(g_common_obj.get_user_log_dir(), AudioLogger.LOG_FILE)
        loglevel = loglevel if loglevel else Logger.LOGLEVELS.get("DEBUG")
        self.format = logging.Formatter(
                "[%(asctime)s - Multimedia_Audio - %(levelname)s] %(message)s")

        self.fileHandler = logging.FileHandler(filename = filename, mode = mode)
        self.fileHandler.setLevel(loglevel)
        self.fileHandler.setFormatter(self.format)
        self.handlers.append(self.fileHandler)
        self.logger._logobj.addHandler(self.fileHandler)

    def removeHandler(self, handler):
        if handler in self.logger._logobj.handlers:
            self.logger._logobj.removeHandler(handler)
        else:
            self.logger.error("Handler not in logger object. Skip to remove")

    def removeHanlders(self):
        for h in self.handlers:
            self.removeHandler(h)
        self.handlers = []

    def __getattr__(self, attr):
        ''' wrapper for logger methods '''
        return self.__dict__.get(attr) or getattr(self.logger, attr)

    @staticmethod
    def getLogger(*args, **kws):
        if not AudioLogger._log_instance:
            AudioLogger._log_lock.acquire()
            AudioLogger._log_instance = AudioLogger(*args, **kws)
            AudioLogger._log_lock.release()
        return AudioLogger._log_instance

    #def __new__(cls, *args, **kws):
    #    if not hasattr(cls, "_instance"):
    #        orig = super(AudioLogger, cls)
    #        cls._instance = orig.__new__(cls, *args, **kws)
    #    return cls._instance
