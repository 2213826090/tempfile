import logging
import os
import sys
import threading
#import threading
from testlib.util.log import Logger
from testlib.util.common import g_common_obj

class Singleton(object):
    __singleton_lock = threading.Lock()
    __singleton_instance = None

    @classmethod
    def instance(cls):
        if not cls.__singleton_instance:
            with cls.__singleton_lock:
                if not cls.__singleton_instance:
                    cls.__singleton_instance = cls()
        return cls.__singleton_instance
    
    @classmethod
    def destory(cls):
        #print "-cls.__singleton_instance="+str(cls.__singleton_instance)
        with cls.__singleton_lock:
            cls.__singleton_instance=None

class CameraLogger(Singleton):
    LOG_FILE = "camera_dbg.log"
    #handlers = []

    def __init__(self):
        self.format = None
        self.fileHandler = None
        self.logger = Logger.getlogger()
        self.handlers = []

    def addFileHandler(self, mode = "a+", filename = None, loglevel = 0):
        filename = filename if filename else \
                os.path.join(g_common_obj.get_user_log_dir(), CameraLogger.LOG_FILE)
        loglevel = loglevel if loglevel else Logger.LOGLEVELS.get("DEBUG")
        self.format = logging.Formatter(
                "[%(asctime)s - tid:%(thread)d - camera-debug - %(levelname)s] %(message)s")

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
        ''' wrapper for jsonrpc methods '''
        return self.__dict__.get(attr) or getattr(self.logger, attr)
