import threading
from log import Logger

EXPOSED_OBJECTS = {}


class IWatchdog(threading.Thread):
    """
    interface class needed to instanciate a watchdog class
    """
    def __init__(self):
        """
        constructor
        """
        threading.Thread.__init__(self)
        self.logger = Logger("IWatchdog")

    def run(self):
        """
        redefinition of run method of threading.Thread class
        :return: None
        """
        self.logger.error("undefined watchdog launcher")
