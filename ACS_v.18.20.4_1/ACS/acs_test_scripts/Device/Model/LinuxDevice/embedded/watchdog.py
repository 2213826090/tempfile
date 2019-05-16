import importlib
import watchdogs
import pkgutil
from watchdogs import IWatchdog
from log import Logger


class WatchdogHandler(object):
    """
    the class that handles the ever running tasks called watchdogs
    watchdogs are expected to be classes inheriting from IWatchdog
    located in package watchdogs
    """
    def __init__(self):
        """
        constructor
        """
        self._watchdogs = {}
        self._log = Logger("Watchdogs")
        for _, module, _ in pkgutil.walk_packages(watchdogs.__path__):
            # load the module
            m = importlib.import_module("watchdogs.%s" % module)
            for a in dir(m):
                try:
                    if m.__dict__[a] != IWatchdog and issubclass(m.__dict__[a], IWatchdog):
                        self._log.debug("found %s for %s" % (m.__dict__[a], module))
                        self._watchdogs[module] = m.__dict__[a]
                        # there is only 1 IWatchdog class expected by module
                        break
                except:
                    pass

    def start(self):
        """
        method to start the watchdogs
        :return: None
        """
        for feat, watchdog in self._watchdogs.items():
            self._log.debug("starting %s watchdog" % feat)
            watchdog().start()
