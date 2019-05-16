import logging
import dbus
import dbus.service
from decoration import *
from bt.constants import Constants as Consts

PROFILES = {}


class ProfileHandler(dbus.service.Object):
    """
    mother class for bluetooth profiles management
    its behavior is not exactly the classic one due to command server and DBus constraints
    It must be inherited from all profile implementation
    It retains registered profiles so that only 1 python object is used to handle 1 defined profile
    profiles must define their attributes as class ones, not instance
    the __init__ of DBus service object is performed when 'register' is called
    """
    def __new__(cls, *args, **kwargs):
        """
        handles new profile creation
        """
        if cls.__name__ in PROFILES:
            Inner.LOGGER.debug("ProfileHandler: profile already registered")

        else:
            Inner.LOGGER.debug("ProfileHandler: new profile")
            obj = object.__new__(cls, *args, **kwargs)
            PROFILES[cls.__name__] = {"instance": obj, "initiated": False}
        return PROFILES[cls.__name__]["instance"]

    def __init__(self, bus, path, uuid, opts, *args, **kwargs):
        """
        common profile initialization part
        """
        if not PROFILES[self.__class__.__name__]["initiated"]:
            PROFILES[self.__class__.__name__]["initiated"] = True
            self._log = self.set_logger()
            self._bus = bus
            self._path = path
            self._opts = opts
            self._uuid = uuid
            obj = self._bus.get_object(Consts.Bt.SVC_NAME, Consts.DBus.BT_PATH)
            self._manager = dbus.Interface(obj, Consts.Bt.PROFILE_MNGR_IFACE)

    def register(self):
        """
        Registers profile
        """
        try:
            dbus.service.Object.__init__(self, self._bus, self._path)
            self._manager.RegisterProfile(self._path, self._uuid, self._opts)
            self._log.debug("'%s' registered" % self._path)
        except dbus.exceptions.DBusException as E:
            self._log.error(E.get_dbus_message())
            return FAILURE, E.get_dbus_message()
        return SUCCESS, "Profile registered at {0}".format(self._path)

    def unregister(self):
        """
        Unregisters profile
        """
        try:
            self._manager.UnregisterProfile(self._path)
            self.remove_from_connection(self._bus, self._path)
            self._log.debug("'%s' unregistered" % self._path)
        except dbus.DBusException as E:
            self._log.debug("unregister error")
            self._log.error(E.get_dbus_message())
            return FAILURE, E.get_dbus_message()
        return SUCCESS, "Profile at {0} unregistered".format(self._path)

    @staticmethod
    def set_logger():
        # Set logging
        # Log format
        log_fmt = '%(asctime)s   BtProfile\t%(levelname)5s   %(message)s'
        # Log level => default log level is INFO
        log_lvl = logging.DEBUG
        log_file = "/tmp/bt_edison.log"
        # Retrieve logger
        logging.basicConfig(filename=log_file, level=log_lvl, format=log_fmt)
        Log = logging.getLogger('BtProfile')
        Log.name = 'BtProfile'
        return Log
