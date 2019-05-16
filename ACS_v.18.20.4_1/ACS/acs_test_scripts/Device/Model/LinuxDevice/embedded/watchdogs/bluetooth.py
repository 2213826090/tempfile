import dbus
import dbus.glib
import dbus.service
import dbus.mainloop.glib
import gobject as GObject
from bt.constants import Constants as Consts
from bt.device import Device
import os
from bt.adapter import Adapter
from . import IWatchdog, EXPOSED_OBJECTS
from log import Logger
import json


class WatchdogEvent(dbus.service.Object):
    """
    definition of events raised by the bluetooth watchdog on device / adapter creation or deletion
    """
    def __init__(self, logger):
        with open("/tmp/bus.conf", "r") as f:
            # lines = f.readlines()
            lines = json.load(f)
        for name, value in lines.items():
            # name, value = line.replace("\n", "").split("=", 1)
            os.environ[name] = value
        my_bus = dbus.SessionBus()
        bus_name = dbus.service.BusName('bt.watchdog', my_bus)
        dbus.service.Object.__init__(self, bus_name, "/watchdog/bluetooth")
        self._logger = logger

    @dbus.service.signal(dbus_interface='bt.watchdog', signature='ss')
    def AdapterAdded(self, path, address):
        """
        event raised when an adapter is added
        """
        self._logger.debug("Adapter %s added" % path)

    @dbus.service.signal(dbus_interface='bt.watchdog', signature='s')
    def AdapterRemoved(self, path):
        """
        event raised when an adapter is removed
        """
        self._logger.debug("Adapter %s removed" % path)

    @dbus.service.signal(dbus_interface='bt.watchdog', signature='ss')
    def DeviceAdded(self, path, address):
        """
        event raised when a device is added
        """
        self._logger.debug("Device %s added" % path)

    @dbus.service.signal(dbus_interface='bt.watchdog', signature='s')
    def DeviceRemoved(self, path):
        """
        event raised when a device is removed
        """
        self._logger.debug("Device %s removed" % path)


class Watchdog(IWatchdog):
    """
    watchdog implementation for bluetooth
    """
    def __init__(self):
        """
        constructor
        """
        IWatchdog.__init__(self)
        self._eventloop = GObject.MainLoop()
        self.logger = Logger("bt.watchdog")
        self.logger.debug("initializing Bluetooth watchdog")
        self._events = WatchdogEvent(self.logger)
        self.bt_powered = False
        # global objects about DBus
        self._bus = dbus.SystemBus()
        self._dbus_obj = self._bus.get_object(Consts.Bt.SVC_NAME, Consts.DBus.OBJ_MNGR_PATH)
        try:
            self._dbus_manager = dbus.Interface(self._dbus_obj, Consts.DBus.OBJ_MNGR)
        except dbus.DBusException as E:
            self.logger.error("%s" % (E.get_dbus_message()))
            self._dbus_manager = None
        except:
            self.logger.error("Unknown error while retrieving DBUS object manager")
            self._dbus_manager = None
        self._dbus_adapter_obj = None

        self._adapter = [Adapter(self.logger)]
        # identify it as the used adapter
        self._used_adapter = self._adapter[0]
        self.logger.debug("registering adapter")
        EXPOSED_OBJECTS["adapter"] = self._used_adapter
        self.logger.debug(EXPOSED_OBJECTS)

        # devices is expected to be a @MAC -> Device object mapping
        self._devices = self._used_adapter.get_devices()
        # map_devices is expected to be a path -> @MAC mapping
        self._map_devices = {}

    def run(self):
        """
        redefinition of run method from IWatchdog
        """
        self.logger.info("Start BT power watchdog")
        self._parse_devices()
        self._bus.add_signal_receiver(self._add_handler, bus_name=Consts.Bt.SVC_NAME,
                                      dbus_interface=Consts.DBus.OBJ_MNGR,
                                      signal_name="InterfacesAdded")
        self._bus.add_signal_receiver(self._rm_handler, bus_name=Consts.Bt.SVC_NAME,
                                      dbus_interface=Consts.DBus.OBJ_MNGR,
                                      signal_name="InterfacesRemoved")
        self._eventloop.run()

        self.logger.info("Exit BT power watchdog")

    def get_eventloop(self):
        """
        allows to get the DBUS eventloop
        :return the eventloop
        :rtype MainLoop
        """
        self.logger.debug("get_eventloop")
        return self._eventloop

    def _parse_devices(self, pattern=None):
        """
        Find bluetooth adapter in DBus objects
        :return adapter or None if adapter not found
        """
        objects = self._dbus_manager.GetManagedObjects()
        for dbus_path, ifaces in objects.iteritems():
            if Consts.Bt.DEV_IFACE in ifaces:
                self._add_device(str(dbus_path))
            if Consts.Bt.ADAPTER_IFACE in ifaces:
                self._add_adapter(str(dbus_path))

    def _add_handler(self, path, interfaces):
        """
        Handle configured signal
        :param path: dbus path
        :type path: str
        :param interfaces: list of interfaces
        :type interfaces: dict
        :return: None
        """
        if Consts.Bt.ADAPTER_IFACE in interfaces:
            self._add_adapter(path)
        if Consts.Bt.DEV_IFACE in interfaces:
            self._add_device(path)

    def _rm_handler(self, path, interfaces):
        """
        Handle configured signal
        :param path: dbus path
        :type path: str
        :param interfaces: list of interfaces
        :type interfaces: dict
        :return: None
        """
        if Consts.Bt.ADAPTER_IFACE in interfaces:
            self._rm_adapter(path)
        if Consts.Bt.DEV_IFACE in interfaces:
            self._rm_device(path)

    def _add_adapter(self, path):
        """
        method called when a DBUS adapter is detected
        :param path: dbus path
        :type path: str
        :return: None
        """
        if self._dbus_adapter_obj is None:
            self.logger.info("BT has been powered ON")
            self._dbus_adapter_obj = self._bus.get_object(Consts.Bt.SVC_NAME, path)
            self._used_adapter.enable(self._dbus_adapter_obj)
            self.bt_powered = True
            self._events.AdapterAdded(path, self._used_adapter.address)
        else:
            self.logger.info("Adapter already exists")

    def _rm_adapter(self, path):
        """
        method called when a DBUS adapter is removed
        :param path: dbus path
        :type path: str
        :return: None
        """
        self.logger.info("BT has been powered OFF")
        self._used_adapter.disable()
        self.bt_powered = False
        self._dbus_adapter_obj = None
        self._events.AdapterRemoved(path)

    def _add_device(self, path):
        """
        method called when a new DBUS devices is detected
        :param path: dbus path
        :type path: str
        :return: None
        """
        if path not in self._map_devices:
            dbus_obj = self._bus.get_object(Consts.Bt.SVC_NAME, path)
            dev = Device(dbus_obj, Consts.Bt.DEV_IFACE)
            self._devices[str(dev.address)] = dev
            self._map_devices[path] = str(dev.address)
            self._events.DeviceAdded(path, dev.address)
        else:
            self.logger.debug("device %s already scanned" % str(self._map_devices[path]))

    def _rm_device(self, path):
        """
        method called when a DBUS device is removed
        :param path: dbus path
        :type path: str
        :return: None
        """
        if path in self._map_devices:
            del self._devices[self._map_devices[path]]
            del self._map_devices[path]
            self._events.DeviceRemoved(path)
        else:
            self.logger.debug("device %s not found" % str(path))
