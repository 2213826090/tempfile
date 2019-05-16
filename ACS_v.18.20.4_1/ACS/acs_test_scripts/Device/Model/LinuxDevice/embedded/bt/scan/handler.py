#!/usr/bin/env python

import dbus
import logging
from decoration import *
import time


class ScanHandler():

    DEFAULT_TIMEOUT = 20

    def __init__(self, addr=None, timeout=DEFAULT_TIMEOUT, *args, **kwargs):
        """
        Constructor
        """
        self._log = self.set_logger()
        self._remote_dev_addr = addr
        self._timeout = timeout
        self._result = None
        self._adapter = None
        self._devices = {}

    @staticmethod
    def set_logger():
        # Set logging
        # Log format
        log_fmt = '%(asctime)s   BtScanDevice\t%(levelname)5s   %(message)s'
        # Log level => default log level is INFO
        log_lvl = logging.DEBUG
        log_file = "/tmp/bt_edison.log"
        # Retrieve logger
        logging.basicConfig(filename=log_file, level=log_lvl, format=log_fmt)
        Log = logging.getLogger('BtScanDevice')
        Log.name = 'BtScanDevice'
        return Log

    @threaded_executor
    def get_result(self, *args, **kwargs):
        """
        Get scan handler result
        @rtype: string
        @return: "Success" if:
                    - specified device has been found
                    - no specified device
                 "Failure" otherwise
        """
        begin_time = time.time()
        while (time.time() - begin_time) < self._timeout:
            if self._result is not None:
                break
            time.sleep(1)
        else:
            if self._remote_dev_addr is None:
                output = {}
                devices = self._adapter.get_devices()
                for device in devices:
                    output[device] = devices[device].name
                self._result = (SUCCESS, output)
            else:
                self._log.debug("device not found")
                self._result = (FAILURE, "Device not found: scan timeout")

        self._end()
        return self._result

    def _handler(self, path, address):
        """
        Handle configured signal
        """
        self._log.debug("DeviceAdded signal received (%s, %s)" % (path, address))
        if address == self._remote_dev_addr:
            self._result = (SUCCESS, "device %s found" % self._remote_dev_addr)

    def _end(self):
        self._adapter.stop_discovery()
        self.remove_signal()

    @executor
    def scan(self, adapter=None, *args, **kwargs):
        """
        """
        self._log.debug("starting scan")
        self._adapter = adapter
        if adapter.get_dbus_adapter_iface() is None:
            self._result = (FAILURE, "Bluetooth adapter not found")
            self._log.error(self._result[1])

        # Check device has not already been scanned
        if self._remote_dev_addr in adapter.get_devices():
            self._result = (SUCCESS, "device %s found" % self._remote_dev_addr)
        else:
            self._log.debug("starting scan")
            self.add_signal()
            adapter.start_discovery()

    def add_signal(self):
        """
        Start listening to signal
        """
        self._log.debug("Listening to added device signal")
        bus = dbus.SessionBus()
        bus.add_signal_receiver(self._handler, bus_name="bt.watchdog",
                                dbus_interface="bt.watchdog",
                                signal_name="DeviceAdded")

    def remove_signal(self):
        """
        Stop listening to signal
        """
        self._log.debug("Stop listening to added device signal")
        bus = dbus.SessionBus()
        bus.remove_signal_receiver(self._handler, bus_name="bt.watchdog",
                                dbus_interface="bt.watchdog",
                                signal_name="DeviceAdded")
