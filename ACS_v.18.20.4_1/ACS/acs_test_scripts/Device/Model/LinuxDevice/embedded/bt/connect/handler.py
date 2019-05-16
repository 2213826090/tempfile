#!/usr/bin/env python

import logging
from decoration import *
import subprocess
import time
import dbus

from bt.network_server import NetworkServer
from bt.constants import Constants as Consts

class BtConnectHandler():

    def __init__(self, adapter=None, addr=None, profile=None, *args, **kwargs):
        """
        Constructor
        """
        self._log = self.set_logger()
        self._bus = dbus.SystemBus()
        self._remote_dev_addr = addr
        self._adapter = adapter
        self._profile = profile
        self._timeout = 20
        self._disconnect = False
        self._device_path = None
        self._result = None
        self._listening = False
        self._signal_match = None

    def _set_result(self, status, result):
        """
        Set final command result
        @type status: str
        @param status: command status
        @type result: object
        @param result: command result
        """
        self._result = (status, result)

    @staticmethod
    def set_logger():
        # Set logging
        # Log format
        log_fmt = '%(asctime)s   BtConnect\t%(levelname)5s   %(message)s'
        # Log level => default log level is INFO
        log_lvl = logging.DEBUG
        log_file = "/tmp/bt_edison.log"
        # Retrieve logger
        logging.basicConfig(filename=log_file, level=log_lvl, format=log_fmt)
        Log = logging.getLogger('BtConnect')
        Log.name = 'BtConnect'
        return Log

    @threaded_executor
    def get_result(self, *args, **kwargs):
        """
        Get command result
        @rtype: tuple
        @return: command result as tuple:
            - command status
            - command result
        """
        begin_time = time.time()
        while (time.time() - begin_time) < self._timeout:
            if self._result is not None:
                break
            time.sleep(1)
        else:
            self._set_result(FAILURE, "Timeout")

        self._remove_signal()
        return self._result

    def _configure(self):
        """
        Configure host device regarding profile and other options
        """
        fct = None
        if self._profile is None:
            self._set_result(SUCCESS, "Connection success: no specific profile configured")
        else:
            try:
                fct = getattr(self, "_configure_" + self._profile)
            except AttributeError:
                self._log.error("_configure_%s function doesn't exist" % self._profile)
                self._set_result(FAILURE, "cannot configure '%s'" % self._profile)
            # Call adapted configuration function
            fct()

    def _handler(self, interface, changed, invalidated, path):
        """
        Handle configured signal
        """
        self._log.debug("PropertyChanged signal received")
        iface = interface[interface.rfind(".") + 1:]

        if "Connected" not in changed:
            return

        connected = changed["Connected"]
        self._log.debug("{%s.PropertyChanged} [%s] Connected = %s" % (iface, path, connected))

        if self._disconnect != connected:
            # => Expected state reached
            # Check device address
            if self._remote_dev_addr is not None:
                # Check device address
                if self._device_path != path:
                    return

            # => Expected device
            if connected:
                self._configure()
            else:
                # Disconnection success
                self._set_result(SUCCESS, "Disconnection success")

    def _add_signal(self):
        """
        Start listening to signal
        """
        if self._signal_match is None:
            self._log.debug("Listening to PropertiesChanged signal")
            self._signal_match = self._bus.add_signal_receiver(self._handler, bus_name=Consts.Bt.SVC_NAME,
                dbus_interface=Consts.DBus.PROPS,
                signal_name="PropertiesChanged",
                path_keyword="path")

    def _remove_signal(self):
        """
        Stop listening to signal
        """
        if self._signal_match is not None:
            self._log.debug("Stop listening to PropertiesChanged signal")
            self._log.debug("Remove signal match")
            self._signal_match.remove()
            self._signal_match = None

    def _wait_for_state(self, disconnect, profile=None):
        """
        Wait for expected connection state
        @type disconnect: bool
        @param disconnect: True if expected state is disconnected, False otherwise
        @type profile: str
        @param profile: specific bluetooth profile to connect t
        """
        self._disconnect = disconnect
        self._profile = profile
        action = "connection"
        if self._disconnect:
            action = "disconnection"
        if self._remote_dev_addr is None:
            self._log.info("Waiting for device %s" % action)
        else:
            self._log.info("Waiting for %s device %s" % (self._remote_dev_addr, action))
            # Check if device is already in expected state
            devices = self._adapter.get_devices()
            if self._remote_dev_addr in devices:
                device = devices[self._remote_dev_addr]
                self._device_path = device.get_dbus_device_iface().object_path
                if device.connected != self._disconnect:
                    # => Device is already in expected state
                    if device.connected:
                        # => Device is already connected
                        self._log.debug("Device %s is already connected" % self._remote_dev_addr)
                        # Configure device in case of specific profile connection
                        self._configure()
                    else:
                        # => Device is already disconnected
                        self._log.debug("Device %s is already disconnected" % self._remote_dev_addr)
                        self._set_result(SUCCESS, "Disconnection success")
                    return

        # Device is not connected => listen to signal
        self._add_signal()

    @executor
    def wait_for_connection(self, *args, **kwargs):
        """
        Wait for bluetooth device connection
        """
        self._wait_for_state(False)

    @executor
    def wait_for_disconnection(self, *args, **kwargs):
        """
        Wait for bluetooth device disconnection
        """
        self._wait_for_state(True)

    @threaded_executor
    def connect(self, *args, **kwargs):
        """
        Connect to device
        """
        self._log.info("Connect to %s device" % self._remote_dev_addr)
        devices = self._adapter.get_devices()
        if self._remote_dev_addr in devices:
            device = devices[self._remote_dev_addr]
            try:
                self._log.debug(device)
                device.get_dbus_device_iface().Connect()
            except dbus.exceptions.DBusException as E:
                if E.get_dbus_name() == "org.bluez.Error.AlreadyConnected":
                    self._log.info("Connection success: already connected")
                    return SUCCESS, "Already connected"
                self._log.error(E)
                return FAILURE, "Connection failure"
            return SUCCESS, "Connection succeeded"
        else:
            return FAILURE, "Device not found"

    @threaded_executor
    def disconnect(self, *args, **kwargs):
        """
        Disconnect from device
        """
        self._log.info("Disconnect from %s device" % self._remote_dev_addr)
        devices = self._adapter.get_devices()
        if self._remote_dev_addr in devices:
            device = devices[self._remote_dev_addr]
            try:
                self._log.debug(device)
                device.get_dbus_device_iface().Disconnect()
            except dbus.exceptions.DBusException as E:
                self._log.error(E)
                return FAILURE, "Disconnection failure"
            return SUCCESS, "Disconnection succeeded"
        else:
            return FAILURE, "Device not found"
