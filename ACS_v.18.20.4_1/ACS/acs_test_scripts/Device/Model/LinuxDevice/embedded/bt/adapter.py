#!/usr/bin/env python

import dbus
import dbus.service
import dbus.mainloop.glib
import time
from decoration import *
from bt.properties import Properties
from bt.constants import Constants as Consts


class Adapter(object):
    """
    """

    def __init__(self, logger):
        """
        """
        self._log = logger
        self._dbus_adapter_iface = None
        self._dbus_object = None
        self._properties = None
        self._devices = {}
        self._enabled = False

    def __getattribute__(self, item):
        if item.startswith("_") or item in dir(Adapter):
            return object.__getattribute__(self, item)
        else:
            return getattr(self._properties, item)

    def __setattr__(self, key, value):
        if key.startswith('_'):
            self.__dict__[key] = value
        else:
            setattr(self._properties, key, value)

    def is_enabled(self):
        return self._enabled

    def enable(self, dbus_obj):
        """
        """
        self._log.info("Enabling BtCtrl ...")
        self._dbus_object = dbus_obj
        self._log.info("Enabling BtCtrl ...")
        self._dbus_adapter_iface = dbus.Interface(self._dbus_object, Consts.Bt.ADAPTER_IFACE)
        self._log.info("Enabling BtCtrl ...")
        self._properties = Properties(self._dbus_adapter_iface, Consts.Bt.ADAPTER_IFACE)

        self._log.debug("BT controller address: %s" % self.address)
        self._enabled = True

        self._log.info("BtCtrl enabled")

    def disable(self):
        """
        """
        self._enabled = False
        self._discovering = False
        self._dbus_object = None
        self._dbus_adapter_iface = None
        self._log.info("Bluetooth adapter disabled")

    def get_dbus_object(self):
        return self._dbus_object

    def get_dbus_adapter_iface(self):
        return self._dbus_adapter_iface

    def get_devices(self):
        return self._devices

    @executor
    def list_devices(self):
        """
        """
        # build a address => name mapping
        output = {}
        for device in self._devices:
            output[device] = self._devices[device].name

        return SUCCESS, output

    @executor
    def set_scan_mode(self, mode):
        """
        Set scan mode
        :type mode: str
        :param mode: scan mode:
            - "ON" to start scanning
            - "OFF" to stop scanning
        """
        if mode.lower() == "on":
            self.start_discovery()
            return SUCCESS, "Scanning mode ON"
        elif mode.lower() == "off":
            self.stop_discovery()
            return SUCCESS, "Scanning mode OFF"
        else:
            return FAILURE, "Invalid scan mode '%s'" % mode

    def start_discovery(self):
        """
        Start bluetooth device discovery
        """
        self._log.debug("Start bluetooth device discovery")
        if not self.discovering:
            self._dbus_adapter_iface.StartDiscovery()
            self._log.debug("Bluetooth device discovery started")

    def stop_discovery(self):
        """
        Stop bluetooth device discovery
        """
        self._log.debug("Stop bluetooth device discovery")
        if self.discovering:
            self._dbus_adapter_iface.StopDiscovery()
            self._log.debug("Bluetooth device discovery stopped")

    @executor
    def getprop(self, name):
        """
        Get adapter property
        """
        try:
            self._log.debug("get property '%s'" % name)
            value = getattr(self, name.lower())
            return SUCCESS, value
        except:
            self._log.error("[Adapter.getprop] unknown property")
            return FAILURE, "unknown property: %s" % name

    @executor
    def setprop(self, name, value):
        """
        Set adapter property
        :param name: name of the property
        :type name: str
        :param value: value to be set
        :type value: str, int or bool
        """
        try:
            self._log.debug("set property '{0}' to '{1}'".format(name, value))
            setattr(self, name.lower(), value)
            return SUCCESS, "{0} property set to {1}".format(name, value)
        except:
            self._log.error("[Adapter.setprop] unknown property or invalid value type")
            return FAILURE, "unknown property '{0}' or invalid value type '{1}' ".format(name, value)

    @executor
    def device_setprop(self, addr, name, value):
        """
        Set device property
        @type addr: str
        @param addr: bluetooth device address
        @type name: str
        @param name: property name
        @type value: object
        @param value: value to set
        """
        func = "[Adapter.device_setprop]:"
        self._log.debug("set {0} device property {1} to {2}".format(addr, name, value))
        if addr in self._devices:
            try:
                setattr(self._devices[addr], name.lower(), value)
                msg = "{0} property set to {1}".format(name, value)
                self._log.info("{0} {1}".format(func, msg))
                return SUCCESS, msg
            except:
                msg = "unknown property {0} or invalid value type {1} ".format(name, value)
                self._log.error("{0} {1}".format(func, msg))
                return FAILURE, msg
        else:
            msg = "device {0} not found in scanned devices list".format(addr)
            self._log.error("{0} {1}".format(func, msg))
            return FAILURE, msg

    @executor
    def device_getprop(self, addr, name):
        """
        Get device property
        """
        func = "[Adapter.device_getprop]:"
        self._log.debug("get {0} device {1} property".format(addr, name))
        if addr in self._devices:
            try:
                value = getattr(self._devices[addr], name.lower())
                msg = "{0} device {1} property is {2}".format(addr, name, value)
                self._log.info("{0} {1}".format(func, msg))
                return SUCCESS, value
            except:
                msg = "unknown property: {0}".format(name)
                self._log.error("{0} {1}".format(func, msg))
                return FAILURE, msg
        else:
            msg = "{0} device not found in scanned devices list".format(addr)
            self._log.error("{0} {1}".format(func, msg))
            return FAILURE, msg

    @executor
    def remove_device(self, addr):
        """
        Remove bluetooth device
        """
        status = FAILURE
        msg = "missing bluetooth address of device to remove"
        if addr is None:
            self._log.error("[Adapter.remove] missing bluetooth address of the device to remove")
            return status, msg

        self._log.info("Remove device %s" % addr)
        msg = "Device %s not found: nothing to remove" % addr
        for device in self._devices.values():
            if device.address == addr:
                self._dbus_adapter_iface.RemoveDevice(device.get_dbus_device_iface())
                msg = "device %s removed" % addr
                break
        status = SUCCESS
        return status, msg

    @executor
    def connect(self, addr):
        """
        """
        if addr in self._devices:
            device = self._devices[addr].get_dbus_device_iface()
            try:
                device.Connect()
                return SUCCESS, "device %s connected" % addr
            except dbus.DBusException as E:
                self._log.error(str(E.get_dbus_message()))
                return FAILURE, str(E.get_dbus_message())
        else:
            return FAILURE, "device %s not found" % addr

    @executor
    def disconnect(self, addr):
        """
        """
        if addr in self._devices:
            device = self._devices[addr].get_dbus_device_iface()
            try:
                device.Disconnect()
                return SUCCESS, "device %s disconnected" % addr
            except dbus.DBusException as E:
                self._log.error(str(E.get_dbus_message()))
                return FAILURE, str(E.get_dbus_message())
        else:
            return FAILURE, "device %s not found" % addr

    @executor
    def connect_profile(self, addr, profile):
        """
        """

        if addr in self._devices:
            device = self._devices[addr].get_dbus_device_iface()
            try:
                device.ConnectProfile(profile)
                return SUCCESS, "connected to profile {0} of device {1}".format(profile, addr)
            except dbus.DBusException as E:
                self._log.error(str(E.get_dbus_message()))
                return FAILURE, str(E.get_dbus_message())
        else:
            return FAILURE, "device %s not found" % addr

    @executor
    def disconnect_profile(self, addr, profile):
        """
        """
        if addr in self._devices:
            device = self._devices[addr].get_dbus_device_iface()
            try:
                device.DisonnectProfile(profile)
                return SUCCESS, "disconnected from profile {0} of device {1}".format(profile, addr)
            except dbus.DBusException as E:
                self._log.error(str(E.get_dbus_message()))
                return FAILURE, str(E.get_dbus_message())
        else:
            return FAILURE, "device %s not found" % addr

    @executor
    def paired_devices(self):
        """
        """
        devices = {}

        for addr in self._devices:
            if self._devices[addr].paired:
                devices[addr] = self._devices[addr].name
        return SUCCESS, devices
