#!/usr/bin/env python

from properties import Properties
import dbus


class Device(object):
    def __init__(self, dbus_obj, iface_name):
        self._dbus_object = dbus_obj
        self._dbus_device_iface = dbus.Interface(self._dbus_object, iface_name)
        self._properties = Properties(self._dbus_object, iface_name)

    def __getattribute__(self, item):
        if item.startswith("_") or item in dir(Device):
            return object.__getattribute__(self, item)
        else:
            return getattr(self._properties, item)

    def __setattr__(self, key, value):
        if key.startswith('_'):
            super(Device, self).__setattr__(key, value)
        else:
            setattr(self._properties, key, value)

    def get_dbus_device_iface(self):
        return self._dbus_device_iface

    def get_dbus_object(self):
        return self._dbus_object
