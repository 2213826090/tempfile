#!/usr/bin/env python

"""
This module contains the classes for abtracting the DBUS properties.
expected properties are:
- for local bluetooth adapter:
    Address, Name, Alias, Powered, Pairable, PairableTimeout, Discoverable, DiscoverableTimeout, Discovering',
    UUIDs
- for remote bluetooth devices:
    Name, Adapter, Address, Alias, Appearance, Class, Icon, Paired, Trusted, RSSI, Blocked, Connected,
    LegacyPairing, UUIDs, Modalias, Vendor, Product, Version
"""
import dbus
from constants import Constants as Consts

from decoration import *

class Property(object):
    def __init__(self, name, iface, iface_name):
        self.iface = iface
        self.iface_name = iface_name
        self.name = name

    def __get__(self, obj=None, objtype=None):
        return self.iface.Get(self.iface_name, self.name)

    def __set__(self, instance=None, value=None):
        t = type(self.iface.Get(self.iface_name, self.name))
        self.iface.Set(self.iface_name, self.name, t(value))


class Properties(object):
    def __init__(self, dbus_obj, iface_name):
        iface = dbus.Interface(dbus_obj, Consts.DBus.PROPS)
        prop = iface.GetAll(iface_name)

        for k in prop:
            setattr(self, str(k).lower(), Property(k, iface, iface_name))

    def __getattribute__(self, item):
        if item.startswith("_"):
            return object.__getattribute__(self, item)
        elif item in self.__dict__:
            return self.__dict__[item].__get__()
        else:
            return None

    def __setattr__(self, key, value):
        if "Property" in type(value).__name__:
            self.__dict__[key] = value
        elif key in self.__dict__:
            self.__dict__[key].__set__(value=value)
