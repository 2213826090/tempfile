#!/usr/bin/env python

import dbus

from bt.constants import Constants as Consts
from bt.properties import BtProperties as BTP
from bt.device import BtDevice as BTD

def get_manager():
    """
    Get bluetooth manager
    """
    bus = dbus.SystemBus()
    manager = dbus.Interface(bus.get_object(Consts.Bt.SVC_NAME, Consts.DBus.OBJ_MNGR_PATH), Consts.DBus.OBJ_MNGR)
    return manager

def get_managed_objects():
    """
    Get bluetooth managed objects
    """
    manager = get_manager()
    return manager.GetManagedObjects()

def find_adapter(pattern=None):
    """
    Find bluetooth adapter object (org.bluez.Adapter1)
    """
    return _find_adapter_in_objects(get_managed_objects(), pattern)

def find_device(device_address, adapter_pattern=None):
    """
    Find device in scanned devices
    """
    objects = get_managed_objects()
    return _find_device_in_objects(objects, device_address, adapter_pattern)

def find_paired_devices(adapter_pattern=None):
    """
    Find paired devices
    """
    objects = get_managed_objects()
    return _find_paired_devices(objects, adapter_pattern)

def list_devices():
    """
    List scanned devices
    """
    scanned_devices = {}

    objects = get_managed_objects()

    all_devices = (str(path) for path, interfaces in objects.iteritems() if Consts.Bt.DEV_IFACE in interfaces.keys())

    for path, interfaces in objects.iteritems():
        if Consts.Bt.ADAPTER_IFACE not in interfaces.keys():
            continue

        properties = interfaces[Consts.Bt.ADAPTER_IFACE]
        for key in properties.keys():
            value = properties[key]
            if (key == BTP.UUIDS):
                uuids = _extract_uuids(value)

        device_list = [d for d in all_devices if d.startswith(path + "/")]

        for dev_path in device_list:

            dev = objects[dev_path]
            properties = dev[Consts.Bt.DEV_IFACE]

            if BTD.NAME in properties and BTD.ADDRESS in properties:
                scanned_devices.update({properties[BTD.NAME]: properties[BTD.ADDRESS]})

    return scanned_devices

def _find_adapter_in_objects(objects, pattern=None):
    """
    Find bluetooth adapter in DBus objects
    @return adapter or None if adapter not found
    """
    bus = dbus.SystemBus()
    for path, ifaces in objects.iteritems():
        adapter = ifaces.get(Consts.Bt.ADAPTER_IFACE)
        if adapter is None:
            continue
        if not pattern or pattern == adapter[BTP.ADDRESS] or path.endswith(pattern):
            obj = bus.get_object(Consts.Bt.SVC_NAME, path)
            return dbus.Interface(obj, Consts.Bt.ADAPTER_IFACE)
    return None

def _find_device_in_objects(objects, device_address, adapter_pattern=None):
    """
    """
    bus = dbus.SystemBus()
    path_prefix = ""
    if adapter_pattern:
        adapter = _find_adapter_in_objects(objects, adapter_pattern)
        path_prefix = adapter.object_path
    for path, ifaces in objects.iteritems():
        device = ifaces.get(Consts.Bt.DEV_IFACE)
        if device is None:
            continue
        if (device[BTD.ADDRESS] == device_address and path.startswith(path_prefix)):
            obj = bus.get_object(Consts.Bt.SVC_NAME, path)
            return (dbus.Interface(obj, Consts.Bt.DEV_IFACE), dbus.Interface(obj, Consts.DBus.PROPS))

    return None, None

def _find_paired_devices(objects, adapter_pattern=None):
    """
    """
    path_prefix = ""
    devices = []

    if adapter_pattern:
        adapter = _find_adapter_in_objects(objects, adapter_pattern)
        path_prefix = adapter.object_path
    for path, ifaces in objects.iteritems():
        device = ifaces.get(Consts.Bt.DEV_IFACE)
        if device is None:
            continue
        if (device[BTD.PAIRED] == 1 and path.startswith(path_prefix)):
            devices.append(device)
    return devices

def _extract_uuids(uuid_list):
    """
    """
    res = ""
    for uuid in uuid_list:
        if (uuid.endswith("-0000-1000-8000-00805f9b34fb")):
            if (uuid.startswith("0000")):
                val = "0x" + uuid[4:8]
            else:
                val = "0x" + uuid[0:8]
        else:
            val = str(uuid)
        res = res + val + " "
    return res

def _extract_objects(object_list):
    """
    """
    objs = ""
    for obj in object_list:
        val = str(obj)
        objs = objs + val[val.rfind("/") + 1:] + " "
    return objs
