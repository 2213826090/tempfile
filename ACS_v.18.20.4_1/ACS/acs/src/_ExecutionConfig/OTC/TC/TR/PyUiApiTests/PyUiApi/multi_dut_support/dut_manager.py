import re
from uiautomator import Device
import subprocess
import os
import json
from PyUiApi.log.logging_utils import *

import __builtin__
if not hasattr(__builtin__, 'LOG'):
    __builtin__.LOG = Log()


class DutManager(object):
    device_serial_regex_pattern = "(\w+).*device\Z"
    FIRST_DEVICE_INDEX = 0
    ACS_CONFIG_ENVIRONMENT_VAR_NAME = "ACS_GLOBAL_PARAMETERS"

    def __init__(self):
        self.acs_config = self.get_acs_config()
        self.devices_detected = self.get_available_dut_serials()
        self.active_uiautomator_device_serial = None
        self.active_uiautomator_device = None
        assert len(self.devices_detected) > 0, "at least 1 DUT must be connected"
        self.active_devices = {}
        for dut_serial in self.devices_detected:
            self.active_devices[dut_serial] = Device(dut_serial)
        if self.acs_config is None:
            LOG.info("acs config not found, proceeding with default init")
            self.init_default_active_device()
        else:
            LOG.info("acs config found, trying to import acs parameters")
            self.select_active_device_with_acs_config()

    def refresh_active_device(self):
        self.active_uiautomator_device = Device(self.active_uiautomator_device_serial)
        self.active_devices[self.active_uiautomator_device_serial] = self.active_uiautomator_device
        __builtin__.d = self.active_uiautomator_device

    def init_default_active_device(self):
        self.active_uiautomator_device_serial = self.active_devices.keys()[DutManager.FIRST_DEVICE_INDEX]
        self.active_uiautomator_device = self.active_devices[self.active_uiautomator_device_serial]

    def select_active_device_with_acs_config(self):
        LOG.info("found acs config environment")
        try:
            phone1_serial = str(self.acs_config[u'PHONE1'][u'serialNumber'])
            LOG.info("PHONE1 serial: " + phone1_serial)
            connected_dut_serials = self.active_devices.keys()
            for i in range(len(connected_dut_serials)):
                if phone1_serial == connected_dut_serials[i]:
                    LOG.info("found valid PHONE1 serial in acs_config: " + phone1_serial)
                    self.active_uiautomator_device_serial = connected_dut_serials[i]
                    self.active_uiautomator_device = self.active_devices[phone1_serial]
                    break
            if self.active_uiautomator_device_serial is None:
                LOG.info("could not find valid serial in acs_config, proceeding with default init")
                self.init_default_active_device()
        except:
            LOG.info("PHONE1 serial not specified")
            self.init_default_active_device()

    def get_available_dut_serials(self):
        output = subprocess.check_output("adb devices", shell=True)
        serials = []
        for line in output.splitlines():
            line_serials = re.findall(DutManager.device_serial_regex_pattern, line)
            serials = serials + line_serials
        return serials

    def get_number_of_duts_connected(self):
        return len(self.devices_detected)

    def activate_dut(self, dut_serial):
        self.active_uiautomator_device = self.active_devices[dut_serial]
        self.active_uiautomator_device_serial = dut_serial
        __builtin__.d = self.active_uiautomator_device

    def activate_phone(self, phone_string):
        try:
            phone_serial = str(self.acs_config[u'' + phone_string][u'serialNumber'])
            self.activate_dut(phone_serial)
        except:
            LOG.info("could not activate %s , not found in acs benchconfig" % phone_string)

    def get_acs_config(self):
        if DutManager.ACS_CONFIG_ENVIRONMENT_VAR_NAME in os.environ:
            return json.loads(os.environ[DutManager.ACS_CONFIG_ENVIRONMENT_VAR_NAME])
        return None


if __name__ == "__main__":
    dm = DutManager()
