
from uiautomator import Device
import __builtin__
import json,os
import subprocess,re
from testlib.util.log import Logger

LOG = Logger.getlogger(__name__)

class DutManager(object):
    device_serial_regex_pattern = "([A-Za-z0-9_\-]+).*device\Z"
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

    def init_default_active_device(self):
        self.active_uiautomator_device_serial = self.active_devices.keys()[DutManager.FIRST_DEVICE_INDEX]
        self.active_uiautomator_device = self.active_devices[self.active_uiautomator_device_serial]

    def refresh_active_device(self):
        self.active_uiautomator_device = Device(self.active_uiautomator_device_serial)
        self.active_devices[self.active_uiautomator_device_serial] = self.active_uiautomator_device
        __builtin__.d = self.active_uiautomator_device

    def get_acs_config(self):
        if DutManager.ACS_CONFIG_ENVIRONMENT_VAR_NAME in os.environ:
            return json.loads(os.environ[DutManager.ACS_CONFIG_ENVIRONMENT_VAR_NAME])
        return None

    def get_available_dut_serials(self):
        output = subprocess.check_output("adb devices", shell=True)
        serials = []
        for line in output.splitlines():
            line_serials = re.findall(DutManager.device_serial_regex_pattern, line)
            serials = serials + line_serials
        return serials

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

dut_manager = DutManager()