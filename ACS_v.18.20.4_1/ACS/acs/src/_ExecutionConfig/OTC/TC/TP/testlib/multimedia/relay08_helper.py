# coding: UTF-8
from testlib.multimedia.get_device_port_helper import GetDevicePortHelper
from testlib.em import relay08

from testlib.util.log import Logger
logger = Logger.getlogger()

class Relay08Helper:
    def __init__(self):
        get_device_port_helper = GetDevicePortHelper("", "relay08_helper.conf")
        self.device_port = get_device_port_helper.init_device_port("relay08_port", "relay08_port", "config")

        self.relay_config = get_device_port_helper.get_config_file_helper.get_section("relay_config")

    def set_relay_NO(self, t_str):
        if t_str in self.relay_config.keys():
            t_str = self.relay_config[t_str]
        logger.debug("set_relay_NO relay_num:%s" % str(t_str))
        relay08.set_relay_status_compatible(self.device_port + ":" + str(t_str), 0)

    def set_relay_NC(self, t_str):
        if t_str in self.relay_config.keys():
            t_str = self.relay_config[t_str]
        logger.debug("set_relay_NC relay_num:%s" % str(t_str))
        relay08.set_relay_status_compatible(self.device_port + ":" + str(t_str), 1)

