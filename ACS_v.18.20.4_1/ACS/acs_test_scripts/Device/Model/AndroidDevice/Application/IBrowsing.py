"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related
to the source code ("Material") are owned by Intel Corporation or its
suppliers or licensors. Title to the Material remains with Intel Corporation
or its suppliers and licensors. The Material contains trade secrets and
proprietary and confidential information of Intel or its suppliers and
licensors.

The Material is protected by worldwide copyright and trade secret laws and
treaty provisions. No part of the Material may be used, copied, reproduced,
modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual
property right is granted to or conferred upon you by disclosure or delivery
of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL MCG PSI
:summary: This script is an interface for benchmark based on browser
:since: 31/05/2013
:author: pbluniex
"""
import time
import re
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Device.Model.AndroidDevice.Application.IApplication import IApplication


class IBrowsing(IApplication):

    """
    Browsing implementation
    """

    def __init__(self, device):
        """
        Initializes this instance.

        :type device: Device
        :param device: The DUT
        """
        IApplication.__init__(self, device)

        self._wifi_router = None
        self._ssid = None
        self._passphrase = None
        self._security = None
        self._browser_cache_dir = None
        self._results = None
        self._wait_btwn_cmd = None
        self._browser = "native"
        self._browsername = None
        self._logcat_pattern = None
        self._display_orientation = False
        self.__url = None
        self._parameters = {}

        self._sw_release = device.get_sw_release()
        self._model_number = device.get_model_number()
        self._kernel_version = device.get_kernel_version()
        self._fw_version = device.get_fw_version()
        self._device_id = device.get_device_id()

    def __set_orientation(self):
        """
        Set orientation to the given parameter
        """
        orientation = self._parameters.get("orientation")
        if orientation in ["landscape", "portrait"]:
            self._display_orientation = True
            self._display.set_display_orientation(orientation)

    def __set_browser(self):
        """
        Set the browser
        """
        browser = self._parameters.get("browser", "native")
        if browser in ["native", "chrome"]:
            self._browser = browser

        if self._browser == "native":
            self._browser_cache_dir = "/data/data/com.android.browser/cache/webviewCacheChromium"
            self._browsername = "com.android.browser"
            self._logcat_pattern = "(?P<level>\w)\s*browser\s*:\s*Console:\s*(?P<msg>.*) http://.*"
        else:
            self._browser_cache_dir = "/data/data/com.android.chrome/cache/Cache"
            self._browsername = "com.android.chrome"
            self._logcat_pattern = "(?P<level>\w)\s*chromium:\s*\[.*:CONSOLE\(\d*\)\]\s*"\
                                   "\"(?P<msg>.*)\", source:"

    def __parse_arguments(self):
        """
        Parse arguments
        """
        if self._arguments is None:
            return

        configs = ("".join(line.strip()
                           for line in self._arguments.strip(" ;\n").splitlines())).split(";")

        arguments = {}
        for conf in configs:
            self._logger.info("configs")
            self._logger.info(conf)
            args = [x.strip(" \n").lower() for x in conf.split(":")]
            self._logger.info(args[0])
            self._logger.info(args[1])
            arguments[args[0]] = args[1]

        self._parameters = arguments

    def __wifi_enable(self):
        """
        Activate wifi
        dlogger = self._get_device_logger()
        wifi_enabled = "WifiService: setWifiEnabled: true"

        dlogger.add_trigger_message(wifi_enabled)

        self._networking.set_wifi_power("on")

        timeout = time.time() + 60
        while time.time() < timeout and self._networking.get_wifi_power_status() == 0:
            time.sleep(1)

        if not dlogger.is_message_received(wifi_enabled, 60):
            raise DeviceException(DeviceException.OPERATION_OPERATION,
                                  "Failed to enable (remains Off)")

        """
        self._networking.wifi_remove_config("all")
        self._networking.set_wifi_power("off")
        time.sleep(5)
        self._networking.set_wifi_power("on")
        time.sleep(5)
        #dlogger.remove_trigger_message(wifi_enabled)

    def __wifi_connection(self):
        """
        Connection to the wifi
        """
        dlogger = self._get_device_logger()
        wifi_connect = "ConnectivityService: handleInetConditionHoldEnd: net=1"

        self._networking.set_wificonfiguration(self._ssid,
                                               self._passphrase,
                                               self._security)
        self._networking.wifi_connect(self._ssid, True)

    def wait(self, timeout):
        """
        Wait until the application ends

        :type timeout: integer
        :param timeout: Time in second beyond the application should end
        """
        self._system_api.wait_process_loaded(self._browsername, timeout, 10)

    def install(self, appuri, additionnals=None, arguments=None, url=None, destination=None):
        """
        Install the application on the device

        :type appuri: String
        :param appuri: The full path to the application file

        :type additionnals: String
        :param additionnals: The full path of additionnals elements to run the
                             application

        :type arguments: String
        :param arguments: The arguments of the application. May be everything
                          the application need to run.

        :type destination: String
        :param destination: The directory where the application will be installed
        """
        IApplication.install(self, appuri, additionnals, arguments, url, destination)
        self.__url = url

    def pre_install(self, execution_config_path, global_config, dut_config, sysdebug_apis=None):
        """
        Pre installation actions

        :type execution_config_path: str
        :param execution_config_path: The path of the configuration of ACS

        :type global_config: Dictionnary
        :param global_config: Global configuration of ACS

        :type dut_config: Dictionnary
        :param dut_config: The configuration of the DUT
        """
        IApplication.pre_install(self, execution_config_path,
                                 global_config, dut_config, sysdebug_apis)

        self._wait_btwn_cmd = float(dut_config.get("waitBetweenCmd"))

        if self._parameters.get("wifi_connect", True):
            self._wifi_router = global_config.benchConfig.get_parameters("WPA_WIFI_ROUTER")
            self._ssid = self._wifi_router.get_param_value("SSID")
            self._passphrase = self._wifi_router.get_param_value("passphrase")
            self._security = self._wifi_router.get_param_value("WIFI_SECURITY")

            self._phonesystem.display_on()
            self.__wifi_enable()
            self.__wifi_connection()
            self._phonesystem.display_off()

    def post_install(self):
        """
        Post installation configuration
        """
        self._logger.debug("Software release: %s " % self._sw_release)
        self._logger.debug("Model number: %s " % self._model_number)
        self._logger.debug("Kernel version: %s " % self._kernel_version)
        self._logger.debug("FW version: %s " % self._fw_version)
        self._logger.debug("Device ID: %s " % self._device_id)

        IApplication.post_install(self)
        self.__parse_arguments()
        self.__set_browser()
        self.__set_orientation()

    def start(self):
        """
        Start benchmark
        """
        self._networking.close_web_browser(self._browser)
        self.adb_shell("rm -r %s" % self._browser_cache_dir, 3)
        self._logger.info(self._browser)
        self._networking.open_web_browser(website_url=self.__url, browser_type=self._browser, skip_eula=True)

    def stop(self):
        """
        Stop sunspider benchmark
        """
        IApplication.stop(self)
        self._networking.close_web_browser(self._browser)

    def uninstall(self):
        """
        Uninstall configuration
        """
        if self._display_orientation:
            self._display.set_display_orientation("auto")
