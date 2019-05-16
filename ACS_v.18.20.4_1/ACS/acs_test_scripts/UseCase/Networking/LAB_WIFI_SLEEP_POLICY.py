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
:summary: This file implements the LIVE WIFI WAKES UP UC
:since: 30/03/2012 BZ2409
:author: apairex
"""
import time
import math
from acs_test_scripts.UseCase.Networking.LAB_WIFI_BASE import LabWifiBase
from UtilitiesFWK.Utilities import Global
from datetime import datetime
import os
import numpy
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser


class LabWifiSleepPolicy(LabWifiBase):

    """
    Lab Wifi wakes up test.
    """

    # Set the screen timeout (in sec) to a very small value
    SCREEN_TIMEOUT = 15

    # SCAN acceptance criteria:
    # The computed mean should be in the -15%, +15% bounding box
    # around the expected value.
    # The standard deviation should be lower than 15% of the expected value
    DEFAULT_TOLERANCE = .15

    FAKE_SSID = "FAKE_SSID"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWifiBase.__init__(self, tc_name, global_config)

        self._connected = \
            str(self._tc_parameters.get_param_value("WIFI_CONNECTED"))
        self._display_state = \
            str(self._tc_parameters.get_param_value("DISPLAY")).upper()
        self._keep_wifi_on = \
            str(self._tc_parameters.get_param_value("KEEP_WIFI_ON_DURING_SLEEP"))
        self._ssid_list_empty = \
            str(self._tc_parameters.get_param_value("REMEMBERED_SSID_LIST_EMPTY"))
        self._usb_pluggedin = \
            str(self._tc_parameters.get_param_value("USB_PLUGGED_IN"))
        self._tolerance = str(self._tc_parameters.get_param_value("TOLERANCE"))

        # Behavior informations:
        # 1- Scan period in seconds
        self._scan = 0
        # 2- Time to WiFi turn off in minutes
        self._wifi_off = 0
        # 3- Recommended test duration in seconds
        self._tduration = 0

        self._wifi_mac = ""

        self._original_screen_timeout = None

        # USB Connected stated
        self._usb_connected_status = True

        # Set tolerance parameter
        if self._tolerance.isdigit():
            self._tolerance = float(self._tolerance) / 100
        else:
            self._tolerance = self.DEFAULT_TOLERANCE
        self._logger.debug("Tolerance set to %2.2f%%" % (self._tolerance * 100))

        # Forbid the usage of the sniffer in LAB_WIFI_BASE
        self._sniffer = None
        self._sp_sniffer = None

        # Get WiFi interface name
        self._wifi_interface = str(self._dut_config.get("wlanInterface"))

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LabWifiBase.set_up(self)

        # Check input testcase parameters
        self.__check_input_tcparameter()

        # Retrieve the expected wifi behavior depending on the input params
        self._scan, self._wifi_off, self._tduration, desc = \
            ConfigsParser("Wifi_Sleep_Policies").\
            parse_wifi_sleep_policies(self._device.get_phone_model(),
                                      self._connected,
                                      self._display_state,
                                      self._keep_wifi_on,
                                      self._usb_pluggedin,
                                      self._ssid_list_empty)
        if self._scan == 0 \
                and self._wifi_off == 0 \
                and self._tduration == 0:
            msg = "No Wifi sleep policy found in Wifi_Sleep_Policies.XML"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        if desc:
            self._logger.info("TestCase to run: %s" % desc)
        self._logger.debug("Scan period (s): %d" % self._scan)
        self._logger.debug("Time to WiFi turn off (min): %d" % self._wifi_off)
        self._logger.debug("Recommended test duration (s): %d" %
                           self._tduration)

        # Initialize the Sniffer equipment
        self._sp_sniffer = self._em.get_sniffer("WIFI_SNIFFER1")

        # Apply WIFI_CONNECTED initial condition
        if self._connected.lower() == "false":
            self._networking_api.wifi_remove_config('all')

            # Apply REMEMBERED_SSID_LIST_EMPTY initial condition
            if self._ssid_list_empty.lower() == "false":
                # Create non existing WiFi network in the known network list
                self._networking_api.set_wificonfiguration(self.FAKE_SSID,
                                                           None,
                                                           "OPEN")

        # Non consistent case
        elif self._ssid_list_empty.lower() == "true":
            self._logger.warning("TC is requested with WiFi CONNECTED," +
                                 " whereas REMEMBERED_SSID_LIST has to be EMPTY. " +
                                 "Test will be run CONNECTED " +
                                 "and REMEMBERED_SSID_LIST_EMPTY is ignored.")

        # Apply KEEP_WIFI_ON_DURING_SLEEP initial condition
        if self._keep_wifi_on.lower() == "always":
            policy = self._networking_api.WIFI_SLEEP_POLICY["NEVER"]
        elif self._keep_wifi_on.lower() == "only_when_plugged_in":
            policy = \
                self._networking_api.WIFI_SLEEP_POLICY["NEVER_WHILE_PLUGGED"]
        else:
            # "never"
            policy = self._networking_api.WIFI_SLEEP_POLICY["WHEN_SCREEN_OFF"]
        self._networking_api.set_wifi_sleep_policy(policy)

        # Retrieve WiFi MAC Address
        self._wifi_mac = self._networking_api.get_interface_mac_addr("wifi")

        # Unlock the WiFi power saving mode
        self._networking_api.set_wifi_power_saving_mode(1)

        # Apply display initial condition
        if self._display_state == "OFF":
            self._original_screen_timeout = \
                self._phone_system_api.get_screen_timeout()
            # Set display off timeout to its minimal value
            self._phone_system_api.set_screen_timeout(self.SCREEN_TIMEOUT)
            time.sleep(self.SCREEN_TIMEOUT)

            # Go back to idle screen and turn screen off
            self._phone_system_api.display_on()
            self._phone_system_api.set_phone_lock(0)
            self._networking_api.wifi_menu_settings(False)
            self._phone_system_api.set_phone_lock(1)
            self._phone_system_api.display_off()

        else:
            # case if self._display_state == "ON_OUT_WIFI_MENU"
            is_in = False
            if self._display_state == "ON_IN_WIFI_MENU":
                is_in = True
            # Prevent screen back-light to turn off
            self._phone_system_api.display_on()
            time.sleep(self._wait_btwn_cmd)

            # Unlock the device
            self._phone_system_api.set_phone_lock(0)
            time.sleep(self._wait_btwn_cmd)

            # Open or exit WiFi settings menu
            self._networking_api.wifi_menu_settings(is_in)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LabWifiBase.run_test(self)

        if self._wifi_off > 0:
            # WiFi interface OFF setup
            self.__compute_test_duration("WIFI_OFF")
            self._networking_api.\
                measure_wifi_turns_off_duration("async_init",
                                                self._wifi_interface,
                                                duration=self._tduration)

        # Unplug the USB cable if necessary
        if self._usb_pluggedin.lower() == "false":
            # We have to disable S3 in order to keep DUT active when unplugged
            self._sleep_mode_api.init("s0i3")

            self._device.disconnect_board()
            self._usb_connected_status = False
            self._io_card.usb_host_pc_connector(False)

        if self._scan > 0:
            # WiFi scan test
            self.__run_scan_test(False)

        elif self._scan == -1:
            # PERIODIC scan (every 40s for the first 5min then is every 5min)
            self.__run_periodic_scan_test()

        elif self._wifi_off > 0:
            # Wifi OFF test
            self._logger.info("Sleep %d sec" % self._tduration)
            time.sleep(self._tduration)

        elif self._scan == 0 and self._wifi_off == 0 and self._tduration > 0:
            # No scan test
            self.__run_scan_test(True)

        else:
            msg = "Unhandled test scenario"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.PROHIBITIVE_BEHAVIOR, msg)

        # Plug the USB cable back if necessary
        if self._usb_pluggedin.lower() == "false":
            self._io_card.usb_host_pc_connector(True)
            self._device.connect_board()
            self._usb_connected_status = True

            # Enable S3
            self._sleep_mode_api.clear()

        if self._wifi_off > 0:
            # WiFi interface OFF teardown
            self._networking_api.\
                measure_wifi_turns_off_duration("async_result",
                                                self._wifi_interface,
                                                expected_duration=self._wifi_off * 60,
                                                tolerance=self._tolerance)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        self._logger.info("-- Tear down starts --")
        if not self._usb_connected_status:
            self._io_card.usb_host_pc_connector(True)
            self._device.connect_board()
            self._usb_connected_status = True
            # Enable S3
            self._phone_system_api.clear_sleep_mode("s0i3")

        if self._connected.lower() == "false" \
                and self._ssid_list_empty.lower() == "false":
            self._networking_api.wifi_remove_config(self.FAKE_SSID)

        LabWifiBase.tear_down(self)

        self._networking_api.set_wifi_sleep_policy(
            self._networking_api.WIFI_SLEEP_POLICY["DEFAULT"])

        # Relock the phone and remove the backlight on lock
        self._networking_api.wifi_menu_settings(False)
        self._phone_system_api.set_phone_lock(1)
        self._phone_system_api.display_off()

        # Set display off timeout to its original value
        if self._original_screen_timeout is not None:
            self._phone_system_api.\
                set_screen_timeout(self._original_screen_timeout)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def __compute_datelist(self, datelist):
        """
        Compute Date list in order to extract mean delta time and deviation
        delta time <= 0.5 s are ignored

        :type datelist: list of floats
        :param datelist: list of date in seconds

        :rtype: list of 2 floats
        :return: mean delta time and deviation
        """
        # Compute delta time array
        deltas = list()
        prev = datelist[0]
        for nextdate in datelist[1:]:
            delta = nextdate - prev
            if delta > 0.5:
                deltas.append(delta)
            prev = nextdate

        if len(deltas) < (self._tduration / self._scan) - 1:
            self._logger.warning("Delta list is too small: %s - Expected %s values" %
                                 (str(deltas), str((self._tduration / self._scan) - 1)))
            return [float(-1), float(-1)]

        return [float(numpy.mean(deltas)), float(numpy.std(deltas))]

    def __scan_generate_verdict(self, expected_value, mean_measure, std_dev):
        """
        Analyze the measured scan period and
        raise Exception in case of failure

        :type expected_value: int
        :param expected_value: expected scan period in seconds

        :type mean_measure: float
        :param mean_measure: Mean measured scan periods

        :type std_dev: float
        :param std_dev: standard deviation for measured scan periods

        """
        if mean_measure < 0 or math.isnan(mean_measure):
            msg = "No WiFi scan observed"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        lower_bound = expected_value * (1 - self._tolerance)
        upper_bound = expected_value * (1 + self._tolerance)
        if mean_measure < lower_bound or mean_measure > upper_bound:
            msg = "Mean scan period (%f) != expected value [%f, %f]" % \
                (mean_measure, lower_bound, upper_bound)
            self._logger.error(msg)
            raise DeviceException(DeviceException.PROHIBITIVE_MEASURE, msg)

        if std_dev == 0 or math.isnan(std_dev):
            msg = "Standard deviation error : %f" % std_dev
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        threshold = expected_value * self._tolerance
        if std_dev > threshold:
            msg = "Scan period measure is not relevant. "
            msg += "Standard Deviation = %f. " % std_dev
            msg += "Expected value is lower than %f" % threshold
            self._logger.error(msg)
            raise DeviceException(DeviceException.PROHIBITIVE_MEASURE, msg)

        msg = "Mean scan period: %f " % mean_measure
        msg += "Std deviation: %f " % std_dev
        msg += "(expected value: %d)" % expected_value
        self._logger.info(msg)

    def __build_tmp_capture_filename(self):
        """
        Build a temporary filename for the sniff capture file

        :rtype: str
        :return: temporary file name located in the campaign report folder
        """
        filename = "capture-%s.cap" % datetime.now().strftime("%Hh%M.%S")
        pathname = self._device.get_report_tree().get_report_path()
        sniff_tmplog = os.path.join(pathname, filename)
        # We assume that on Windows the current drive is the ACS one
        if sniff_tmplog[1] == ":":
            # Remove the drive letter
            sniff_tmplog = sniff_tmplog[2:]
        return sniff_tmplog

    def __compute_test_duration(self, test_type):
        """
        Compute test duration attribute

        :type test_type: str
        :param test_type: Type of test to run. Can be "SCAN", "...", "..."
        """
        if test_type == "SCAN":
            # Minimum test duration should be a little more than twice the
            # expected scan period, in order to be able to sniff 2 scan actions
            min_duration = int(math.ceil(2.1 * self._scan))

        elif test_type == "PERIODIC_SCAN":
            # Minimum test duration should be more than 3 times 5 minutes:
            min_duration = 16 * 60

        elif test_type == "WIFI_OFF":
            # Minimum test duration should be: expected time for WiFi to turn
            # OFF + tolerance
            min_duration = \
                math.ceil((self._wifi_off * (1 + self._tolerance)) * 60)

        else:
            msg = "test_type not permitted: " + test_type
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        if self._tduration == 0:
            self._tduration = min_duration
        elif self._tduration < min_duration:
            self._logger.warning(("Minimum test duration is: %d s. " +
                                  "Test duration forced to this value") % min_duration)
            self._tduration = min_duration

    def __check_input_tcparameter(self):
        """
        Check validity of input TestCase parameters
        """
        # Check UC paramters
        if self._connected.lower() not in ["true", "false"]:
            msg = "value not expected for WIFI_CONNECTED: %s" % self._connected
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        if self._display_state not in ["OFF",
                                       "ON_IN_WIFI_MENU",
                                       "ON_OUT_WIFI_MENU"]:
            msg = "value not expected for DISPLAY: %s" % self._display_state
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        if self._keep_wifi_on.lower() not in ["always",
                                              "only_when_plugged_in",
                                              "never"]:
            msg = "value not expected for KEEP_WIFI_ON_DURING_SLEEP: %s" % \
                self._keep_wifi_on
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        if self._ssid_list_empty.lower() not in ["true", "false"]:
            msg = "value not expected for REMEMBERED_SSID_LIST_EMPTY: %s" % \
                self._ssid_list_empty
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        if self._usb_pluggedin.lower() not in ["true", "false"]:
            msg = "value not expected for USB_PLUGGED_IN: %s" % \
                self._usb_pluggedin
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        if not str(self._channel).isdigit():
            msg = "value not expected for CHANNEL: %s" % str(self._channel)
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        if self._sniffer is not None:
            msg = "You cannot activate Wifi Sniff log during this test"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

    def __run_scan_test(self, no_scan=False):
        """
        Execute the process that tests the scan period

        :type no_scan: boolean
        :param no_scan: If True, this function should test no scan is performed
        """
        # Calculate the minimum test duration
        self.__compute_test_duration("SCAN")

        # start the scan period sniff
        sniff_tmplog = self.__build_tmp_capture_filename()

        self._sp_sniffer.init()
        try:
            self._sp_sniffer.start_wifi_scan_monitor(self._channel,
                                                     self._wifi_mac)
        finally:
            self._sp_sniffer.release()

        self._logger.info("waiting %d sec" % self._tduration)
        time.sleep(self._tduration)

        # Stop the scan period sniff
        self._sp_sniffer.init()
        try:
            datelist = self._sp_sniffer.stop_wifi_scan_monitor(sniff_tmplog)
        finally:
            self._sp_sniffer.release()

        if no_scan and len(datelist) > 0:
            msg = "Scan actions occur during test duration: %d" % \
                len(datelist)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        elif not no_scan:
            if len(datelist) < 1:
                msg = "No sniff log observed during %s seconds" % \
                    self._tduration
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            # Compute the result
            mean, std = self.__compute_datelist(datelist)
            self._logger.info("Mean value : %f - Std deviation: %f" %
                              (mean, std))

            # Determine the acceptance criteria
            self.__scan_generate_verdict(self._scan, mean, std)

    def __run_periodic_scan_test(self):
        """
        Execute the process to test the specific PERIODIC scan
        (every 40s for the first 5min then is every 5min)
        """
        # Compute test duration
        self.__compute_test_duration("PERIODIC_SCAN")
        remaining_duration = self._tduration

        # 1st part of the test: sniff during less than 5 minutes : 4m and 30s
        self._tduration = int(4.5 * 60)
        self._scan = 40
        self.__run_scan_test()
        remaining_duration = remaining_duration - self._tduration

        # 2nd part of the test: sniff scan every 5 minutes.
        self._tduration = remaining_duration
        self._scan = 5 * 60
        self.__run_scan_test()
