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
:summary: This file implements the LAB sleep usb UC to measure residency
:since: 14/03/2012
:author: ssavrimoutou
"""
import os
import time
from Device.DeviceManager import DeviceManager
from SYSTEM_SLEEP_BASE import SystemSleepBase
from acs_test_scripts.Utilities.PnPUtilities import PnPResults
from UtilitiesFWK.Utilities import Global
from ErrorHandling.DeviceException import DeviceException


class LiveSystemSleepResidencyMeasurement(SystemSleepBase):

    """
    Sleep mode residency measurement class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor

        :type tc_name: BaseConf
        :param tc_name: Configuration of the usecase

        :type global_config: Dictionary
        :param global_config: Global configuration of the campaign
        """

        # Call UseCase base constructor
        self._device = DeviceManager().get_device("PHONE1")

        self.__adbConnectionTimeout = self._device.get_config("adbConnectTimeout", 30, float)
        self._device_uptime_begin = None
        self._sleep_duration = None

        # If the device was disconnected before due to an error
        # we must reconnect it explicitly at the beginning of the test
        # else some commands will not be executed and the test will be blocked
        self._system_api = self._device.get_uecmd("System")
        return_code = self._system_api.wait_for_device(self.__adbConnectionTimeout)
        if not return_code:
            time.sleep(30)

        if not self._device.is_available():
            self._device.connect_board()

        # Call SystemSleepBase base Init function
        SystemSleepBase.__init__(self, tc_name, global_config)

        self._report_tree = global_config.campaignConfig.get("campaignReportTree")
        self._sysdebug_apis = self._device.get_uecmd("SysDebug")

        self._tc_name = os.path.basename(self.get_name())
        self._tc_date = ""
        attributes = {"id": self._tc_name,
                      "date": self._tc_date}

        self.__results = PnPResults(self._report_tree,
                                    self._dut_config.get("Name"),
                                    self._failure_file,
                                    None,
                                    attributes)

    def run_test(self):
        """
        Execute the test
        """

        sysdbg_modules_config = self._tc_parameters.get_param_value("SYSDEBUG_MODULES")
        self._sysdebug_apis.init(sysdbg_modules_config)

        sleep_parameter = self._tc_parameters.get_param_value("SLEEP_DURATION")
        if sleep_parameter is not None and sleep_parameter != "" and sleep_parameter.isdigit():
            self._sleep_duration = int(sleep_parameter)

        # Call live sleep use case base
        SystemSleepBase.run_test(self)

        self._tc_date = time.strftime("%Y-%m-%d %H:%M:%S")
        self.__results.update({"date": self._tc_date})

        while not self._sysdebug_apis.synchronize():
            time.sleep(10)

        adbConnectTimeout = self._device.get_config("adbConnectTimeout", 10, float)
        usbReplugRetries = self._device.get_config("usbReplugRetries", 1, int)

        if self._io_card is not None:
            self._device.disconnect_board()
            self._sysdebug_apis.reset() # will clear mid_pmu_states
            self._residency_api.clear(self._sleep_duration) # will also clear mid_pmu_states
                                        # but this is needed to do the fetch on this instance
            self._io_card.usb_host_pc_connector(False)
            # Unplug wall charger only if it is AC_CHGR
            if self._device.get_default_wall_charger() == self._io_card.AC_CHGR:
                self._io_card.wall_charger_connector(False)

        # Update device uptime
        updated, self._device_uptime_begin = self._device._update_device_up_state(0)
        if not updated:
            self._device_uptime_begin = None

        if self._sleep_duration:
            self._logger.info("Wait for %s s before measurement (sleep duration before %s)" %
                              (str(self._sleep_duration), self._sleep_mode))
            time.sleep(self._sleep_duration)

        self._sysdebug_apis.start()
        self._logger.info("Wait for %s s to enter in %s" % (str(self._duration), self._sleep_mode))
        time.sleep(self._duration)
        self._sysdebug_apis.stop()

        residency_spent = 0
        ret_code = None

        if self._io_card is not None:
            for cnt in range(0, usbReplugRetries + 1):
                self._logger.debug("Loop Iteration: %d" % cnt)
                # plug wall charger only if it is AC_CHGR
                if self._device.get_default_wall_charger() == self._io_card.AC_CHGR:
                    self._io_card.wall_charger_connector(True)
                self._io_card.usb_host_pc_connector(True)

                self._logger.debug("Wait for device %s seconds" % self.__adbConnectionTimeout)
                ret_code = self._system_api.wait_for_device(self.__adbConnectionTimeout)
                self._logger.debug("Wait for device return code: %s" % ret_code)
                if not ret_code:
                    if cnt < usbReplugRetries:
                        self._logger.warning("timeout on wait-for-device, trying to unplug/replug (try %s/%s)"
                                             % (str(cnt + 1), str(usbReplugRetries)))
                        self._io_card.usb_host_pc_connector(False)
                        # Unplug wall charger only if it is AC_CHGR
                        if self._device.get_default_wall_charger() == self._io_card.AC_CHGR:
                            self._io_card.wall_charger_connector(False)
                        time.sleep(10)
                    continue

                residency_spent = self._residency_api.get_value("residency",
                                                                self._sleep_mode_api.get_sleep_mode())
                self._sysdebug_apis.fetch()
                self._device.connect_board()
                self._logger.debug("device retrieved after %s tries" % str(cnt + 1))
                break

            if not ret_code:
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                      "Could not retrieve the device after %s plug/unplug cycles" %
                                      str(usbReplugRetries))

        if residency_spent is None:
            raise DeviceException(DeviceException.INVALID_PARAMETER,
                                  "There is no %s sleep mode for this device model" %
                                  self._sleep_mode_api.get_sleep_mode())

        # Get device uptime and raise an exception if the device rebooted
        if self._device_uptime_begin:
            updated, uptime = self._device._update_device_up_state(self._device_uptime_begin)
            if updated and not self._device.is_up:
                self._logger.warning("the device uptime was %s before the measurement, %s now !"
                                     % (str(self._device_uptime_begin), str(uptime)))
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                      "Device rebooted during the measurement")

        sysreport = self._sysdebug_apis.report()
        self.__results.append(sysreport)
        self.__results.write()

        return self._residency_verdict(residency_spent)
