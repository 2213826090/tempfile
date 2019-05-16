"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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
:summary: This file is the implementation of modem trace using AMTL
:since: 27/07/2015
:author: mariussx
"""

import time
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.Device.UECmd.UECmdTypes import BPLOG_LOC
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class ModemActivateTraceAmtl(UseCaseBase):
    """
    This UC will allows user to activate modem tracing, test steps:
        - Retrieve modem trace options
        - activate modem trace option
        - check file is growing up
        - reboot the device
        - check file is growing up
        - deactivate modem trace
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # Get TC Parameters
        # Read the modem trace config which shall be activated
        self._modem_trace_option = \
            self._tc_parameters.get_param_value("TRACE_CONFIG_OPTION")

        # The location for BP logs
        self._log_location = self._tc_parameters.get_param_value("LOG_LOCATION")

        # Time where to control size growing
        self._time_period = int(self._tc_parameters.
                                get_param_value("TIME_PERIOD"))

        # Target percentage of time period where to control size growing
        self._target_rate = int(self._tc_parameters.
                                get_param_value("TARGET_RATE"))

        # Target size needed to control size growing
        self._target_size_mb = int(self._tc_parameters.
                                get_param_value("TARGET_SIZE"))
        self._target_size = None

        self._modem_trace_options_list = []

        # Instantiate UE Command categories
        self._modem_api = self._device.get_uecmd("Modem")
        self._networking_api = self._device.get_uecmd("Networking")
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")

        # pylint: disable=E1101
        self._bplog_loc = {
            "SDCARD": BPLOG_LOC.SDCARD,
            "EMMC": BPLOG_LOC.EMMC,
            "DATA": BPLOG_LOC.DATA}
        # pylint: enable=E1101

        # Gets the initial screen timeout
        self._initial_screen_timeout = self._phonesystem_api.get_screen_timeout()

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        UseCaseBase.set_up(self)

        return_code = Global.SUCCESS
        return_msg = "No errors"

        # Check that value for "TARGET_SIZE" parameter is in the (0,200) MB interval
        if (self._target_size_mb < 0) or (self._target_size_mb > 200):
            err_msg = "Invalid parameter value (%s) for '%s'" % (self._target_size_mb,
                                                                 "TARGET_SIZE",)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     err_msg)
        else:
            # Convert the target size in bytes
            self._target_size = self._target_size_mb*1024*1024

        if self._log_location.upper() not in self._bplog_loc:
            err_msg = "Invalid parameter value for '%s' (%s)" % ("LOG_LOCATION",
                                                                 self._log_location)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     err_msg)

        # Transform as enum element
        log_location = self._bplog_loc[self._log_location.upper()]

        # Set the location for BP logs
        self._modem_api.set_bp_logs_location(log_location)

        # Set phone screen on
        self._phonesystem_api.display_on()
        # Sets the screen timeout to maximum value
        self._phonesystem_api.set_screen_timeout(0)

        # Disable flight mode
        self._networking_api.set_flight_mode("off")

        # Wait 30 seconds
        time.sleep(30)

        # Check if modem trace is already activated
        (status, current_option) = self._modem_api._get_modem_trace_status_amtl()
        if status == "activated":
            self._logger.info("Modem trace is already activated for option " + current_option)
            # Deactivate the modem trace
            self._modem_api.deactivate_modem_trace_amtl(current_option)
            # Wait 15 seconds
            time.sleep(15)

        # Retrieve the modem trace configuration
        self._logger.info("Get the modem trace options from AMTL")
        self._modem_trace_options_list = self._modem_api.get_modem_trace_configuration_amtl()

        if (self._modem_trace_option is None) or \
            (self._modem_trace_option not in self._modem_trace_options_list):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                         "Invalid parameter value for '%s' (%s)" % ("TRACE_CONFIG",
                                                                    str(self._modem_trace_option)))

        return return_code, return_msg

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # Call UseCase base Run function
        UseCaseBase.run_test(self)

        # Activate modem trace
        self._logger.info("Activate modem trace with option: " + self._modem_trace_option)
        self._modem_api.activate_modem_trace_amtl(self._modem_trace_option)

        # Wait a while
        time.sleep(15)

        # Check that the BP logging is active
        self._logger.info("Checking BP logging status...")

        # Retrieve the current bplog size
        current_size = self._modem_api._get_bplog_file_size()

        if current_size > self._target_size:
            status = Global.SUCCESS
            output = "BPlog size already bigger than the target, %d" % self._target_size
        else:
            while current_size < self._target_size:
                (status, output) = \
                    self._modem_api.check_bplog_file_growing(self._time_period,
                                                             self._target_rate)
                current_size = self._modem_api._get_bplog_file_size()

        # Reboot phone
        self._device.reboot(
            mode="MOS",
            wait_for_transition=True,
            wait_settledown_duration=False)

        # Prevent locking the screen
        self._phonesystem_api.set_phone_lock("off")

        # Check that the BP logging is active
        # pylint: disable=E1101
        self._logger.info("Checking BP logging status after reboot...")
        # pylint: enable=E1101
        (status, output) = \
            self._modem_api.check_bplog_file_growing(self._time_period,
                                                     self._target_rate)

        # Return the status
        return status, output

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        UseCaseBase.tear_down(self)

        return_code = Global.SUCCESS
        output_message = "No errors"

        # Set phone screen on
        self._phonesystem_api.display_on()

        try:
            self._logger.info("Deactivating modem trace ...")
            # Deactivate the modem trace
            self._modem_api.deactivate_modem_trace_amtl(self._modem_trace_option)
            # Wait 30 seconds
            time.sleep(30)

        except DeviceException as ex:
            return_code = Global.FAILURE
            output_message = "Fail to deactivate modem trace - exception: " + str(ex)
            self._logger.warning(output_message)

        # Set the initial screen timeout
        self._phonesystem_api.set_screen_timeout(self._initial_screen_timeout)
        # Enable locking the screen
        self._phonesystem_api.set_phone_lock("on")

        # Check that DUT has modem up
        try:
            states = ["registered", "searching", "roaming"]
            self._modem_api.check_cdk_state_bfor_timeout(states, timeout = 60)
        except DeviceException as ex:
            return_code = Global.FAILURE
            output_message = "Registration fail - " + str(ex)
            self._logger.warning(output_message)
            if self._modem_api.get_modem_online_status == 1:
                self._logger.warning("Modem status is OFFLINE")
            else:
                return_code = Global.SUCCESS
                output_message = "Modem status is ONLINE"
                self._logger.info(output_message)

        return return_code, output_message
