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
:summary: This file is the implementation of
            MODEM_ACTIVATE_MODEM_TRACE use case
:since: 03/06/2013
:author: jreynaux
"""

import time
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global, str_to_bool
from acs_test_scripts.Device.UECmd.UECmdTypes import BPLOG_LOC
from ErrorHandling.AcsConfigException import AcsConfigException


class ModemActivateTrace(UseCaseBase):

    """
    This UC will allows user to activate modem tracing, test steps :
    - configure modem trace
    - reboot phone
    - activate modem trace
    - check file is growing up
    optional steps:
    - deactivate trace
    - reset configuration
    - reboot device
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # Initialize modem trace test parameters
        self._var_platform_pf450cl = False
        self._var_modem_7160 = False

        # Get TC Parameters

        # mean "-u / -d / -h" on configuration script
        self._hsi_speed = self._tc_parameters.get_param_value("HSI_SPEED")

        # mean "-tX" on configuration script
        self._trace_level = int(self._tc_parameters.
                                get_param_value("TRACE_LEVEL"))

        # mean sd or e on activation script
        self._log_location = self._tc_parameters.get_param_value("LOG_LOCATION")

        # mean 1 or 2 for sd / e on activation script
        self._file_size_option = int(self._tc_parameters.
                                     get_param_value("FILE_SIZE_OPTION"))

        # time where to control size growing
        self._time_period = int(self._tc_parameters.
                                get_param_value("TIME_PERIOD"))

        # target percentage of time period where to control size growing
        self._target_rate = int(self._tc_parameters.
                                get_param_value("TARGET_RATE"))

        # First command to configure modem trace
        self._configure_modem_trace1 = self._tc_parameters.get_param_value("COMMAND_CONFIG_1")

        # Second command to configure modem trace
        self._configure_modem_trace2 = self._tc_parameters.get_param_value("COMMAND_CONFIG_2")

        # Second command to configure modem trace
        self._configure_modem_trace3 = self._tc_parameters.get_param_value("COMMAND_CONFIG_3")

        # Command to stop configure modem trace
        self._configure_modem_stop = self._tc_parameters.get_param_value("COMMAND_CONFIG_STOP")

        # target size needed to control size growing
        self._target_size = int(self._tc_parameters.
                                get_param_value("TARGET_SIZE"))

        # Compute the target size in bytes
        self._target_size = self._target_size*1024*1024

        self._hw_reboot = \
            str_to_bool(self._tc_parameters.get_param_value("HW_REBOOT"))

        self._restore_initial = \
            str_to_bool(self._tc_parameters.get_param_value("RESTORE_INITIAL"))

        # Get UECmdLayer
        self._modem_api = self._device.get_uecmd("Modem")

        # Instantiate UE Command categories
        self._networking_api = self._device.get_uecmd("Networking")

        # pylint: disable=E1101
        self._bplog_loc = {
            "SDCARD": BPLOG_LOC.SDCARD,
            "EMMC": BPLOG_LOC.EMMC}

        # pylint: enable=E1101

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """
        UseCaseBase.set_up(self)

        return_code = Global.SUCCESS
        return_msg = "No errors"

        # Disable flight mode
        self._networking_api.set_flight_mode("off")

        # Configure Modem Trace
        # Special treatment required for Saltbay 7160 and MOFD 7160
        # Temporary solution as only high and critical issues are accepted on imin_legacy

        # Retrieve platform properties
        hardware_config_cmd = "adb shell getprop ro.product.name"
        (return_code, output) = self._device.run_cmd(hardware_config_cmd,
                                                    10,
                                                    force_execution=True)
        if (return_code is Global.FAILURE):
            return_msg = "Command %s has failed" % hardware_config_cmd
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, return_msg)

        # Check if the platform is pf450cl or MOFD 7160
        if ("pf450cl" in output):
            self._var_platform_pf450cl = True
            self._var_modem_7160 = True
        else:
            # Retrieve software properties
            software_config_cmd = "adb shell getprop ro.swconf.info"
            (return_code, output) = self._device.run_cmd(software_config_cmd,
                                                        10,
                                                        force_execution=True)
            if (return_code is Global.FAILURE):
                return_msg = "Command %s has failed" % software_config_cmd
                raise AcsConfigException(AcsConfigException.OPERATION_FAILED, return_msg)

            # Platform under test is a MOFD 7160
            if ("V1_7160" in output):
                self._var_modem_7160 = True

        if ( self._var_modem_7160 is True):
            if (self._var_platform_pf450cl is True):
                self._configure_modem_trace1 = "adb shell %s" % self._configure_modem_trace1
                (return_code, output) = self._device.run_cmd(self._configure_modem_trace1,
                                                        10,
                                                        force_execution=True)
                if (return_code is Global.FAILURE):
                    return_msg = "Command %s has failed" % self._configure_modem_trace1
                    raise AcsConfigException(AcsConfigException.OPERATION_FAILED, return_msg)

            self._configure_modem_trace2 = "adb shell %s" % self._configure_modem_trace2
            (return_code, output) = self._device.run_cmd(self._configure_modem_trace2,
                                                        10,
                                                        force_execution=True)
            if (return_code is Global.FAILURE):
                return_msg = "Command %s has failed" % self._configure_modem_trace2
                raise AcsConfigException(AcsConfigException.OPERATION_FAILED, return_msg)

            time.sleep(1)
            self._configure_modem_trace3 = "adb shell %s" % self._configure_modem_trace3
            (return_code, output) = self._device.run_cmd(self._configure_modem_trace3,
                                                        10,
                                                        force_execution=True)
            if (return_code is Global.FAILURE):
                return_msg = "Command %s has failed" % self._configure_modem_trace3
                raise AcsConfigException(AcsConfigException.OPERATION_FAILED, return_msg)

            time.sleep(2)
            (return_code, output) = self._device.run_cmd("adb shell echo 'configure_trace_modem -d -t5 SUCCESS. Your board needs a HARDWARE reboot'",
                                                        10,
                                                        force_execution=True)

        else:
            # Configure modem trace for all other platforms
            self._modem_api.configure_modem_trace(self._hsi_speed,
                                              self._trace_level)

        # Wait a while
        time.sleep(self._wait_btwn_cmd)

        # Enable Flight Mode
        self._logger.info("Now enabling Flight Mode for modem reset purpose")
        self._networking_api.set_flight_mode("on")

        # Wait a while
        time.sleep(30)

        # Disable Flight Mode
        self._logger.info("Disable Flight Mode.")
        self._networking_api.set_flight_mode("off")

        # Wait a while
        time.sleep(self._wait_btwn_cmd)

        # Transform as enum element
        log_location = self._bplog_loc[self._log_location.upper()]

        # Wait a while to ensure MTS is up
        time.sleep(60)

        self._modem_api.activate_modem_trace(log_location,
                                             self._file_size_option)

        return return_code, return_msg

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """
        # Call UseCase base Run function
        UseCaseBase.run_test(self)

        # Wait a while
        time.sleep(10)

        # Check that the BP logging is active
        # pylint: disable=E1101
        self._logger.info("Checking BP logging status...")
        # pylint: enable=E1101

        # Retrieve the current bplog size
        current_size = self._modem_api._get_bplog_file_size()

        if current_size>self._target_size:
            status = Global.SUCCESS
            output = "BPlog size already bigger than the target, %d"
        else:
            while (current_size<self._target_size):
                (status, output) = self._modem_api.check_bplog_file_growing(
                            self._time_period,
                            self._target_rate)
                current_size = self._modem_api._get_bplog_file_size()

        if (self._hw_reboot is True):
            # HW reset the device
            self._device.disconnect_board()
            self._io_card.usb_host_pc_connector(False)
            time.sleep(5)
            self._io_card.press_power_button(10)
            time.sleep(15)
            self._io_card.press_power_button(3)
            time.sleep(25)
            self._io_card.usb_host_pc_connector(True)
            time.sleep(5)
            self._device.connect_board()
        else:
            # Reboot phone
            self._device.reboot(
                mode="MOS",
                wait_for_transition=True,
                wait_settledown_duration=True)

        # Wait a while
        time.sleep(10)

        # Check that the BP logging is active
        # pylint: disable=E1101
        self._logger.info("Checking BP logging status after reboot...")
        # pylint: enable=E1101
        (status, output) = self._modem_api.check_bplog_file_growing(
            self._time_period,
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

        # If value as been set on tc parameters, deactivate traces
        # in order to avoid disturbing next tests
        if self._restore_initial:
            self._logger.info("Deactivating modem trace  ")

            # stopping service
            self._modem_api.deactivate_modem_trace()

            # stop tracing
            self._modem_api.configure_modem_trace("u", 0)

            # If the platform under test is a SB 7160 or MOFD 7160
            if (self._var_modem_7160 is True):
                self._configure_modem_stop = "adb shell %s" % self._configure_modem_stop
                (return_code, output) = self._device.run_cmd(self._configure_modem_stop,
                                                            10,
                                                            force_execution=True)

            # Wait a while
            time.sleep(self._wait_btwn_cmd)

            # Enable Flight Mode
            self._logger.info("Now enabling Flight Mode for modem reset purpose")
            self._networking_api.set_flight_mode("on")

            # Wait a while
            time.sleep(30)

            # Disable Flight Mode
            self._logger.info("Disable Flight Mode.")
            self._networking_api.set_flight_mode("off")

        # Wait a while
            time.sleep(30)

        else:
            # Otherwise the log will remain active
            self._logger.warning("Will not stop modem trace, next tests"
                                 " will continue tracing !")
            output_message = "No error, but trace still activated"

        return return_code, output_message
