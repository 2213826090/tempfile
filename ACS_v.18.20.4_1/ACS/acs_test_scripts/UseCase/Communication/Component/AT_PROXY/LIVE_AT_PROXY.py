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
:summary: This file implements AT Proxy Use Case via GTester.
:since: 15/04/2013
:author: dgonza4x
"""

from UtilitiesFWK.Utilities import Global, is_number
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.Utilities.CommunicationUtilities import SerialHandler, ATCommandAnalyser
from acs_test_scripts.Device.UECmd.Imp.Android.Common.System.ModemFlashing import ModemFlashing
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LiveAtProxy(UseCaseBase):

    """
    Live AT proxy class, used for checking that AT proxy is functional.
    """

    def __init__(self, tc_name, global_config):
        """
        Initializes this instance.
        """

        # Call UseCase base __init__ method
        UseCaseBase.__init__(self, tc_name, global_config)

        # Retrieve testcase parameters
        self.__launch_mode = self._tc_parameters.get_param_value("LAUNCH_MODE")
        self.__command = self._tc_parameters.get_param_value("COMMAND")
        self.__expected_result = \
            self._tc_parameters.get_param_value("EXPECTED_RESULT")
        self.__command_timeout = \
            self._tc_parameters.get_param_value("COMMAND_TIMEOUT")

        # initialize at proxy connection parameters
        self.__serial_handler = SerialHandler()

        # Instantiate the modem UE commands
        self.__modem_flashing_api = self._device.get_uecmd("ModemFlashing")

        # Check input parameter LAUNCH_MODE
        possible_launch_mode = {"NORMAL": ModemFlashing.AT_PROXY_NORMAL_MODE,
                                "TUNNELING": ModemFlashing.AT_PROXY_TUNNELING_MODE}
        if self.__launch_mode in possible_launch_mode.keys():
            self.__launch_mode = possible_launch_mode[self.__launch_mode]
        else:
            self.__launch_mode = None

        # Check input parameter COMMAND_TIMEOUT
        if is_number(self.__command_timeout) is True:
            self.__command_timeout = int(self.__command_timeout)
        else:
            self.__command_timeout = None

    def set_up(self):
        """
        Initializes this test.
        """
        # Run the inherited 'set_up' method
        UseCaseBase.set_up(self)

        # Raise an exception if at least one of checked parameters is None
        # (meaning unknown)
        if self.__launch_mode is None:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                   "Unknown LAUNCH_MODE value")
        if self.__command_timeout is None:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                   "COMMAND_TIMEOUT value must be a number")

        # Check that the phone is in Main OS
        boot_mode = self._device.get_boot_mode()
        if boot_mode != "MOS":
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE,
                                   "Usecase is only available with a board booted in Main OS")

        # Start AT proxy, according to LAUNCH_MODE
        self._logger.info("Starting AT proxy")
        at_proxy_tty = \
            self.__modem_flashing_api.start_at_proxy_from_mos(self.__launch_mode)

        # Setup AT proxy serial connection
        self.__serial_handler.set_data_analyser(
            ATCommandAnalyser(self.__expected_result))
        self.__serial_handler.set_default_timeout(self.__command_timeout)
        self._logger.info("Connecting to the port " + str(at_proxy_tty))

        # Connect to the at proxy
        self.__serial_handler.connect(at_proxy_tty)

        # Check that the modem is available
        modem_available = self.__modem_flashing_api.ping_modem_from_serial(
            self.__serial_handler.get_serial())

        if not modem_available:
            return Global.FAILURE, "AT proxy hasn't been correctly launched"

        # Return SUCCESS status
        return Global.SUCCESS, "No error"

    def run_test(self):
        """
        Executes the test
        """
        # Call the usecase base run_test method
        UseCaseBase.run_test(self)

        # Execute the AT command 'COMMAND' through the serial connection
        # Wait a maximum of COMMAND_TIMEOUT seconds for result
        # Compare command result with EXPECTED_RESULT
        # Return verdict
        self._logger.info("Executing AT command: " + str(self.__command) +
                          " and checking result until " + str(self.__command_timeout) +
                          " seconds")
        verdict, msg = self.__serial_handler.write_and_analyse(self.__command)
        self._logger.info("AT command result: " + str(msg))

        # Return verdict
        return verdict, msg

    def tear_down(self):
        """
        Disposes this test.
        """
        # call the usecase base tear_down method
        UseCaseBase.tear_down(self)

        # Stop AT proxy
        self.__modem_flashing_api.stop_at_proxy_from_mos()

        # Close the serial connection
        self._logger.info("Disconnecting from the port " +
                          str(self.__serial_handler.get_port()))
        self.__serial_handler.disconnect()

        # Return SUCCESS status
        return Global.SUCCESS, "No error"
