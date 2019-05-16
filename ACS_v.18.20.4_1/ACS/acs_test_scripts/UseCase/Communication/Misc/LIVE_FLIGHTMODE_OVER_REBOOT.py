"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
The source code contained or described herein and all documents related
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
:summary: This file implements the flight mode over reboot test
:since: 23/11/2012
:author: cbresoli
"""

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global, str_to_bool
from ErrorHandling.DeviceException import DeviceException


class LiveFlightModeOverReboot(UseCaseBase):

    """
    Flight mode over reboot, test steps :
    - activate flight mode
    - set flight mode as desired
    - reboot phone
    - check flight mode still in desired state
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCaseBase init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # Get FLIGHT_MODE_ENABLED from parameters
        self._is_flight_mode_enabled = str_to_bool(self._tc_parameters.get_param_value("FLIGHT_MODE_ENABLED"))

        # Get UECmdLayer
        self._networking_api = self._device.get_uecmd("Networking")

        self._initial_flight_mode_state = None

        self._expected_flight_mode_state = None
        self._expected_flight_mode_state_str = ""

        self._current_flight_mode_state = None
        self._current_flight_mode_state_str = ""

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Test setup
        """

        UseCaseBase.set_up(self)

        # Recording initial state before starting the test to put it back when test is over
        self._initial_flight_mode_state = self._networking_api.get_flight_mode()

        # Set flight mode to wanted state
        if self._is_flight_mode_enabled:
            self._networking_api.set_flight_mode("on")
            self._expected_flight_mode_state_str = "on"
            self._expected_flight_mode_state = 1
        else:
            self._networking_api.set_flight_mode("off")
            self._expected_flight_mode_state_str = "off"
            self._expected_flight_mode_state = 0

        msg = "Flight mode is %s before reboot" % self._expected_flight_mode_state_str
        self._logger.info(msg)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        UseCaseBase.run_test(self)

        # Reboot phone
        self._device.reboot()

        # Get flight mode status after reboot
        self._current_flight_mode_state = self._networking_api.get_flight_mode()
        if self._current_flight_mode_state == 1:
            self._current_flight_mode_state_str = "on"
        else:
            self._current_flight_mode_state_str = "off"

        # Checks that flight mode state is matching the expected state
        if self._current_flight_mode_state == self._expected_flight_mode_state:
            msg = "Flight mode is %s before and after reboot" % self._current_flight_mode_state_str
            self._logger.info(msg)
        else:
            error_msg = "Flight mode was %s before reboot and is %s after reboot!" \
                % (self._expected_flight_mode_state_str, self._current_flight_mode_state_str)
            self._logger.error(error_msg)
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, error_msg)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        Tear down function
        """

        UseCaseBase.tear_down(self)

        # Set flight mode back to initial state
        self._networking_api.set_flight_mode(self._initial_flight_mode_state)

        return Global.SUCCESS, "No errors"
