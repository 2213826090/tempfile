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
:summary: This file implements the LIVE BT Airplane Mode switched ON OFF
:author: cmichelx
:since:05/25/2012
"""

import time

from acs_test_scripts.UseCase.LocalConnectivity.LIVE_BT_BASE import LiveBTBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Device.UECmd.UECmdTypes import BT_STATE
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException


class LiveBTApmOnOff(LiveBTBase):

    """
    Live BT APM on off test.
    Description:
      Initialize device state to normal mode
      Enable/Disable APM and check BT enable auto if BT was previously on
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LiveBTBase init function
        LiveBTBase.__init__(self, tc_name, global_config)

        # Get TestCase parameters
        # Read TURN_APM_SEQUENCE from test case xml file
        self._turn_apm_sequence = \
            str(self._tc_parameters.get_param_value("TURN_APM_SEQUENCE"))

        # Read BT_INIT_MODE from test case xml file
        self._bt_mode = \
            str(self._tc_parameters.get_param_value("BT_INIT_MODE"))
        if self._bt_mode is None:
            self._bt_mode = "on"

        # Read BT_GETMODE_TIMEOUT from test case xml file
        self._bt_getmode_timeout = \
            self._tc_parameters.get_param_value("BT_GETMODE_TIMEOUT")
        if self._bt_getmode_timeout is None \
           or not str(self._bt_getmode_timeout).isdigit():
            self._bt_getmode_timeout = int(self._device._uecmd_default_timeout)  # pylint: disable=W0212
        else:
            self._bt_getmode_timeout = int(self._bt_getmode_timeout)

        # get original bt mode
        self._original_bt_mode = self._bt_api.get_bt_power_status_eot()

        # local variable: BT power state when Airplane mode is disabled
        self._initial_bt_state = self._original_bt_mode

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """

        LiveBTBase.set_up(self)

        # set flight mode
        self._logger.info("set Device to normal mode")
        if self._networking_api.get_flight_mode():
            self._networking_api.set_flight_mode(0)
            time.sleep(self._wait_btwn_cmd)
            if self._networking_api.get_flight_mode():
                msg = "set to normal mode failure"
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # pylint: disable=E1101
        # set BT mode
        if self._bt_mode.strip().lower() in ("on", "1"):
            # Already Powered On in LiveBTBase
            # Store BT power state
            self._initial_bt_state = self._bt_api.get_bt_power_status_eot()
            if self._initial_bt_state != str(BT_STATE.STATE_ON):
                msg = "set BT ON failure"
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        elif self._bt_mode.strip().lower() in ("off", "0"):
            self._logger.info("set BT mode to OFF")
            self._bt_api.set_bt_power("0")
            # Store BT power state
            self._initial_bt_state = self._bt_api.get_bt_power_status_eot()
            if self._initial_bt_state != str(BT_STATE.STATE_OFF):
                msg = "set BT OFF failure"
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        else:
            self._logger.info("Set wrong BT init mode")
            msg = "set wrong BT init mode, failed . alter bt_init_mode in "\
                + "test case xml file"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # Call UseCase base run_test function
        LiveBTBase.run_test(self)

        time.sleep(self._wait_btwn_cmd)

        # begin APM turn on off sequence
        self._logger.info("Airplane mode sequence is :"
                          + self._turn_apm_sequence)
        time.sleep(self._wait_btwn_cmd)

        self._turn_apm_sequence = self._turn_apm_sequence.lower()
        seqlist = self._turn_apm_sequence.strip().split()
        # pylint: disable=E1101
        for switch in seqlist:
            if switch.lower() in ("flight", "on", "1"):
                self._logger.info("try to set airplane mode to flight mode")
                self._networking_api.set_flight_mode(1)
                # CHeck BT is switched off
                if not self._bt_api.bt_power_state_reached(str(BT_STATE.STATE_OFF), self._bt_getmode_timeout):
                    msg = "set BT OFF failure"
                    self._logger.error(msg)
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            elif switch.lower() in ("normal", "off", "0"):
                self._logger.info("try to set airplane mode to normal mode")
                self._networking_api.set_flight_mode(0)
                # CHeck BT is coming back to power state before flight mode
                if not self._bt_api.bt_power_state_reached(self._initial_bt_state, self._bt_getmode_timeout):
                    msg = "switch BT back to previous state failure"
                    self._logger.error(msg)
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            else:
                msg = "input wrong sequence, failed. " \
                    + "Alter your sequence in test case xml file"
                self._logger.error(msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        return Global.SUCCESS, "No errors"

    def tear_down(self):
        """
        End and dispose the test
        """
        LiveBTBase.tear_down(self)

        time.sleep(self._wait_btwn_cmd)

        self._logger.info("Set original BT mode: " + str(self._original_bt_mode))
        self._bt_api.set_bt_power(self._original_bt_mode)
        if self._original_bt_mode != self._bt_api.get_bt_power_status_eot():
            msg = "Set BT original mode failure"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return Global.SUCCESS, "No errors"
