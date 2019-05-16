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
:summary: This file implements the LIVE BT TETHERING TURN ON OFF
:author: apairex
:since:07/03/2013
"""

import time

from acs_test_scripts.UseCase.LocalConnectivity.LIVE_BT_BASE import LiveBTBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException


class LiveBTTetheringTurnOnOff(LiveBTBase):

    """
    Live BT Tethering Turn on off test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LiveBTBase init function
        LiveBTBase.__init__(self, tc_name, global_config)

        # Read BT_INITIAL_STATE from test case xml file
        self._bt_initial_state = \
            str(self._tc_parameters.get_param_value("BT_INITIAL_STATE")).upper()

        # Read TURN_BLUETOOTH_TETHERING_SEQUENCE from test case xml file
        self._turn_bt_tethering_sequence = \
            str(self._tc_parameters.
                get_param_value("TURN_BLUETOOTH_TETHERING_SEQUENCE")).upper().strip()

        # Get Tethering default state after BT Off/On from device catalog file
        self._tethering_reset = str(self._dut_config.get("TetheringResetAfterBtOffOn"))
        if self._tethering_reset.lower() == "true":
            self._tethering_reset = True
        else:
            # By default, tethering is not reset
            self._tethering_reset = False

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """
        LiveBTBase.set_up(self)

        # Disable BT interface if required
        if self._bt_initial_state in ["OFF", "0", "FALSE", False]:
            self._bt_initial_state = False
            self._bt_api.set_bt_power("off")
            time.sleep(self._wait_btwn_cmd)
        else:
            # BT is enabled in LiveBTBase.set_up
            self._bt_initial_state = True

        # Verify BT tethering state after BT OFF/ON sequence
        # if BT OFF, tethering shall be also OFF
        pan_state = self._bt_api.get_bt_tethering_power()
        if not self._bt_initial_state and pan_state:
            msg = "BT Tethering shall be OFF when BT is OFF"
            self._logger.error(msg)
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, msg)
        elif self._bt_initial_state and self._tethering_reset and pan_state:
            msg = "BT Tethering shall be OFF after BT power OFF/ON sequence"
            self._logger.error(msg)
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, msg)
        elif self._bt_initial_state and not self._tethering_reset:
            msg = "BT Tethering reset state cannot be validated as previous state unknown"
            self._logger.info(msg)
            # ensure BT tethering is OFF
            self._bt_api.set_bt_tethering_power("0")
            time.sleep(self._wait_btwn_cmd)
        else:
            msg = "BT Tethering initial state is OFF"
            self._logger.info(msg)

        return Global.SUCCESS, "No errors"

    def run_test(self):
        """
        Execute the test
        """
        # Call UseCase base run_test function
        LiveBTBase.run_test(self)

        self._logger.info("bluetooth tethering turn on off sequence is :"
                          + self._turn_bt_tethering_sequence)

        # begin bluetooth tethering turn on off sequence
        seqlist = self._turn_bt_tethering_sequence.split()
        for switch in seqlist:
            if switch in ("ON", "1"):
                # Turn ON BT Tethering
                self._logger.info("try to turn on bluetooth tethering")
                self._bt_api.set_bt_tethering_power("1")
                time.sleep(self._wait_btwn_cmd)

                # Check BT interface activation by ACS_Agent
                if self._bt_initial_state == False and \
                        self._bt_api.get_bt_power_status() != "STATE_ON":
                    msg = "BT interface has not been activated" \
                        + " while BT tethering activation"
                    self._logger.error(msg)
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

            elif switch in ("OFF", "0"):
                # Turn OFF BT Tethering
                self._logger.info("try to turn off bluetooth tethering")
                self._bt_api.set_bt_tethering_power("0")
                time.sleep(self._wait_btwn_cmd)

            else:
                msg = "Wrong input sequence. " \
                    + "Change your sequence in test case xml file: " \
                    + self._turn_bt_tethering_sequence
                self._logger.error(msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        return Global.SUCCESS, "No errors"
