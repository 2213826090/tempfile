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
:summary: This file implements the LIVE CSFB VC MO-MR
:since: 01/07/2015
:author: mariussx
"""

import time

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.DeviceException import DeviceException


class LiveLteCsfbVcMoMr(UseCaseBase):

    """
    Live Voice Call MO/MR.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)

        # Read registrationTimeout from Device_Catalog.xml
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

        # Read callSetupTimeout from Phone_Catalog.xml
        self._call_setup_time = \
            int(self._dut_config.get("callSetupTimeout"))

        # Get Test Cases Parameters
        self._numtocall = self._tc_parameters.get_param_value("PHONE_NUMBER")

        self._callduration = \
            int(self._tc_parameters.get_param_value("CALL_DURATION"))

        self._initial_pref_network = None

        # Get UECmdLayer
        self._voicecall_api = self._device.get_uecmd("VoiceCall")
        self._networking_api = self._device.get_uecmd("Networking")
        self._modem_api = self._device.get_uecmd("Modem")

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        UseCaseBase.set_up(self)

        # Disable flight mode
        self._networking_api.set_flight_mode("off")
        time.sleep(self._wait_btwn_cmd)

        # There is a preferred network to set, backup initial, then set configured one
        self._initial_pref_network = self._dut_config.get("defaultPreferredNetwork")
        time.sleep(self._wait_btwn_cmd)

        # Check if DUT remained camped on a LTE cell
        # If not try to force it
        dut_network_type = self._modem_api.get_network_type()
        if dut_network_type != "LTE":
            self._logger.info("Force DUT to camp on LTE")
            self.__force_to_camp_on_lte()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        UseCaseBase.run_test(self)

        # Release any previous call (Robustness)
        self._voicecall_api.release()

        self._voicecall_api.dial(self._numtocall)

        self._logger.info("Wait for call duration: " + str(self._callduration) + "s...")
        time.sleep(self._callduration)

        self._voicecall_api.check_state(self._uecmd_types.VOICE_CALL_STATE.ACTIVE)  # pylint: disable=E1101

        # Release the call
        self._voicecall_api.release()

        # Check the DUT is camped back on the LTE cell before timeout
        camp_on_lte = self._modem_api.check_network_type_before_timeout("LTE",
                                                                        self._registration_timeout)
        if camp_on_lte == False:
            return Global.FAILURE, "DUT didn't camp back on the LTE cell after the call was released"

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        Disposes this test.
        """
        UseCaseBase.tear_down(self)
        # Release the call
        self._voicecall_api.release()

        # Set the preferred NW type to "4G preferred"
        self._networking_api.set_preferred_network_type(self._initial_pref_network)
        time.sleep(self._wait_btwn_cmd)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def __force_to_camp_on_lte(self):
        """
        Force the device to camp on a LTE cell

        :raise DeviceException: if DUT is not camped on a LTE cell
        """

        # Set the preferred NW type to "4G only"
        self._networking_api.set_preferred_network_type("4G_ONLY")
        time.sleep(self._wait_btwn_cmd)

        # Check the DUT is camped on a LTE cell before timeout
        camp_on_lte = self._modem_api.check_network_type_before_timeout("LTE",
                                                                        self._registration_timeout)
        if camp_on_lte == False:
            err_msg = "The DUT couldn't camp on a LTE cell when preferred NW type was set to '4G_ONLY'"
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, err_msg)

        # Set the preferred NW type to 4G_PREF in order to be able to perform CSFB
        self._networking_api.set_preferred_network_type("4G_PREF")
        time.sleep(self._wait_btwn_cmd)

        # Check the DUT is camped on a LTE cell before timeout
        camp_on_lte = self._modem_api.check_network_type_before_timeout("LTE",
                                                                        self._registration_timeout)
        if camp_on_lte == False:
            err_msg = "The DUT is camped on another RAT after setting the preferred NW type to '4G_PREF'"
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, err_msg)
