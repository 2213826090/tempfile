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
:summary: This file implements the MT VC UC to measure residency
:since: 20/03/2012
:author: ssavrimoutou
"""

import time
from UtilitiesFWK.Utilities import Global
from SYSTEM_SLEEP_BASE import SystemSleepBase


class LabSystemSleepMtVcResidencyMeasurement(SystemSleepBase):

    """
    Sleep mode MT VC residency measurement class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        SystemSleepBase.__init__(self, tc_name, global_config)

        # Instantiate the network simulator
        self._ns = self._em.get_cellular_network_simulator("NETWORK_SIMULATOR1")
        self._ns_2g = self._ns.get_cell_2g()

        # VC call duration
        self._call_duration = int(self._tc_parameters.get_param_value("VC_CALL_DURATION"))

        # MT CALL ringing duration (NS parameter)
        self._vc_ringing_timer = int(self._tc_parameters.get_param_value("VC_RINGING_TIMER"))

        # Instantiate generic UECmd for voiceCall Ucs
        self._modem_api = self._device.get_uecmd("Modem")
        self._voicecall_api = self._device.get_uecmd("VoiceCall")

        # Load 2G voice call driver
        self._voice_call_2g = self._ns.get_cell_2g().get_voice_call()

        # Read registrationTimeout from Device_Catalog.xml
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

        # Read callSetupTimeout from Device_Catalog.xml
        self._call_setup_time = \
            int(self._dut_config.get("callSetupTimeout"))

    def set_up(self):
        """
        Set up the test configuration
        """

        SystemSleepBase.set_up(self)

        # Connect to equipment using GPIBAddress and GPIBBoardId
        self._ns.init()

        # Set the equipment application format GSM/GPRS
        self._ns.switch_app_format("GSM/GPRS")

        # Perform a full preset
        self._ns.perform_full_preset()

        # Set cell off
        self._ns_2g.set_cell_on()

        # Configure T301 Call originate timer
        self._voice_call_2g.set_mt_originate_call_timeout(self._vc_ringing_timer)

        # Check registration status before registrationTimeout (CDK)
        time.sleep(self._wait_btwn_cmd)
        self._modem_api.check_cdk_registration_bfor_timeout(self._registration_timeout)

        return Global.SUCCESS, "No errors"

    def run_test(self):
        """
        Execute the test
        """

        SystemSleepBase.run_test(self)

        # Release any previous call (Robustness)
        time.sleep(self._wait_btwn_cmd)
        self._voicecall_api.release()

        # Mobile Terminated originate call
        self._voice_call_2g.mt_originate_call()

        # Wait for state "incoming" before callSetupTimeout seconds
        self._voicecall_api.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.INCOMING,  # pylint: disable=E1101
            self._call_setup_time)

        # Answer call
        self._voicecall_api.answer()

        # Check call state "CONNECTED" before callSetupTimeout seconds
        self._voice_call_2g.check_call_connected(self._call_setup_time)

        # Wait for state "active" before callSetupTimeout seconds
        self._voicecall_api.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.ACTIVE,  # pylint: disable=E1101
            self._call_setup_time)

        # Unplug usb
        self._device.disconnect_board()
        # Reset residency value
        self._residency_api.clear()

        self._io_card.usb_host_pc_connector(False)
        # unplug wall charger only if it is AC_CHGR
        if self._device.get_default_wall_charger() == self._io_card.AC_CHGR:
            # Unplug wall charger
            self._io_card.wall_charger_connector(False)

        # Check call is connected for CALL_DURATION seconds
        self._logger.info("Waiting %s s to enter in %s during voice call" %
                          (str(self._call_duration), self._sleep_mode))

        self._voice_call_2g.is_voice_call_connected(self._call_duration)
        # plug wall charger only if it is AC_CHGR
        if self._device.get_default_wall_charger() == self._io_card.AC_CHGR:
            # Plug wall charger
            self._io_card.wall_charger_connector(True)
        # Plug USB to release call
        self._io_card.usb_host_pc_connector(True)
        time.sleep(self._wait_btwn_cmd)
        residency_spent = self._residency_api.get_value("residency",
                                                        self._sleep_mode_api.get_sleep_mode())

        self._device.connect_board()
        return self._residency_verdict(residency_spent)

#------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """

        # Call power measurement base tear_down function
        SystemSleepBase.tear_down(self)

        # Set cell off
        self._ns_2g.set_cell_off()

        # DisConnect from equipment
        self._ns.release()

        return Global.SUCCESS, "No errors"
