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
:summary: This file implements the MT VC UC
:since: 21/06/2011
:author: vtinelli
"""

import time
from UtilitiesFWK.Utilities import Global

from SYSTEM_SLEEP_BASE import SystemSleepBase


class LabSystemSleepMtVcWake(SystemSleepBase):

    """
    S0i3 MT VC class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        SystemSleepBase.__init__(self, tc_name, global_config)

        # Instantiate the network simulator
        self._ns = self._em.get_cellular_network_simulator("NETWORK_SIMULATOR1")
        self._ns_2g = self._ns.get_cell_2g()

        # VC and USB plug sleep to avoid USB wake-up
        self._vc_usb_plug_timer = int(self._tc_parameters.get_param_value("VC_USB_TIMER"))

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

        # It's time to sleep !
        time.sleep(self._wait_btwn_cmd)
        self._phonesystem_api.sleep_mode("on")

        # Get initial wakeup count
        wake_matched_before_value = \
            self._phonesystem_api.get_sleep_wakeup_count(self._sleep_mode)

        # Unplug usb
        self._device.disconnect_board()
        self._io_card.usb_host_pc_connector(False)
        # unplug wall charger only if it is AC_CHGR
        if self._device.get_default_wall_charger() == self._io_card.AC_CHGR:
            # Unplug wall charger
            self._io_card.wall_charger_connector(False)

        self._logger.info("Wait for %s s to enter in %s before voice call" % (str(self._duration), self._sleep_mode))
        time.sleep(self._duration)

        # Mobile Terminated originate call
        self._voice_call_2g.mt_originate_call()

        # Wait before USB plug
        time.sleep(self._vc_usb_plug_timer)
        # plug wall charger only if it is AC_CHGR
        if self._device.get_default_wall_charger() == self._io_card.AC_CHGR:
            # Plug wall charger
            self._io_card.wall_charger_connector(True)
        # Plug USB to answer call
        self._io_card.usb_host_pc_connector(True)
        self._device.connect_board()

        # Get the wake up source
        wakeup_source = self._phonesystem_api.get_wakeup_source()
        if wakeup_source == "HSI":
            return_code = Global.SUCCESS
        else:
            return_code = Global.FAILURE

        return_msg = "Device wakes up from %s" % wakeup_source

        # Wait for state "incoming" before callSetupTimeout seconds
        self._voicecall_api.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.INCOMING,  # pylint: disable=E1101
            self._call_setup_time)

        # Mobile Release call
        self._voicecall_api.release()

        # Retrieve wakeup count in sleep mode
        wake_matched_after_value = self._phonesystem_api.get_sleep_wakeup_count(self._sleep_mode)
        wake_diff = wake_matched_after_value - wake_matched_before_value
        return_msg += " - %s wake-up %d" % (self._sleep_mode, wake_diff)

        return return_code, return_msg
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
