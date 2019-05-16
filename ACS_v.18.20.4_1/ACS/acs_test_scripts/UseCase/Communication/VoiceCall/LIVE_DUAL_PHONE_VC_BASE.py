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
:summary: This file implements the LIVE VC Base.
The use case need two phones to perform calls from DUT to reference phone
:since: 07/11/2011
:author: ssavrimoutou
"""

import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from Device.DeviceManager import DeviceManager
from ErrorHandling.AcsConfigException import AcsConfigException


class LiveDualPhoneVcBase(UseCaseBase):

    """
    Live Voice Call base class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call init of use case base
        UseCaseBase.__init__(self, tc_name, global_config)

        # Get Test Cases Parameters
        self._callduration = \
            int(self._tc_parameters.get_param_value("CALL_DURATION"))

        # Read callSetupTimeout from Phone_Catalog.xml
        self._call_setup_time = \
            int(self._dut_config.get("callSetupTimeout"))

        # Get UECmdLayer
        self._voice_call_api = self._device.get_uecmd("VoiceCall")
        self._networking_api = self._device.get_uecmd("Networking")

        # Load instance of the PHONE2
        self._phone2 = DeviceManager().get_device("PHONE2")

        if self._phone2 is not None:
            self._voice_call_api2 = self._phone2.get_uecmd("VoiceCall")
            self._networking_api2 = self._phone2.get_uecmd("Networking")

        else:
            self._voice_call_api2 = None
            self._networking_api2 = None

        # Instantiate the instances for phone caller, receiver and releaser
        self._caller_phone = None
        self._receiver_phone = None
        self._releaser_phone = None

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call set_up of use case base
        UseCaseBase.set_up(self)

        # Check if we have the second phone available
        if self._phone2 is None:
            # We are using this multi UC with only one phone
            error_msg = \
                "This use case requires two phones to be executed !"
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, error_msg)

        # Boot the other phone (the DUT is already booted)
        if not self._phone2.is_available():
            DeviceManager().boot_device("PHONE2")

        # Disable flight mode
        self._networking_api.set_flight_mode("off")
        self._networking_api2.set_flight_mode("off")

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # pylint: disable-msg=E1101
        # This to avoid pylint warnings due to enum class VOICE_CALL_STATE

        # Call init of use case base
        UseCaseBase.run_test(self)

        # Release any previous call (Robustness)
        time.sleep(self._wait_btwn_cmd)
        self._voice_call_api.release()
        self._voice_call_api2.release()

        # ESTABLISH THE CALL
        # Phone1 : Dial
        time.sleep(self._wait_btwn_cmd)
        self._caller_phone.dial(self._phone_number, False)

        # Phone2 : Wait for state incoming before callSetupTimeout seconds
        self._receiver_phone.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.INCOMING,
            self._call_setup_time)

        # Phone2 : Answer call
        self._receiver_phone.answer()

        # Phone1 & 2 : Check voice call is active
        self._voice_call_api.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
            self._call_setup_time)
        self._voice_call_api2.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
            self._call_setup_time)

        # WAIT FOR CALL DURATION
        self._logger.info(
            "Wait for call duration: " + str(self._callduration) + "s...")
        time.sleep(self._callduration)

        # RELEASE THE CALL
        # Phone1 & 2 : Check call is still active
        self._voice_call_api.check_state(
            self._uecmd_types.VOICE_CALL_STATE.ACTIVE)
        self._voice_call_api2.check_state(
            self._uecmd_types.VOICE_CALL_STATE.ACTIVE)

        # Phone1 : Release the call
        time.sleep(self._wait_btwn_cmd)
        self._releaser_phone.release()

        # Phone1 & 2 : Check call is idle
        self._voice_call_api.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.NOCALL,
            self._call_setup_time)
        self._voice_call_api2.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.NOCALL,
            self._call_setup_time)

        # pylint: enable=E1101
        return Global.SUCCESS, "No errors"
