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
:summary: This file implements the LIVE VC Ring Tone Vibra.
The use case need two phones to perform calls from reference phone to DUT
:since: 05/02/2014
:author: mmorchex
"""

import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from Device.DeviceManager import DeviceManager
from ErrorHandling.AcsConfigException import AcsConfigException


class LiveDualPhoneVcRingToneVibra(UseCaseBase):

    """
    Live Voice Call Ring Tone Vibra.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call init of use case base
        UseCaseBase.__init__(self, tc_name, global_config)

        # Get Test Cases Parameters
        self.__minimum_variation = float(self._tc_parameters.get_param_value("MINIMUM_VARIATION"))

        # Read callSetupTimeout from Phone_Catalog.xml
        self.__call_setup_time = int(self._dut_config.get("callSetupTimeout"))

        # Get UECmdLayer
        self.__voice_call_api = self._device.get_uecmd("VoiceCall")
        self.__networking_api = self._device.get_uecmd("Networking")
        self.__sensor_api = self._device.get_uecmd("Sensor")
        self.__system_api = self._device.get_uecmd("System")

        # Load instance of the PHONE2
        self.__phone2 = DeviceManager().get_device("PHONE2")

        if self.__phone2 is not None:
            self.__voice_call_api2 = self.__phone2.get_uecmd("VoiceCall")
            self.__networking_api2 = self.__phone2.get_uecmd("Networking")

        else:
            self.__voice_call_api2 = None
            self.__networking_api2 = None

        # Get phone1 number
        self.__phone_number = str(self._device.get_phone_number())

        # Instantiate the instances for phone caller, receiver and releaser
        self.__caller_phone = self.__voice_call_api2
        self.__receiver_phone = self.__voice_call_api
        self.__releaser_phone = self.__voice_call_api2

        self.__check_accelerometer_time = 5
        self.__wait_btw_accelerometer_check = 0.5

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call set_up of use case base
        UseCaseBase.set_up(self)

        if not self.__minimum_variation or self.__minimum_variation <= 0.5:
            self.__minimum_variation = 0.5

        # Check if we have the second phone available
        if self.__phone2 is None:
            # We are using this multi UC with only one phone
            error_msg = "This use case requires two phones to be executed !"
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, error_msg)

        # Boot the other phone (the DUT is already booted)
        if not self.__phone2.is_available():
            DeviceManager().boot_device("PHONE2")

        # Disable flight mode
        self.__networking_api.set_flight_mode("off")
        self.__networking_api2.set_flight_mode("off")

        # Phone1: set Ringtone volume to 0%, will be in vibration mode
        self.__system_api.adjust_specified_stream_volume('Ringtone', 0)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        verdict = Global.SUCCESS
        msg = "No errors"

        # Call init of use case base
        UseCaseBase.run_test(self)

        # Release any previous call (Robustness)
        time.sleep(self._wait_btwn_cmd)
        self.__voice_call_api.release()
        self.__voice_call_api2.release()

        # PHONE1: Check accelerometer return before the call
        output, x, y, z = self.__sensor_api.check_sensor_info("accelerometer", "data", "reserve")
        time.sleep(self._wait_btwn_cmd)
        initial_accelerometer_value = abs(x) + abs(y) + abs(z)

        # ESTABLISH THE CALL
        # Phone2 : Dial
        time.sleep(self._wait_btwn_cmd)
        self.__caller_phone.dial(self.__phone_number, False)

        # Phone1 : Wait for state incoming before callSetupTimeout seconds
        self.__receiver_phone.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.INCOMING,
            self.__call_setup_time)

        # PHONE1: Check accelerometer return during the call
        accelerometer_value_while_voice_call = 0
        for iteration in range(self.__check_accelerometer_time):
            output, x, y, z = self.__sensor_api.check_sensor_info("accelerometer", "data", "reserve")
            # Store the highest accelerometer value
            if accelerometer_value_while_voice_call < abs(x) + abs(y) + abs(z):
                accelerometer_value_while_voice_call = abs(x) + abs(y) + abs(z)
            time.sleep(self.__wait_btw_accelerometer_check)

        if accelerometer_value_while_voice_call < initial_accelerometer_value + self.__minimum_variation:
            verdict = Global.FAILURE
            msg = "Accelerometer value during voice call: %s, is not as expected, should be >= %s " \
                % (str(accelerometer_value_while_voice_call), \
                   str(initial_accelerometer_value + self.__minimum_variation))

        # RELEASE THE CALL
        # Phone2 : Release the call
        time.sleep(self._wait_btwn_cmd)
        self.__releaser_phone.release()

        # Phone1 & 2 : Check call is idle
        self.__voice_call_api.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.NOCALL,
            self.__call_setup_time)
        self.__voice_call_api2.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.NOCALL,
            self.__call_setup_time)

        return verdict, msg

    def tear_down(self):
        """
        tear down
        """
        UseCaseBase.tear_down(self)

        # Phone1: set Ringtone volume to 50%
        self.__system_api.adjust_specified_stream_volume('Ringtone', 50)

        return self._error.Code, "No errors"
