"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

property right is granted to or conferred upon you by disclosure or delivery
of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL MCG PSI
:summary: This file implements the AudioComms VOIP call (Check volume
quality).
:since: 13/02/2014
:author: nprecigx
"""

import time
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.UseCase.Audio.LAB_AUDIO_QUALITY_BASE import LabAudioQualityBase
from ErrorHandling.AcsBaseException import AcsBaseException

# pylint: disable=E1101


class LabAudioVolumeVOIP(LabAudioQualityBase):

    """
    AudioComms Audio CSV Call class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LabAudioQualityBase Init function
        LabAudioQualityBase.__init__(self, tc_name, global_config)

        # Instantiate generic UECmd for voiceCall Ucs
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self._keyevent_api = self._device.get_uecmd("KeyEvent")
        self._networking_api = self._device.get_uecmd("Networking")

        self._voice_call_api2 = None

        # Read wifi parameters from bench config xml file (int)
        self._wifirouter = global_config.benchConfig.\
            get_parameters("WIFI_ACCESS_POINT")
        self._ssid = self._wifirouter.get_param_value("SSID")

        # Read dut sip address from test case xml file (string)
        self._dut_sip_address = \
            str(self._tc_parameters.get_param_value("DUT_SIP_ADDRESS"))

        # Read ref phone sip address from test case xml file (string)
        self._peer_sip_address = \
            str(self._tc_parameters.get_param_value("PEER_SIP_ADDRESS"))
        # Instantiate generic UECmd for voiceCall Ucs
        self._networking_api = self._device.get_uecmd("Networking")
        self._sip_call_api = self._device.get_uecmd("SipCall")

        # Instantiate the instances of member class variables
        self._sip_call_api2 = None
        self._networking_api2 = None
        self._phone_calling = None
        self._phone_receiving = None
        self._phone_releasing = None
        self._calling_phone_number = None
        self._delta_volume = 0

    def __compute_test_verdict__(self, audio_analyzer_result):
        """
        Compute the result verdict and the verdict comment

        @type audio_analyzer_result: int
        @param audio_analyzer_result: the Audio Analyzer return code return by run command
        """

        if audio_analyzer_result == 0:
            self._result_verdict = Global.SUCCESS
            self._verdict_comment = ("Audio Volume test success: with "
                                     "a %s call and %s accessories and %s db"
                                     % (self._call_type,
                                        self._acc_type,
                                        self._delta_volume))
        elif audio_analyzer_result == 1:
            self._result_verdict = Global.FAILURE
            self._verdict_comment = ("Audio Volume test fail: with "
                                     "a %s call and %s accessories and %s db"
                                     % (self._call_type,
                                        self._acc_type,
                                        self._delta_volume))
        elif audio_analyzer_result == 3:
            self._result_verdict = Global.BLOCKED
            self._verdict_comment = "Audio Volume test fail: Audio Analyzer executable exception occurred"
        else:
            self._result_verdict = Global.BLOCKED
            self._verdict_comment = ("Audio Volume test fail: Audio Analyzer problems detected "
                                     "with a %s call and %s accessories"
                                     % (self._call_type,
                                        self._acc_type))

    def set_up(self):
        """
        Set up the test configuration
        """
        # Call LabAudioQualityBase Init function
        LabAudioQualityBase.set_up(self)

        if self._phone2 is not None:
            self._networking_api2 = self._phone2.get_uecmd("Networking")
            self._sip_call_api2 = self._phone2.get_uecmd("SipCall")
        else:
            self._sip_call_api2 = None
            self._networking_api2 = None

        # Setup phone type ( calling, receiving, releasing phone)
        if self._call_origin_type == "MO":
            self._phone_calling = self._sip_call_api
            self._phone_receiving = self._sip_call_api2
            self._calling_phone_number = self._peer_sip_address
        elif self._call_origin_type == "MT":
            self._phone_calling = self._sip_call_api2
            self._phone_receiving = self._sip_call_api
            self._calling_phone_number = self._dut_sip_address
        else:
            error_msg = \
                "This test case requires call originated type to be executed !"
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        if self._call_end_type == "NR":
            self._phone_releasing = self._sip_call_api2
        elif self._call_end_type == "MR":
            self._phone_releasing = self._sip_call_api
        else:
            error_msg = \
                "This test case requires call end type to be executed !"
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        if self._networking_api.get_wifi_power_status() == 0:
            # Turn Wifi interface ON
            self._networking_api.set_wifi_power("off")
            self._networking_api.set_wifi_power("on")
            # Configure the DUT
            self._networking_api.clean_all_data_connections()
            time.sleep(self._wait_btwn_cmd)
            # Connect to wifi using SSID parameter value for DUT
            self._networking_api.wifi_connect(self._ssid)
            time.sleep(self._wait_btwn_cmd)

        if self._networking_api2.get_wifi_power_status() == 0:
            self._networking_api2.set_wifi_power("off")
            self._networking_api2.set_wifi_power("on")
            self._networking_api2.clean_all_data_connections()
            time.sleep(self._wait_btwn_cmd)
            # Connect to wifi using SSID parameter value for Ref phone
            self._networking_api2.wifi_connect(self._ssid)
            time.sleep(self._wait_btwn_cmd)

        # Start Sip Service
        self._phone_calling.initialize_sipcall_module()
        self._phone_receiving.initialize_sipcall_module()

        # Setup Sip Account
        self._sip_call_api.initialize_local_profile(self._dut_sip_address)
        self._sip_call_api2.initialize_local_profile(self._peer_sip_address)

        self._phone_calling.wait_for_state(
            self._uecmd_types.SIP_CALL_STATE.READY_TO_CALL,
            self._call_setup_time)
        self._phone_receiving.wait_for_state(
            self._uecmd_types.SIP_CALL_STATE.READY_TO_CALL,
            self._call_setup_time)

        # Dial using PHONE_NUMBER parameter
        self._phone_calling.dial(self._calling_phone_number)

        self._phone_receiving.wait_for_state(
            self._uecmd_types.SIP_CALL_STATE.INCOMING_CALL,
            self._call_setup_time)

        self._phone_receiving.answer()
        # Phone1 & 2 : Check voice call is active
        self._phone_calling.wait_for_state(
            self._uecmd_types.SIP_CALL_STATE.IN_CALL,
            self._call_setup_time)
        self._phone_receiving.wait_for_state(
            self._uecmd_types.SIP_CALL_STATE.IN_CALL,
            self._call_setup_time)

        # Configure Audio output to acc_type given with the test_case
        if self._acc_type == "EARPIECE":
            self._sip_call_api.switch_to_earpiece()
            if self._use_io_card:
                self._wired_headset.unplug_whs()
                self._wired_headset.unplug_headphone()
            time.sleep(self._wait_btwn_cmd)
        elif self._acc_type == "SPEAKER":
            self._sip_call_api.switch_to_speaker()
            time.sleep(self._wait_btwn_cmd)
        elif self._acc_type == "HEADSET":
            self._sip_call_api.switch_to_earpiece()
            if self._use_io_card:
                self._wired_headset.unplug_headphone()
                self._wired_headset.plug_whs()
        elif self._acc_type == "HEADPHONE":
            if self._use_io_card:
                self._sip_call_api.switch_to_earpiece()
                self._wired_headset.unplug_whs()
                self._wired_headset.plug_headphone()
        elif self._acc_type == "BLUETOOTHHSP" or self._acc_type == "BLUETOOTHHFP":
            self._sip_call_api.switch_to_bluetooth()
        elif self._acc_type == "BLUETOOTHA2DP":
            self._sip_call_api.switch_to_bluetooth_a2dp()
        else:
            self._logger.error("Unknown accessories")
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED,
                                     "Unknown accessories")
        time.sleep(self._wait_btwn_cmd)

        self._system_api2.adjust_specified_stream_volume("VoiceCall",
                                                         self._call_stream_volume_ref)

        return Global.SUCCESS, "No errors"

    def run_test(self):
        """
        Execute the test
        """
        # Call LabAudioQualityBase run_test function
        LabAudioQualityBase.run_test(self)



        # WAIT FOR CALL DURATION
        self._logger.info(
            "Wait for call duration: %s s..." % str(self._call_duration))

        # Set Voice Call Volume 0%
        self._system_api.adjust_specified_stream_volume("VoiceCall", 0)
        # Launch audio_quality test
        audio_analyzer_result, l_volume_zero_percent = self._audio_analyzer.get_volume(self._call_type, self._acc_type)
        self._logger.info(
            "Volume at 0 percent: %s " % str(l_volume_zero_percent))

        # Set Voice Call Volume 100%
        self._system_api.adjust_specified_stream_volume("VoiceCall", 100)
        # Launch audio_quality test
        audio_analyzer_result2, l_volume_hundred_percent = self._audio_analyzer.get_volume(self._call_type, self._acc_type)
        self._logger.info(
            "Volume at 100 percent: %s " % str(l_volume_hundred_percent))

        self._keyevent_api.scenario(["POWER_BUTTON"])
        # Set Voice Call Volume 0%
        self._system_api.adjust_specified_stream_volume("VoiceCall", 0)
        # Launch audio_quality test
        audio_analyzer_result3, l_volume_zero_percent2 = self._audio_analyzer.get_volume(self._call_type, self._acc_type)
        self._logger.info(
            "Volume at 0 percent after press power down button: %s "
            % str(l_volume_zero_percent2))

        if audio_analyzer_result == 0 and audio_analyzer_result2 == 0 and audio_analyzer_result3 == 0:
            l_delta1 = int(l_volume_hundred_percent) - int(l_volume_zero_percent)
            l_delta2 = int(l_volume_hundred_percent) - int(l_volume_zero_percent2)
            self._delta_volume = abs(l_delta1 - l_delta2)

            if self._delta_volume < 10:
                self._result_verdict = 0
            else:
                self._result_verdict = 1
        else:
            self._result_verdict = 3
        # Compute test verdict and comment verdict
        self.__compute_test_verdict__(self._result_verdict)

        return self._result_verdict, self._verdict_comment

    def tear_down(self):
        """
        End and dispose the test
        """
        try:
            # RELEASE THE CALL
            # Phone1 & 2 : Check call is still active
            self._phone_calling.wait_for_state(
                self._uecmd_types.SIP_CALL_STATE.IN_CALL, self._call_setup_time)

            if self._phone_receiving:
                self._phone_receiving.wait_for_state(
                    self._uecmd_types.SIP_CALL_STATE.IN_CALL, self._call_setup_time)

            self._phone_releasing.release()

            self._phone_releasing.wait_for_state(
                self._uecmd_types.SIP_CALL_STATE.READY_TO_CALL,
                self._call_setup_time)

            if self._phone_receiving:
                self._phone_receiving.wait_for_state(
                    self._uecmd_types.SIP_CALL_STATE.READY_TO_CALL,
                    self._call_setup_time)

        except AcsBaseException as acs_exception:
            self._logger.warning("Call release fail:" + str(acs_exception))

        # Call LabAudioQualityBase tear_down function
        LabAudioQualityBase.tear_down(self)

        if self._acc_type == "HEADSET":
            if self._use_io_card:
                self._wired_headset.unplug_whs()
        elif self._acc_type == "HEADPHONE":
            if self._use_io_card:
                self._wired_headset.unplug_headphone()

        # Stop Sip Service
        if self._sip_call_api is not None:
            self._sip_call_api.clean_sipcall_module()
        if self._sip_call_api2 is not None:
            self._sip_call_api2.clean_sipcall_module()

        # Disable Wifi connections
        if self._result_verdict != Global.SUCCESS:
            if self._networking_api is not None:
                self._networking_api.clean_all_data_connections()
                time.sleep(self._wait_btwn_cmd)
                self._networking_api.set_wifi_power("off")
            if self._networking_api2 is not None:
                self._networking_api2.clean_all_data_connections()
                time.sleep(self._wait_btwn_cmd)
                self._networking_api2.set_wifi_power("off")

        return Global.SUCCESS, "No errors"
