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

property right is granted to or conferred upon you by disclosure or delivery
of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL MCG PSI
:summary: This file implements the AudioComms VOIP call (Check audio
quality after mute call).
:since: 12/05/2014
:author: nprecigx
"""

import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.Audio.LAB_AUDIO_QUALITY_BASE import LabAudioQualityBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.AcsBaseException import AcsBaseException
# pylint: disable=E1101


class LabAudioQualityVOIPMute(LabAudioQualityBase):

    """
    AudioComms Audio VOIP Mute Call class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LabAudioQualityBase Init function
        LabAudioQualityBase.__init__(self, tc_name, global_config)

        # Read wifi parameters from bench config xml file (int)
        self._wifirouter = global_config.benchConfig. \
            get_parameters("WIFI_ACCESS_POINT")
        self._wifi_security = self._wifirouter.get_param_value("WIFI_SECURITY")
        self._wifi_passphrase = self._wifirouter.get_param_value("passphrase")
        self._ssid = self._wifirouter.get_param_value("SSID")

        # Read dut sip address from test case xml file (str)
        self._dut_sip_address = \
            str(self._tc_parameters.get_param_value("DUT_SIP_ADDRESS"))

        # Read ref phone sip address from test case xml file (str)
        self._peer_sip_address = \
            str(self._tc_parameters.get_param_value("PEER_SIP_ADDRESS"))

        # Read hold or mute parameter (str)
        self._hold_mute = \
            str(self._tc_parameters.get_param_value("HOLD_MUTE"))

        self._dut_sip_passphrase = None
        self._peer_sip_passphrase = None
        # Instantiate generic UECmd for voiceCall Ucs
        self._networking_api = self._device.get_uecmd("Networking")
        self._sip_call_api = self._device.get_uecmd("SipCall")

        # Instantiate the instances of member class variables
        self._sip_call_api2 = None
        self._networking_api2 = None
        self._phone_calling = None
        self._phone_receiving = None
        self._phone_releasing = None
        # Initialization of phone_number to call
        self._calling_phone_number = None

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

        # Setup the wifi connection for DUT and Ref phone
        self.__setup_wifi()

        # Connect to wifi using SSID parameter value for DUT
        self._networking_api.wifi_connect(self._ssid)
        time.sleep(self._wait_btwn_cmd)

        # Connect to wifi using SSID parameter value for Ref phone
        if self._phone2 is not None:
            self._networking_api2.wifi_connect(self._ssid)
            time.sleep(self._wait_btwn_cmd)

        # Start Sip Service
        self._phone_calling.initialize_sipcall_module()

        # In case only 1 phone is used, no need to set a 2nd SIP account (assuming MO call)
        if self._phone_receiving:
            self._phone_receiving.initialize_sipcall_module()

        # Setup Sip Account
        self._sip_call_api.initialize_local_profile(self._dut_sip_address, self._dut_sip_passphrase)
        if self._phone2 is not None:
            self._sip_call_api2.initialize_local_profile(self._peer_sip_address, self._peer_sip_passphrase)

        self._phone_calling.wait_for_state(
            self._uecmd_types.SIP_CALL_STATE.READY_TO_CALL,
            self._call_setup_time)

        # In case only 1 phone is used, no need to make verifications on reference phone (assuming MO call)
        if self._phone_receiving:
            self._phone_receiving.wait_for_state(
                self._uecmd_types.SIP_CALL_STATE.READY_TO_CALL,
                self._call_setup_time)

        # Dial using PHONE_NUMBER parameter
        if self._phone2 is not None:
            self._phone_calling.dial(self._calling_phone_number)
        else:
            self._phone_calling.dial(self._calling_phone_number, check_state=False)

        if self._phone_receiving:
            self._phone_receiving.wait_for_state(
                self._uecmd_types.SIP_CALL_STATE.INCOMING_CALL,
                self._call_setup_time)

            self._phone_receiving.answer()
            # Phone1 & 2 : Check voice call is active
        self._phone_calling.wait_for_state(
            self._uecmd_types.SIP_CALL_STATE.IN_CALL,
            self._call_setup_time)

        if self._phone_receiving:
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
            if self._use_io_card:
                self._sip_call_api.switch_to_earpiece()
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

        if self._acc_type.find("BLUETOOTH") == -1:
            # Set Voice Call Volume
            self._system_api.adjust_specified_stream_volume("VoiceCall",
                                                            self._call_stream_volume_dut)
        else:
            # Set Voice Call Volume
            self._system_api.adjust_specified_stream_volume("Bluetooth",
                                                            self._call_stream_volume_dut)

        if self._phone_receiving:
            self._system_api2.adjust_specified_stream_volume("VoiceCall",
                                                             self._call_stream_volume_ref)

        return Global.SUCCESS, "No errors"

    def run_test(self):
        """
        Execute the test
        """
        # Call LabAudioQualityBase run_test function
        LabAudioQualityBase.run_test(self)

        if self._hold_mute == "MUTE":
            self._sip_call_api.toogle_mute()

            # Check no audio on uplink after mute call
            audio_analyzer_result_ul = self._audio_analyzer.run(self._call_type, self._acc_type, "UL")
            audio_analyzer_result_dl = self._audio_analyzer.run(self._call_type, self._acc_type, "DL")

            #unmute call
            self._sip_call_api.toogle_mute()

            if audio_analyzer_result_ul == 2 and audio_analyzer_result_dl == 0:
                # Check audio on uplink is back after unmute
                audio_analyzer_result_ul = self._audio_analyzer.run(self._call_type, self._acc_type, "UL")
                audio_analyzer_result_dl = self._audio_analyzer.run(self._call_type, self._acc_type, "DL")
                if audio_analyzer_result_dl == 0 and audio_analyzer_result_ul == 0:
                    audio_analyzer_result = 0
                else:
                    audio_analyzer_result = 4
            else:
                audio_analyzer_result = 4

        elif self._hold_mute == "HOLD":
            self._sip_call_api.hold_call()

            # Check no audio on uplink after mute call
            audio_analyzer_result_ul = self._audio_analyzer.run(self._call_type, self._acc_type, "UL")
            audio_analyzer_result_dl = self._audio_analyzer.run(self._call_type, self._acc_type, "DL")

            #unhold call
            self._sip_call_api.unhold_call()

            if audio_analyzer_result_ul == 2 and audio_analyzer_result_dl == 2:
                # Check audio on uplink is back after unmute
                audio_analyzer_result_ul = self._audio_analyzer.run(self._call_type, self._acc_type, "UL")
                audio_analyzer_result_dl = self._audio_analyzer.run(self._call_type, self._acc_type, "DL")
                if audio_analyzer_result_dl == 0 and audio_analyzer_result_ul == 0:
                    audio_analyzer_result = 0
                else:
                    audio_analyzer_result = 4
            else:
                audio_analyzer_result = 4
        else:
            self._logger.error("Unknown hold/mute argument")
            audio_analyzer_result = 4

        # Compute test verdict and comment verdict
        self.__compute_test_verdict__(audio_analyzer_result)

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

            # Hang up call
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

    def __compute_test_verdict__(self, audio_analyzer_result):
        """
        Compute the result verdict and the verdict comment

        :type Audio audio_analyzer: int
        :param Audio audio_analyzer: the Audio Analyzer return code return by run command
        """

        if audio_analyzer_result == 0:
            self._result_verdict = Global.SUCCESS
            self._verdict_comment = ("Audio %s  call test success: with "
                                     "a %s call and %s accessories"
                                     % (self._hold_mute,
                                        self._call_type,
                                        self._acc_type))
        elif audio_analyzer_result == 1 or audio_analyzer_result == 2:
            self._result_verdict = Global.FAILURE
            self._verdict_comment = ("Audio %s call test fail: with "
                                     "a %s call and %s accessories"
                                     % (self._hold_mute,
                                        self._call_type,
                                        self._acc_type))
        elif audio_analyzer_result == 3:
            self._result_verdict = Global.BLOCKED
            self._verdict_comment = "Audio Mute call test fail: Audio analyzer executable exception occurred"
        elif audio_analyzer_result == 4:
            self._result_verdict = Global.FAILURE
            self._verdict_comment = ("Audio %s call test fail: with "
                                     "a %s call and %s accessories: Call not muted"
                                     % (self._hold_mute,
                                        self._call_type,
                                        self._acc_type))
        else:
            self._result_verdict = Global.BLOCKED
            self._verdict_comment = ("Audio %s call test fail: Audio analyzer problems detected "
                                     "with a %s call and %s accessories"
                                     % (self._hold_mute,
                                        self._call_type,
                                        self._acc_type))

    def __setup_wifi(self):
        """
        setup wifi connection but does not connect it.
        """
        if self._networking_api.get_wifi_power_status() == 0:
            # turn off wifi
            self._networking_api.set_wifi_power("off")
            # turn on wifi
            self._networking_api.set_wifi_power("on")

        # disconnect all data connections, including wifi
        self._networking_api.clean_all_data_connections()

        if self._wifi_security not in ("NONE", "OPEN"):
            self._networking_api. \
                set_wificonfiguration(self._ssid,
                                      self._wifi_passphrase,
                                      self._wifi_security)

        if self._phone2 is not None:
            if self._networking_api2.get_wifi_power_status() == 0:
                # turn off wifi
                self._networking_api2.set_wifi_power("off")
                # turn on wifi
                self._networking_api2.set_wifi_power("on")

            # disconnect all data connections, including wifi
            self._networking_api2.clean_all_data_connections()
            time.sleep(self._wait_btwn_cmd)

            if self._wifi_security not in ("NONE", "OPEN"):
                self._networking_api2. \
                    set_wificonfiguration(self._ssid,
                                          self._wifi_passphrase,
                                          self._wifi_security)
