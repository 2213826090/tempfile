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
quality) with accessories change during call.
:since: 27/02/2013
:author: nprecigx
"""

import time
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.UseCase.Audio.LAB_AUDIO_QUALITY_ACCESSORIES_CHANGE_BASE \
    import LabAudioQualityAccessoriesChangeBase

# pylint: disable=E1101


class LabAudioGlitchDetectionAccessoriesChangeVOIP(LabAudioQualityAccessoriesChangeBase):

    """
    AudioComms Audio Glitch Detection VOIP Call Accessories Change class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LabAudioQualityAccessoriesChangeBase Init function
        LabAudioQualityAccessoriesChangeBase.__init__(self, tc_name, global_config)

        # Initialization of the test type
        self._test_type = "glitchdetectiononswitch"

        # Read dut sip address from test case xml file (str)
        self._dut_sip_address = \
            str(self._tc_parameters.get_param_value("DUT_SIP_ADDRESS"))

        # Read ref phone sip address from test case xml file (str)
        self._peer_sip_address = \
            str(self._tc_parameters.get_param_value("PEER_SIP_ADDRESS"))

        # Read wifi parameters from bench config xml file (int)
        self._wifirouter = global_config.benchConfig.\
            get_parameters("WIFI_ACCESS_POINT")
        self._ssid = self._wifirouter.get_param_value("SSID")

        # Instantiate generic UECmd for voiceCall Ucs
        self._networking_api = self._device.get_uecmd("Networking")
        self._sip_call_api = self._device.get_uecmd("SipCall")

        self._sip_call_api2 = None
        self._networking_api2 = None
        # Instantiate the instances for phone caller, receiver and releaser
        self._phone_calling = None
        self._phone_receiving = None
        self._phone_releasing = None
        # Initialization of phone_number to call
        self._calling_phone_number = None

        # Copy of accessories list
        self._temp_acc_list_splited = None

    def set_up(self):
        """
        Set up the test configuration
        """

        # Call LabAudioQualityAccessoriesChangeBase Setup function
        LabAudioQualityAccessoriesChangeBase.set_up(self)

        if self._phone2 is not None:
            self._networking_api2 = self._phone2.get_uecmd("Networking")
            self._sip_call_api2 = self._phone2.get_uecmd("SipCall")
        else:
            self._sip_call_api2 = None
            self._networking_api2 = None

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
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     error_msg)

        if self._call_end_type == "NR":
            self._phone_releasing = self._sip_call_api2
        elif self._call_end_type == "MR":
            self._phone_releasing = self._sip_call_api
        else:
            error_msg = \
                "This test case requires call end type to be executed !"
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     error_msg)

        # Start Sip Service
        self._phone_calling.initialize_sipcall_module()
        self._phone_receiving.initialize_sipcall_module()

        # Setup Sip Account
        self._sip_call_api.initialize_local_profile(self._dut_sip_address)
        self._sip_call_api2.initialize_local_profile(self._peer_sip_address)

        return Global.SUCCESS, "No errors"

    def run_test(self):
        """
        Execute the test
        """
        # Call LabAudioQualityAccessoriesChangeBase run_test function
        LabAudioQualityAccessoriesChangeBase.run_test(self)

        if self._acc_type == "HEADSET":
            if self._use_io_card:
                self._wired_headset.unplug_headphone()
                self._wired_headset.plug_whs()
        elif self._acc_type == "HEADPHONE":
            if self._use_io_card:
                self._wired_headset.unplug_whs()
                self._wired_headset.plug_headphone()

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

        # Save accessories list in temporary copy
        self._temp_acc_list_splited = self._acc_list_split
        # Change Accessories
        while len(self._temp_acc_list_splited) > 0:

            # Set Stream Voice Call Volume
            l_volume_dut = self._call_volume_dut_list_split.pop()
            l_volume_ref = self._call_volume_ref_list_split.pop()
            l_acc_active = int(self._acc_active_list_split.pop())
            self._previous_acc_type = self._acc_type
            self._acc_type = self._temp_acc_list_splited.pop()
            self._logger.info(
                "Accessories transition: " + self._previous_acc_type + " => " + self._acc_type)

            if l_acc_active == 1:
                # Need to test accessories
                # Launch audio_quality test
                l_audio_analyzer_ready, l_audio_analyzer_process = self._audio_analyzer.glitch_detection_before_switch(
                    self._test_type,
                    self._device.get_phone_model(),
                    self._call_type,
                    self._acc_type,
                    self._signal_tested_direction)

                #Ready for switch
                if l_audio_analyzer_ready == 0:
                    if self._acc_type.find("BLUETOOTH") == -1:
                        # Set Voice Call Volume
                        self._system_api.adjust_specified_stream_volume("VoiceCall",
                                                                        int(l_volume_dut))
                    else:
                        # Set Voice Call Volume
                        self._system_api.adjust_specified_stream_volume("Bluetooth",
                                                                        int(l_volume_dut))

                    self._system_api2.adjust_specified_stream_volume("VoiceCall",
                                                                     int(l_volume_ref))

                    # Configure Audio output to acc_type given with the test_case
                    if "EARPIECE" in self._acc_type:
                        self._sip_call_api.switch_to_earpiece()
                        time.sleep(self._wait_btwn_cmd)
                    elif "SPEAKER" in self._acc_type:
                        self._sip_call_api.switch_to_speaker()
                        time.sleep(self._wait_btwn_cmd)
                    elif self._acc_type == "HEADSET":
                        self._sip_call_api.switch_to_earpiece()
                        time.sleep(self._wait_btwn_cmd)
                        if self._use_io_card:
                            self._wired_headset.unplug_headphone()
                            self._wired_headset.plug_whs()
                    elif self._acc_type == "HEADPHONE":
                        self._sip_call_api.switch_to_earpiece()
                        time.sleep(self._wait_btwn_cmd)
                        if self._use_io_card:
                            self._wired_headset.unplug_whs()
                            self._wired_headset.plug_headphone()
                    elif "BLUETOOTHHSP" in self._acc_type:
                        self._sip_call_api.switch_to_bluetooth()
                    elif "BLUETOOTHHFP" in self._acc_type:
                        self._sip_call_api.switch_to_bluetooth()
                    elif "BLUETOOTHA2DP" in self._acc_type:
                        self._sip_call_api.switch_to_bluetooth_a2dp()
                    else:
                        self._logger.error("Unknown accessories => Change accessories failed")
                    time.sleep(self._wait_btwn_cmd)

                    l_audio_analyzer_result = self._audio_analyzer.glitch_detection_after_switch(
                        l_audio_analyzer_process)

                    # Compute test verdict
                    self._compute_test_verdict_(l_audio_analyzer_result, 0)
            else:
                self._logger.info("accessory: " + self._acc_type + " not active => not tested")

        # RELEASE THE CALL
        # Phone1 & 2 : Check call is still active
        self._phone_calling.wait_for_state(
            self._uecmd_types.SIP_CALL_STATE.IN_CALL, self._call_setup_time)
        self._phone_receiving.wait_for_state(
            self._uecmd_types.SIP_CALL_STATE.IN_CALL, self._call_setup_time)

        self._phone_releasing.release()

        self._phone_releasing.wait_for_state(
            self._uecmd_types.SIP_CALL_STATE.READY_TO_CALL,
            self._call_setup_time)
        self._phone_receiving.wait_for_state(
            self._uecmd_types.SIP_CALL_STATE.READY_TO_CALL,
            self._call_setup_time)

        return self._result_verdict, self._verdict_comment

    def tear_down(self):
        """
        End and dispose the test
        """

        # Call LabAudioQualityAccessoriesChangeBase tear_down function
        LabAudioQualityAccessoriesChangeBase.tear_down(self)

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
