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
quality).
:since: 02/02/2015
:author: hgarantx
"""

import os
import time
from UtilitiesFWK.Utilities import Global
import acs_test_scripts.Utilities.AudioUtilities as AudioUtil
from acs_test_scripts.UseCase.Audio.LAB_AUDIO_ACTIVITY_BASE import LabAudioActivityBase
from ErrorHandling.AcsConfigException import AcsConfigException

# pylint: disable=E1101


class LabAudioActivityVOIPStress(LabAudioActivityBase):
    """
    AudioComms Audio VOIP Call class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LabAudioActivityBase Init function
        LabAudioActivityBase.__init__(self, tc_name, global_config)

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

        if self._audio_analyzer_node.get_param_value("Model") in "RS_UPV":
            # Get the current date time for degraded audio file name
            self._cur_date_time = time.strftime('%Y%m%d_%H%M%S', time.localtime())

            # Read ConfPath from AUDIO_ANALYZER BenchConfig.xml
            self._aa_conf_path = os.path.join(str(self._audio_analyzer_node.get_param_value("ConfPath")))

            # Read DestPath from AUDIO_ANALYZER BenchConfig.xml
            self._aa_dest_path = os.path.join(str(self._audio_analyzer_node.get_param_value("DestPath")))

            self._host_trace_file_path = os.path.join(self._aa_conf_path,
                                                      self._name.split('\\')[-1].split('-')[0] + "-" +
                                                      str(self._tc_parameters.get_ucase_name()) + "_" +
                                                      self._cur_date_time + ".trc")

            # Construct trace file path on audio analyzer
            self._aa_trace_file_path = os.path.join(self._aa_dest_path,
                                                    "sip_sweep_trace.trc")

            self._dynamic_threshold = float(self._tc_parameters.get_param_value("THRESHOLD"))

            self._dut_sip_passphrase = self._tc_parameters.get_param_value("DUT_SIP_PASSPHRASE")
            self._peer_sip_passphrase = self._tc_parameters.get_param_value("PEER_SIP_PASSPHRASE")

            self._keep_record = self._tc_parameters.get_param_value("KEEP_RECORD")

            # Number of iteration for the current test
            self._tc_iteration_count = self._tc_parameters.get_b2b_iteration()

            self._audioalgocontroller_api = self._device.get_uecmd("AudioAlgoController")

        else:
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
        self._calling_phone_number = None

    def set_up(self):
        """
        Set up the test configuration
        """
        # Call LabAudioActivityBase Init function
        LabAudioActivityBase.set_up(self)

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

        if self._audio_analyzer_node.get_param_value("Model") in "RS_UPV":
            self._audio_analyzer.load_configuration(self._test_type,
                                                    None,
                                                    None,
                                                    None,
                                                    "DL")

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

        return Global.SUCCESS, "No errors"

    def run_test(self):
        """
        Execute the test
        """
        # Call LabAudioActivityBase run_test function
        LabAudioActivityBase.run_test(self)

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

        # WAIT FOR CALL DURATION
        self._logger.info(
            "Wait for call duration: %s s..." % str(self._call_duration))
        time.sleep(self._call_duration)

        # STRESS WHS PLUG / UNPLUG
        if self._use_io_card:
            self._wired_headset.unplug_whs()
            time.sleep(2)
            self._wired_headset.plug_whs()
            time.sleep(2)
            self._wired_headset.unplug_whs()
            time.sleep(2)
            self._wired_headset.plug_whs()
            time.sleep(2)
            self._wired_headset.unplug_whs()
            time.sleep(2)
            self._wired_headset.plug_whs()
            time.sleep(2)
            self._wired_headset.unplug_whs()
            time.sleep(2)
            self._wired_headset.plug_whs()
            time.sleep(2)
            self._wired_headset.unplug_whs()
            time.sleep(2)
            self._wired_headset.plug_whs()
            time.sleep(2)
            self._wired_headset.unplug_whs()
            time.sleep(2)
            self._wired_headset.plug_whs()
            time.sleep(2)
        if self._audio_analyzer_node.get_param_value("Model") in "APx585":

            # Launch audio_quality test
            if self._signal_tested_direction in ["UL", "DL"]:
                audio_analyzer_result = self._audio_analyzer.run(self._call_type,
                                                                 self._acc_type,
                                                                 self._signal_tested_direction)

                # Compute test verdict and comment verdict
                LabAudioActivityBase.__compute_test_verdict__(self, audio_analyzer_result)

            else:
                # Test both UL and DL audio output
                audio_analyzer_result_ul = self._audio_analyzer.run(self._call_type, self._acc_type, "ul")

                audio_analyzer_result_dl = self._audio_analyzer.run(self._call_type, self._acc_type, "dl")

                # Compute test verdict and comment verdict for UL
                LabAudioActivityBase.__compute_test_verdict__(self, audio_analyzer_result_ul)

                if self._result_verdict is Global.SUCCESS:
                    tmp = self._verdict_comment

                    # Compute test verdict and comment verdict for DL
                    LabAudioActivityBase.__compute_test_verdict__(self, audio_analyzer_result_dl)

                    if self._result_verdict is Global.SUCCESS:
                        self._verdict_comment += tmp

        elif self._audio_analyzer_node.get_param_value("Model") in "RS_UPV":

            # Perform audio routing verification for DL path first
            self._audio_analyzer.start_single_measurement()

            # Wait for the measurement to end
            time.sleep(12)

            # Transfer the result of the measurement to ACS host
            self._audio_analyzer.store_data_list(self._aa_trace_file_path,
                                                 self._host_trace_file_path,
                                                 "SWEep")

            # Get the dynamic range indicator for DL audio path
            res_dl = AudioUtil.get_rms_from_sweep(AudioUtil.get_sweep_trace(self._host_trace_file_path),
                                                  self._logger)

            if res_dl > self._dynamic_threshold:
                self._verdict_comment = "Audio correctly routed - " + \
                                        "Dynamic indicator : %f" % res_dl
                self._logger.info(self._verdict_comment)
                self._result_verdict = Global.SUCCESS
            else:
                self._verdict_comment = "Audio routing error - " + \
                                        "Dynamic indicator : %f" % res_dl
                self._logger.error(self._verdict_comment)
                self._result_verdict = Global.FAILURE

            if self._keep_record and self._result_verdict < 0:
                # Update the name of the trace file, in case of back-to-back and KEEP_RECORD = true
                self._tc_iteration_count -= 1
                self._host_trace_file_path = self._host_trace_file_path.strip(
                    self._host_trace_file_path.split('_')[-1]) + \
                                             str(self._tc_parameters.get_b2b_iteration() - self._tc_iteration_count + 1) \
                                             + ".trc"
            else:
                os.remove(self._host_trace_file_path)

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

        return self._result_verdict, self._verdict_comment

    def tear_down(self):
        """
        End and dispose the test
        """

        # Call LabAudioActivityBase run_test function
        LabAudioActivityBase.tear_down(self)

        if self._acc_type == "HEADSET":
            if self._use_io_card:
                self._wired_headset.unplug_whs()
        elif self._acc_type == "HEADPHONE":
            if self._use_io_card:
                self._wired_headset.unplug_headphone()

        if self._audio_analyzer_node.get_param_value("Model") in "RS_UPV":
            self._audio_analyzer.release()

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
