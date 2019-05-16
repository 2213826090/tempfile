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

:organization: INTEL MCG DRD QCTV
:summary: This file implements MT voice call after exiting flight mode
:since: 24/02/2014
:author: fbelvezx
"""
import os
import time
from acs_test_scripts.Device.UECmd.UECmdTypes import BT_STATE
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.TestEquipmentException import TestEquipmentException
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsBaseException import AcsBaseException
import acs_test_scripts.Utilities.AudioUtilities as AudioUtil
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global, str_to_bool


class LabAudioFitFlightmode(UseCaseBase):
    """
    Lab Audio Flight mode interaction with CSV call class
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCaseBase Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # Read registrationTimeout from Device_Catalog.xml
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

        # Read CELLULAR_NETWORK parameters from BenchConfig.xml
        self._network = \
            global_config.benchConfig.get_parameters("CELLULAR_NETWORK")
        self._apn = self._network.get_param_value("APN")
        self._ssid = self._network.get_param_value("SSID")

        # Read callSetupTimeout from Device_Catalog.xml
        self._call_setup_time = \
            int(self._dut_config.get("callSetupTimeout")) * 2

        # Read NETWORK_SIMULATOR1 parameters from BenchConfig.xml
        self._ns_node = global_config.benchConfig.get_parameters(
            "NETWORK_SIMULATOR1")

        # Read AUDIO_ANALYZER parameters from BenchConfig.xml
        self._audio_analyzer_node = \
            global_config.benchConfig.get_parameters("AUDIO_ANALYZER")

        # Create cellular network simulator
        self._ns = self._em.get_cellular_network_simulator("NETWORK_SIMULATOR1")

        # Read NS1_CELL_TECH from test case xml file (In str)
        self._ns1_cell_tech = \
            str(self._tc_parameters.get_param_value("NS1_CELL_TECH"))

        # Read NS2_CELL_TECH from test case xml file (In str)
        self._ns2_cell_tech = \
            str(self._tc_parameters.get_param_value("NS2_CELL_TECH"))

        # Connect to cellular network simulator
        self._ns.init()

        # Set APIs instances for NS1
        if self._ns1_cell_tech == "2G":
            self._ns1_cell = self._ns.get_cell_2g()
            self._ns1_vc = self._ns1_cell.get_voice_call()
            self._ns1_data = self._ns1_cell.get_data()
            self._ns.switch_app_format("GSM/GPRS")

        elif self._ns1_cell_tech == "3G":
            self._ns1_cell = self._ns.get_cell_3g()
            self._ns1_vc = self._ns1_cell.get_voice_call()
            self._ns1_data = self._ns1_cell.get_data()
            self._ns.switch_app_format("WCDMA")

        else:
            self._error.Msg = "Unknown Cell Radio Access Technology"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, self._error.Msg)

        # Set APIs instances for NS2
        if self._ns2_cell_tech == "2G":
            self._ns2_cell = self._ns.get_cell_2g()
            self._ns2_vc = self._ns2_cell.get_voice_call()
            self._ns2_data = self._ns2_cell.get_data()

        elif self._ns2_cell_tech == "3G":
            self._ns2_cell = self._ns.get_cell_3g()
            self._ns2_vc = self._ns2_cell.get_voice_call()
            self._ns2_data = self._ns2_cell.get_data()

        else:
            self._error.Msg = "Unknown Cell Radio Access Technology"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, self._error.Msg)

        # Read test case parameters related to ns1
        self._ns1_cell_arfcn = int(self._tc_parameters.get_param_value("NS1_ARFCN"))
        self._ns1_cell_band_name = str(self._tc_parameters.get_param_value("NS1_CELL_BAND"))
        self._ns1_cell_service = \
            self._tc_parameters.get_param_value("NS1_CELL_SERVICE")
        self._ns1_cell_codec = str(self._tc_parameters.get_param_value("NS1_CODEC"))

        # Read test case parameters related to ns2
        self._ns2_cell_arfcn = int(self._tc_parameters.get_param_value("NS2_ARFCN"))
        self._ns2_cell_band_name = str(self._tc_parameters.get_param_value("NS2_CELL_BAND"))
        self._ns2_cell_service = \
            self._tc_parameters.get_param_value("NS2_CELL_SERVICE")
        self._ns2_cell_codec = str(self._tc_parameters.get_param_value("NS2_CODEC"))

        # Create audio analyzer
        self._audio_analyzer = self._em.get_audio_analyzer("AUDIO_ANALYZER")

        # Instantiate generic UECmd for voiceCall Ucs plus some others (System, Connectivity)
        self._modem_api = self._device.get_uecmd("Modem")
        self._networking_api = self._device.get_uecmd("Networking")
        self._voicecall_api = self._device.get_uecmd("VoiceCall")
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self._system_api = self._device.get_uecmd("System")
        self._bt_api = self._device.get_uecmd("LocalConnectivity")

        # Read Keep Record value from test case xml file (str)
        self._keep_record = str_to_bool(self._tc_parameters.get_param_value("KEEP_RECORD"))

        # Get the current date time for degraded audio file name
        self._cur_date_time = time.strftime('%Y%m%d_%H%M%S', time.localtime())

        # Read ConfPath from AUDIO_ANALYZER BenchConfig.xml
        self._aa_conf_path = os.path.join(str(self._audio_analyzer_node.get_param_value("ConfPath")))

        # Read DestPath from AUDIO_ANALYZER BenchConfig.xml
        self._aa_dest_path = os.path.join(str(self._audio_analyzer_node.get_param_value("DestPath")))

        self._host_trace_file_path_dl_1 = os.path.join(self._aa_conf_path,
                                                       "UPV_traces",
                                                       self._name.split('\\')[4].split('-')[0] + "-" +
                                                       str(self._tc_parameters.get_ucase_name()) + "_" +
                                                       self._cur_date_time + "_CELL1_DL_1" ".trc")

        self._host_trace_file_path_ul_1 = os.path.join(self._aa_conf_path,
                                                       "UPV_traces",
                                                       self._name.split('\\')[4].split('-')[0] + "-" +
                                                       str(self._tc_parameters.get_ucase_name()) + "_" +
                                                       self._cur_date_time + "_CELL1_UL_1" ".trc")

        self._host_trace_file_path_dl_2 = os.path.join(self._aa_conf_path,
                                                       "UPV_traces",
                                                       self._name.split('\\')[4].split('-')[0] + "-" +
                                                       str(self._tc_parameters.get_ucase_name()) + "_" +
                                                       self._cur_date_time + "_CELL2_DL_1" ".trc")

        self._host_trace_file_path_ul_2 = os.path.join(self._aa_conf_path,
                                                       "UPV_traces",
                                                       self._name.split('\\')[4].split('-')[0] + "-" +
                                                       str(self._tc_parameters.get_ucase_name()) + "_" +
                                                       self._cur_date_time + "_CELL2_UL_1" ".trc")

        # Construct trace file path on audio analyzer
        self._aa_trace_file_path_dl_1 = os.path.join(self._aa_dest_path,
                                                     "csv_sweep_trace_dl_1.trc")

        # Construct trace file path on audio analyzer
        self._aa_trace_file_path_ul_1 = os.path.join(self._aa_dest_path,
                                                     "csv_sweep_trace_ul_1.trc")

        # Construct trace file path on audio analyzer
        self._aa_trace_file_path_dl_2 = os.path.join(self._aa_dest_path,
                                                     "csv_sweep_trace_dl_2.trc")

        # Construct trace file path on audio analyzer
        self._aa_trace_file_path_ul_2 = os.path.join(self._aa_dest_path,
                                                     "csv_sweep_trace_ul_2.trc")

        self._toggle_flight_mode = str_to_bool(self._tc_parameters.get_param_value("TOGGLE_FLIGHTMODE"))

        self._wait_for_registration = str_to_bool(self._tc_parameters.get_param_value("WAIT_REGISTRATION"))

        # Call Stream Volume in percent
        self._call_stream_volume_dut = \
            int(self._tc_parameters.get_param_value("CALL_VOLUME_DUT"))

        # Duration of the audio test file
        self._test_file_duration = int(self._tc_parameters.get_param_value("TEST_SIGNAL_DURATION"))

        # Threshold for audio routing detection
        self._dynamic_threshold = int(self._tc_parameters.get_param_value("THRESHOLD"))

        # Some TC require to activate flight mode iteratively
        self._flightmode_b2b_iteration = self._tc_parameters.get_param_value("FLIGHTMODE_ITERATION")

        self._data_connection_type = self._tc_parameters.get_param_value("DATA_CONNECTION_TYPE")

        # Number of iteration for the current test
        self._tc_iteration_count = self._tc_parameters.get_b2b_iteration()

        self._trace_type = "SWEep"
        self._expected_data_connection_state_2g = None
        self._expected_data_connection_state_3g = None

        self._result_verdict = Global.FAILURE

    #-------------------------------------------------------------------------------------------------------------------
    def set_up(self):
        """
        Set up the test configuration
        """

        # Call UseCaseBase Setup function
        UseCaseBase.set_up(self)

        if not os.path.exists(os.path.join(self._aa_conf_path, "UPV_traces")):
            os.mkdir(os.path.join(self._aa_conf_path, "UPV_traces"))

        # Connect to audio analyzer
        self._audio_analyzer.init()

        # Perform Full Preset
        self._ns.perform_full_preset()

        # Set cell band using CELL_BAND parameter
        self._ns1_cell.set_band(self._ns1_cell_band_name)

        # Set cell off
        self._ns1_cell.set_cell_off()

        # Set cell service using CELL_SERVICE parameter
        self._ns1_cell.set_cell_service(self._ns1_cell_service)

        # Set cell on
        self._ns1_cell.set_cell_on()

        # Set voice call output
        self._ns1_vc.set_speech_configuration("SPEECH_OUTPUT")

        # Set the APN
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Setting APN " + str(self._apn) + "...")
        self._networking_api.set_apn(self._ssid, self._apn)

        # Activate/Deactivate PDP context, depeding on the TC
        time.sleep(self._wait_btwn_cmd)

        if self._data_connection_type in ["DATA_ONLY", "DATA_CONNECTIVITY"]:
            self._logger.info("Activate PDP Context...")
            self._networking_api.activate_pdp_context(self._ssid, check=False)

            self._expected_data_connection_state_2g = "TEST"
            self._expected_data_connection_state_3g = "CONN"

            # Turn on bluetooth in the TC requires both data and connectivity
            if self._data_connection_type in "DATA_CONNECTIVITY":
                self._logger.info("Turn on Bluetooth adapters")
                self._bt_api.set_bt_power("on")
                time.sleep(self._wait_btwn_cmd)
                if self._bt_api.get_bt_power_status() != str(BT_STATE.STATE_ON):
                    msg = "set BT ON failure"
                    self._logger.error(msg)
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        elif self._data_connection_type in "NO_DATA":
            self._logger.info("Deactivate PDP Context...")
            self._networking_api.deactivate_pdp_context(self._ssid, check=False)

            self._expected_data_connection_state_2g = "ATT"
            self._expected_data_connection_state_3g = "ATT"

        # Switch off the DUT
        if self._device.reboot():
            self._logger.info("Device rebooted")
        else:
            self._logger.error("Device not rebooted")

        return Global.SUCCESS, "No errors"

    #-------------------------------------------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """

        # Call UseCaseBase Run Test function
        UseCaseBase.run_test(self)

        # Check registration status before registrationTimeout (CDK)
        self._modem_api.check_cdk_registration_bfor_timeout(self._registration_timeout * 2)

        # Data Connection State verification in 2G
        self._ns1_data.check_data_connection_state(self._expected_data_connection_state_2g, 30)

        # The flight mode activation can be iterative
        for i in range(int(self._flightmode_b2b_iteration)):

            # Activate flight mode
            self._networking_api.set_flight_mode("on")

            # The flight mode activation/disactivation can be either toggled, or separated by a timer
            if not self._toggle_flight_mode:
                # Wait 30s while airplane mode is enabled
                time.sleep(30)

                if self._ns1_cell_tech not in self._ns2_cell_tech:
                    self._logger.info("Switch to 3G cell")

                    # Disconnect network simulator
                    self._ns.release()

                    # Connect to cellular network simulator
                    self._ns.init()

                    # Set the equipment application format WCDMA
                    self._ns.switch_app_format("WCDMA")

                    # Perform Full Preset
                    self._ns.perform_full_preset()

                    # Set paging service to AMR VOICE
                    self._ns2_cell.set_cell_service(self._ns2_cell_service)

                    # Set cell off
                    self._ns2_cell.set_cell_off()

                    # Set Cell Band UARFCN (downlink) using Band and DlUarfcn
                    self._ns2_cell.set_band_and_dl_arfcn(
                        "BAND" + str(self._ns2_cell_band_name), self._ns2_cell_arfcn)

                    # Set voice call output
                    self._ns2_vc.set_speech_configuration("SPEECH_OUTPUT")

                    # Set cell on
                    self._ns2_cell.set_cell_on()

            # Deactivate flight mode
            self._networking_api.set_flight_mode("off")

        if self._wait_for_registration:
            # Check registration status before registrationTimeout (CDK)
            self._modem_api.check_cdk_registration_bfor_timeout(self._registration_timeout * 2)

        # Data Connection State verification in 3G
        self._ns2_data.check_data_connection_state(self._expected_data_connection_state_3g, 30)

        # Initiate a MT voice call from the 3G cell
        self._ns2_vc.mt_originate_call()

        # Check call status before callSetupTimeout (DUT)
        self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.INCOMING,
                                           self._call_setup_time)

        # Answer incoming call
        self._voicecall_api.answer()

        # Check call status before callSetupTimeout (NS)
        self._ns2_vc.check_call_connected(self._call_setup_time,
                                          blocking=False)

        # Check call status before callSetupTimeout (DUT)
        self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
                                           self._call_setup_time)

        # Configure Audio output to headset (jack)
        time.sleep(self._wait_btwn_cmd)
        self._phonesystem_api.switch_audio_output("headset")

        # Set Voice Call Volume
        self._system_api.adjust_specified_stream_volume("VoiceCall", self._call_stream_volume_dut)

        # Load setup on UPV
        self._audio_analyzer.load_configuration(
            self._name,
            str(self._ns_node.get_param_value("Model")),
            None,
            None,
            "DL")

        time.sleep(self._wait_btwn_cmd)

        # Perform audio routing verification for DL path first
        self._logger.info("Verify audio routing in DL")
        self._audio_analyzer.start_single_measurement()

        # Wait until the measurement ends
        self._audio_analyzer.wait_for_sweep_state("Sweep_Waiting")

        # Transfer the result of the measurement to ACS host
        self._audio_analyzer.store_data_list(self._aa_trace_file_path_dl_1,
                                             self._host_trace_file_path_dl_1,
                                             self._trace_type)

        # Get the dynamic range indicator for DL audio path
        res_dl_3g = AudioUtil.get_rms_from_sweep(AudioUtil.get_sweep_trace(self._host_trace_file_path_dl_1),
                                                 self._logger)

        # Load UPV setup for UL path
        self._audio_analyzer.load_configuration(
            self._name,
            str(self._ns_node.get_param_value("Model")),
            None,
            None,
            "UL")

        # Perform audio routing verification for UL path
        self._logger.info("Verify audio routing in UL")
        self._audio_analyzer.start_single_measurement()

        # Wait until the measurement ends
        self._audio_analyzer.wait_for_sweep_state("Sweep_Waiting")

        # Transfer the result of the measurement to ACS host
        self._audio_analyzer.store_data_list(self._aa_trace_file_path_ul_1,
                                             self._host_trace_file_path_ul_1,
                                             self._trace_type)
        time.sleep(2)

        # Get the dynamic range indicator for UL audio path
        res_ul_3g = AudioUtil.get_rms_from_sweep(AudioUtil.get_sweep_trace(self._host_trace_file_path_ul_1),
                                                 self._logger)

        try:
            # Release the call
            self._ns2_vc.voice_call_network_release()

            # Check call is released (NS)
            self._ns2_vc.check_call_idle(self._registration_timeout,
                                         blocking=False)
        except TestEquipmentException as e:
            self._error.Msg = \
                "CSV call dropped before the end"
            self._logger.error(e)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, self._error.Msg)

        # Check call is released (CDK)
        self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.NOCALL,
                                           self._call_setup_time)

        if res_dl_3g > self._dynamic_threshold and res_ul_3g > self._dynamic_threshold:
            self._error.Msg = "Audio correctly routed in both DL and UL - " + \
                              "Dynamic indicator : DL = %f / UL = %f" % (res_dl_3g, res_ul_3g)
            self._logger.info(self._error.Msg)
            self._result_verdict = Global.SUCCESS
        elif res_dl_3g < self._dynamic_threshold and res_ul_3g > self._dynamic_threshold:
            self._error.Msg = "Audio routing error in DL - " + \
                              "Dynamic indicator : DL = %f / UL = %f" % (res_dl_3g, res_ul_3g)
            self._logger.error(self._error.Msg)
            self._result_verdict = Global.FAILURE
        elif res_ul_3g < self._dynamic_threshold and res_dl_3g > self._dynamic_threshold:
            self._error.Msg = "Audio routing error in UL - " + \
                              "Dynamic indicator : DL = %f / UL = %f" % (res_dl_3g, res_ul_3g)
            self._logger.error(self._error.Msg)
            self._result_verdict = Global.FAILURE
        else:
            self._error.Msg = "Audio routing error in both DL and UL - " + \
                              "Dynamic indicator : DL = %f / UL = %f" % (res_dl_3g, res_ul_3g)
            self._logger.error(self._error.Msg)
            self._result_verdict = Global.FAILURE

        if self._ns1_cell_tech not in self._ns2_cell_tech:

            # Activate flight mode
            self._networking_api.set_flight_mode("on")

            # Wait 30s while airplane mode is enabled
            time.sleep(30)

            # Disconnect current equipment application of network simulator
            self._ns.release()

            # Connect to cellular network simulator
            self._ns.init()

            # Set the equipment application format back to 2G
            self._ns.switch_app_format("GSM/GPRS")

            # Set cell band using CELL_BAND parameter
            self._ns1_cell.set_band(self._ns1_cell_band_name)

            # Set cell off
            self._ns1_cell.set_cell_off()

            # Set cell service using CELL_SERVICE parameter
            self._ns1_cell.set_cell_service(self._ns1_cell_service)

            # Set cell on
            self._ns1_cell.set_cell_on()

            # Set voice call output
            self._ns1_vc.set_speech_configuration("SPEECH_OUTPUT")

            # Deactivate flight mode
            self._networking_api.set_flight_mode("off")

            # Check registration status before registrationTimeout (CDK)
            self._modem_api.check_cdk_registration_bfor_timeout(self._registration_timeout * 2)

            # Data Connection State verification in 2G
            self._ns1_data.check_data_connection_state(self._expected_data_connection_state_2g, 30)

            # Perform a MT voice call
            self._ns1_vc.mt_originate_call()

            # Check call status is incoming before callSetupTimeout
            self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.INCOMING,
                                               self._call_setup_time)

            # Answer incoming call
            self._voicecall_api.answer()

            # Check call status before callSetupTimeout (NS)
            self._ns1_vc.check_call_connected(self._call_setup_time,
                                              blocking=False)

            # Check call status before callSetupTimeout (DUT)
            self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
                                               self._call_setup_time)

            # Configure Audio output to headset (jack)
            time.sleep(self._wait_btwn_cmd)
            self._phonesystem_api.switch_audio_output("headset")

            # Set Voice Call Volume at 100%
            self._system_api.adjust_specified_stream_volume("VoiceCall", self._call_stream_volume_dut)

            # Set Audio codec in 2G
            self._ns1_vc.set_audio_codec(self._ns1_cell_codec)

            # Load setup on UPV
            self._audio_analyzer.load_configuration(
                self._name,
                str(self._ns_node.get_param_value("Model")),
                None,
                None,
                "DL")

            # Perform audio routing verification for DL path first
            self._logger.info("Verify audio routing in DL")
            self._audio_analyzer.start_single_measurement()

            # Wait until the measurement ends
            self._audio_analyzer.wait_for_sweep_state("Sweep_Waiting")

            # Transfer the result of the measurement to ACS host
            self._audio_analyzer.store_data_list(self._aa_trace_file_path_dl_2,
                                                 self._host_trace_file_path_dl_2,
                                                 self._trace_type)

            # Get the dynamic range indicator for DL audio path
            res_dl_2g = AudioUtil.get_rms_from_sweep(AudioUtil.get_sweep_trace(self._host_trace_file_path_dl_2),
                                                     self._logger)

            # Load UPV setup for UL path
            self._audio_analyzer.load_configuration(
                self._name,
                str(self._ns_node.get_param_value("Model")),
                None,
                None,
                "UL")

            # Perform audio routing verification for UL path
            self._logger.info("Verify audio routing in UL")
            self._audio_analyzer.start_single_measurement()

            # Wait until the measurement ends
            self._audio_analyzer.wait_for_sweep_state("Sweep_Waiting")

            # Transfer the result of the measurement to ACS host
            self._audio_analyzer.store_data_list(self._aa_trace_file_path_ul_2,
                                                 self._host_trace_file_path_ul_2,
                                                 self._trace_type)

            # Get the dynamic range indicator for UL audio path
            res_ul_2g = AudioUtil.get_rms_from_sweep(AudioUtil.get_sweep_trace(self._host_trace_file_path_ul_2),
                                                     self._logger)
            try:
                # Release the call
                self._ns1_vc.voice_call_network_release()

                # Check call is released (NS)
                self._ns1_vc.check_call_idle(self._registration_timeout,
                                             blocking=False)
            except TestEquipmentException as e:
                self._error.Msg = \
                    "CSV call dropped before the end"
                self._logger.error(e)
                raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, self._error.Msg)

            # Check call is released (CDK)
            self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.NOCALL,
                                               self._call_setup_time)

            if res_dl_2g > self._dynamic_threshold and res_ul_2g > self._dynamic_threshold:
                self._logger.info("Audio correctly routed in both DL and UL while attached to 2G cell - " + \
                                  "Dynamic indicator : DL = %f / UL = %f" % (res_dl_2g, res_ul_2g))

                if self._result_verdict is Global.SUCCESS:
                    self._error.Msg = "Audio correctly routed in both DL and UL while attached to 2G and 3G cell - " + \
                                      "Dynamic indicator : 3G - DL = %f - UL = %f / 2G - DL = %f - UL = %f " % \
                                      (res_dl_3g, res_ul_3g, res_dl_2g, res_ul_2g)

            elif res_dl_2g < self._dynamic_threshold and res_ul_2g > self._dynamic_threshold:
                self._error.Msg = "Audio routing error in DL while attached to 2G cell - " + \
                                  "Dynamic indicator : DL = %f (3G) - %f (2G) / UL = %f (3G) - %f (2G)" % \
                                  (res_dl_3g, res_dl_2g, res_ul_3g, res_ul_2g)
                self._logger.error(self._error.Msg)
                if self._result_verdict is Global.SUCCESS:
                    self._result_verdict = Global.FAILURE
            elif res_ul_2g < self._dynamic_threshold and res_dl_2g > self._dynamic_threshold:
                self._error.Msg = "Audio routing error in UL while attached to 2G cell - " + \
                                  "Dynamic indicator : DL = %f (3G) - %f (2G) / UL = %f (3G) - %f (2G)" % \
                                  (res_dl_3g, res_dl_2g, res_ul_3g, res_ul_2g)
                self._logger.error(self._error.Msg)
                if self._result_verdict is Global.SUCCESS:
                    self._result_verdict = Global.FAILURE
            else:
                self._error.Msg = "Audio routing error in both DL and UL while attached to 2G cell - " + \
                                  "Dynamic indicator : DL = %f (3G) - %f (2G) / UL = %f (3G) - %f (2G)" % \
                                  (res_dl_3g, res_dl_2g, res_ul_3g, res_ul_2g)
                self._logger.error(self._error.Msg)
                if self._result_verdict is Global.SUCCESS:
                    self._result_verdict = Global.FAILURE

            if self._keep_record:
                # Update the name of the trace file, in case of back-to-back and KEEP_RECORD = true
                self._host_trace_file_path_dl_2 = self._host_trace_file_path_dl_2.strip(
                    self._host_trace_file_path_dl_2.split('_')[-1]) + \
                                                  str(
                                                      self._tc_parameters.get_b2b_iteration() - self._tc_iteration_count + 2) + ".trc"

                self._host_trace_file_path_ul_2 = self._host_trace_file_path_ul_2.strip(
                    self._host_trace_file_path_ul_2.split('_')[-1]) + \
                                                  str(
                                                      self._tc_parameters.get_b2b_iteration() - self._tc_iteration_count + 2) + ".trc"
            else:
                os.remove(self._host_trace_file_path_dl_2)
                os.remove(self._host_trace_file_path_ul_2)

        if self._keep_record:
            # Update the name of the trace file, in case of back-to-back and KEEP_RECORD = true
            self._host_trace_file_path_dl_1 = self._host_trace_file_path_dl_1.strip(
                self._host_trace_file_path_dl_1.split('_')[-1]) + \
                                              str(
                                                  self._tc_parameters.get_b2b_iteration() - self._tc_iteration_count + 2) + ".trc"

            self._host_trace_file_path_ul_1 = self._host_trace_file_path_ul_1.strip(
                self._host_trace_file_path_ul_1.split('_')[-1]) + \
                                              str(
                                                  self._tc_parameters.get_b2b_iteration() - self._tc_iteration_count + 2) + ".trc"
        else:
            os.remove(self._host_trace_file_path_dl_1)
            os.remove(self._host_trace_file_path_ul_1)

        self._tc_iteration_count -= 1

        return self._result_verdict, self._error.Msg

    #-------------------------------------------------------------------------------------------------------------------
    def tear_down(self):

        # Call UseCaseBase Teardown function
        UseCaseBase.tear_down(self)

        try:
            # Release any ongoing voice call
            if self._voicecall_api.get_state() is not self._uecmd_types.VOICE_CALL_STATE.NOCALL:
                # Release the call
                self._ns1_vc.voice_call_network_release()

                # Check call is released (CDK)
                self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.NOCALL, # pylint: disable=E1101
                                                   self._call_setup_time)

        except AcsBaseException as acs_exception:
            self._logger.warning("Call release fail:" + str(acs_exception))

        # Set cell off
        self._ns1_cell.set_cell_off()

        # Disconnect audio analyzer
        self._audio_analyzer.release()

        # Disconnect network simulator
        self._ns.release()

        # Turn off bluetooth
        self._logger.info("Turn off Bluetooth adapters")
        if self._bt_api.get_bt_power_status() not in str(BT_STATE.STATE_OFF):
            self._bt_api.set_bt_power("off")
            time.sleep(self._wait_btwn_cmd)
            if self._bt_api.get_bt_power_status() not in str(BT_STATE.STATE_OFF):
                msg = "set BT OFF failure"
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return Global.SUCCESS, "No errors"
