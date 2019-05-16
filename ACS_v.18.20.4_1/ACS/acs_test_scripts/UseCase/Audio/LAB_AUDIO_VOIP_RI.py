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
:summary: Use case to validate KRI targets during VOIP voice call
:since: 17/03/2014
:author: fbelvezx
"""

import os
import numpy
import time
from UtilitiesFWK.Utilities import Global, str_to_bool
from acs_test_scripts.UseCase.Audio.LAB_AUDIO_QUALITY_BASE import LabAudioQualityBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from Device.DeviceManager import DeviceManager
from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser
from acs_test_scripts.Utilities.VoIPServerUtilities import VoIPServerCallUtilities


class LabAudioVoipRi(LabAudioQualityBase):
    """
    AudioComms Audio VOIP Call KPI/KRI class.
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

        # Read dut sip address from test case xml file (string)
        self._dut_sip_address = \
            str(self._tc_parameters.get_param_value("DUT_SIP_ADDRESS"))

        # Read ref phone sip address from test case xml file (string)
        self._peer_sip_address = \
            str(self._tc_parameters.get_param_value("PEER_SIP_ADDRESS"))

        self._dut_sip_passphrase = self._tc_parameters.get_param_value("DUT_SIP_PASSPHRASE")
        self._peer_sip_passphrase = self._tc_parameters.get_param_value("PEER_SIP_PASSPHRASE")

        # Check if a dedicated VoIP server will be used
        if global_config.benchConfig.has_parameter("VOIP_SERVER"):
            self._voip_server_node = global_config.benchConfig.get_parameters("VOIP_SERVER")
            self._voip_server_computer = self._em.get_computer("VOIP_SERVER")
            self._voip_server_computer.init()
            self._voip_server = VoIPServerCallUtilities(self._voip_server_computer,
                                                        self._voip_server_node,
                                                        self._logger)
            self._degraded_audio_file = self._voip_server_node.get_param_value("DegradedAudioFile")
            self._voip_server_deg_file = self._voip_server_node.get_param_value(
                    "ServerRecordDir") + self._degraded_audio_file

            self._local_deg_file = self._voip_server_node.get_param_value("LocalRecordDir") + self._degraded_audio_file
            self._peer_sip_address = VoIPServerCallUtilities.sip_profiles[self._peer_sip_address.split('@')[0]] + \
                                     '@' + self._peer_sip_address.split('@')[-1]

        else:
            self._voip_server_node = None

        # Read ConfPath from AUDIO_ANALYZER BenchConfig.xml
        self._aa_conf_path = os.path.join(str(self._audio_analyzer_node.get_param_value("ConfPath")))

        # Read DestPath from AUDIO_ANALYZER BenchConfig.xml
        self._aa_dest_path = os.path.join(str(self._audio_analyzer_node.get_param_value("DestPath")))

        # Construct trace file path on audio analyzer
        self._aa_trace_file_path = os.path.join(self._aa_dest_path,
                                                "sip_sweep_trace.trc")

        self._keep_record = str_to_bool(self._tc_parameters.get_param_value("KEEP_RECORD"))

        # Number of iteration for the current test
        self._tc_iteration_count = self._tc_parameters.get_b2b_iteration()

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

        # Read AUDIO_FILE from test case xml file
        self._ref_file = str(self._tc_parameters.get_param_value("AUDIO_FILE"))

        # Construct reference audio file path on bench computer
        self._host_ref_file_path = os.path.join(self._aa_conf_path,
                                                self._ref_file)

        self._host_deg_file_path = os.path.join(self._aa_conf_path,
                                                self._name.split('\\')[-1].split('-')[0] + "-" +
                                                str(self._tc_parameters.get_ucase_name()) + "_")

        # Construct degraded audio file path on UPV
        self._aa_deg_file_path = os.path.join(self._aa_dest_path,
                                              "voice_degraded_upv.wav")

        self._wait_between_measure = int(self._tc_parameters.get_param_value("WAIT_BETWEEN_MEASURE"))

        self._offline_measurement = False

        # Get the instance of the second IO_CARD
        if self._phone2:
            if DeviceManager().get_device_config("PHONE2").get("IoCard", ""):
                self._io_card_2 = self._em.get_io_card(DeviceManager().get_device_config("PHONE2").get("IoCard", ""))
            else:
                self._io_card_2 = None
        else:
            self._offline_measurement = True
            self._io_card_2 = None

        self._polqa_result_dl = None
        self._polqa_result_ul = None

        self._elapsed_time = 0

        # Read POLQA Targets from Audio_Quality_Targets.xml
        self._polqa_target = ConfigsParser("Audio_Quality_Targets").parse_audio_quality_target("Polqa",
                                                                                               "VOIP",
                                                                                               "VoIP")

        # Number of retry permitted for a single measurement on the audio analyzer
        self._audio_analyzer_meas_retry = 5

        self._audioalgocontroller_api = self._device.get_uecmd("AudioAlgoController")

        self._result_verdict = Global.SUCCESS

    def set_up(self):
        """
        Set up the test configuration
        """
        # Call LabAudioQualityBase Init function
        LabAudioQualityBase.set_up(self)

        if self._voip_server_node:
            # Restart the SIP server
            self._voip_server.restart_asterisk()

        # Stop Sip Service
        if self._sip_call_api is not None:
            self._sip_call_api.delete_profile_sip_phone_app()

        if self._phone2 is not None:
            self._networking_api2 = self._phone2.get_uecmd("Networking")
            self._sip_call_api2 = self._phone2.get_uecmd("SipCall")
        else:
            self._sip_call_api2 = None
            self._networking_api2 = None

        if self._sip_call_api2 is not None:
            self._sip_call_api2.delete_profile_sip_phone_app()

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
        if self._phone_calling:
            self._phone_calling.initialize_sipcall_module()
        if self._phone_receiving:
            self._phone_receiving.initialize_sipcall_module()

        # Setup Sip Account
        self._sip_call_api.initialize_local_profile(self._dut_sip_address, self._dut_sip_passphrase)
        if self._phone2 is not None:
            self._sip_call_api2.initialize_local_profile(self._peer_sip_address, self._peer_sip_passphrase)

        if not self._phone2:
            # Release any ongoing calls which were using the DUT SIP account
            self._logger.info("Get the status of the channels of the SIP server")
            if self._voip_server.check_call_status(self._dut_sip_address.split('@')[0]) == 'IN_CALL':
                self._voip_server.release_call(self._dut_sip_address.split('@')[0])

        return Global.SUCCESS, "No errors"

    def run_test(self):
        """
        Execute the test
        """
        # Call LabAudioQualityBase run_test function
        LabAudioQualityBase.run_test(self)

        # In case of back-to-back iterations, the DUT or reference phone might have been unplugged
        if self._use_io_card:
            if self._device.get_state() == "unknown":
                self._io_card.usb_connector(plug=True)
                self._system_api.wait_for_device(timeout=60)
                self._device.connect_board()

            if self._io_card_2:
                if self._phone2.get_state() == "unknown":
                    self._io_card_2.usb_connector(plug=True)
                    self._system_api2.wait_for_device(timeout=60)
                    self._phone2.connect_board()

        if self._phone_calling:
            if self._phone_calling.get_sip_call_state() is not self._uecmd_types.SIP_CALL_STATE.READY_TO_CALL:
                self._logger.info("Read state of calling phone: %s" % self._phone_calling.get_sip_call_state())
                self._phone_calling.release()
            self._phone_calling.wait_for_state(
                    self._uecmd_types.SIP_CALL_STATE.READY_TO_CALL,
                    self._call_setup_time)

        if self._phone_receiving:
            if self._phone_receiving.get_sip_call_state() is not self._uecmd_types.SIP_CALL_STATE.READY_TO_CALL:
                self._logger.info("Read state of receiving phone: %s" % self._phone_receiving.get_sip_call_state())
                self._phone_receiving.release()
            self._phone_receiving.wait_for_state(
                    self._uecmd_types.SIP_CALL_STATE.READY_TO_CALL,
                    self._call_setup_time)

        # Dial using PHONE_NUMBER parameter
        if self._phone_calling:
            if self._phone2:
                self._phone_calling.dial(self._calling_phone_number)
            else:
                self._phone_calling.dial(self._calling_phone_number, check_state=False)
        else:
            self._voip_server.start_mt_call(self._dut_sip_address.split('@')[0],
                                            self._peer_sip_address.split('@')[0])

        if self._phone_receiving:
            self._phone_receiving.wait_for_state(
                    self._uecmd_types.SIP_CALL_STATE.INCOMING_CALL,
                    self._call_setup_time)

            self._phone_receiving.answer()

        # Phone1 & 2 : Check voice call is active
        if self._phone_calling:
            self._phone_calling.wait_for_state(
                    self._uecmd_types.SIP_CALL_STATE.IN_CALL,
                    self._call_setup_time)

        if self._phone_receiving:
            self._phone_receiving.wait_for_state(
                    self._uecmd_types.SIP_CALL_STATE.IN_CALL,
                    self._call_setup_time)

        # Set Voice Call Volume
        self._system_api.adjust_specified_stream_volume("VoiceCall",
                                                        self._call_stream_volume_dut)

        if self._phone_receiving and self._phone2:
            self._system_api2.adjust_specified_stream_volume("VoiceCall",
                                                             self._call_stream_volume_ref)

        start_time = time.localtime()

        if self._elapsed_time > 0:
            self._elapsed_time = 0

        polqa_result_dl = []
        polqa_result_ul = []

        # Start audio quality measurement
        while self._elapsed_time < self._call_duration:

            # Unplug IO card(s)
            if self._use_io_card:
                self._device.disconnect_board()
                self._io_card.usb_connector(plug=False)

                if self._io_card_2:
                    self._phone2.disconnect_board()
                    self._io_card_2.usb_connector(plug=False)

            # If only 1 phone is used, the acquisition of DL and UL audio stream are done using a SIP server
            if not self._phone2:
                # For DL, switch to a playback call profile
                self._voip_server.change_call_profile(self._dut_sip_address.split('@')[0],
                                                      VoIPServerCallUtilities.sip_profiles["PLAYBACK"])

            # Start POLQA measurement in DL
            [tmp_polqa_result_dl, meas_in_range_dl] = self.__start_audio_quality_mos("POLQA",
                                                                                     "DL",
                                                                                     self._audio_analyzer_meas_retry)

            # If the current measurement result is not in the expected MOS range, do not take it into account
            if meas_in_range_dl:
                polqa_result_dl.append(float(tmp_polqa_result_dl))

                # If User does not want to keep recorded audio file, it will not be stored
                # only if test is PASS
                if self._keep_record is True or polqa_result_dl[-1] < float(self._polqa_target):
                    self._host_deg_file_path += time.strftime('%Y%m%d_%H%M%S', time.localtime()) + "_DL" ".wav"
                    self._audio_analyzer.copy_from_upv(self._aa_deg_file_path, self._host_deg_file_path)

                    self._host_deg_file_path = self._host_deg_file_path.split(str(time.localtime().tm_year))[0]

            if not self._phone2:
                # For UL, switch to a record call profile
                self._voip_server.change_call_profile(self._dut_sip_address.split('@')[0],
                                                      VoIPServerCallUtilities.sip_profiles["RECORD"])
                self._audio_analyzer.start_single_measurement(wait_for_result=False)
                self._audio_analyzer.wait_for_meas_state("measurement_terminated")

                self._voip_server_computer.copy_file_in_local_path(self._voip_server_deg_file, self._local_deg_file)
                # Copy degraded audio file to UPV for offline POLQA measurement
                self._audio_analyzer.copy_to_upv(os.path.join(self._aa_conf_path, self._degraded_audio_file),
                                                 self._aa_deg_file_path)

                # Start POLQA measurement in offline mode
                [tmp_polqa_result_ul, meas_in_range_ul] = self.__start_audio_quality_mos("POLQA",
                                                                                         "DL",
                                                                                         self._audio_analyzer_meas_retry,
                                                                                         load_setup=False,
                                                                                         offline=True)
            else:
                # Start POLQA measurement in UL
                [tmp_polqa_result_ul, meas_in_range_ul] = self.__start_audio_quality_mos("POLQA",
                                                                                         "UL",
                                                                                         self._audio_analyzer_meas_retry)

            # Plug IO card(s)
            if self._use_io_card:
                self._io_card.usb_connector(plug=True)

                self._system_api.wait_for_device(timeout=60)
                self._device.connect_board()

                if self._io_card_2:
                    self._io_card_2.usb_connector(plug=True)

                    self._system_api2.wait_for_device(timeout=60)
                    self._phone2.connect_board()

            # If the current measurement result is not in the expected MOS range, do not take it into account
            if meas_in_range_ul:
                polqa_result_ul.append(float(tmp_polqa_result_ul))

                # If User does not want to keep recorded audio file, it will not be stored
                # only if test is PASS
                if self._keep_record is True or polqa_result_ul[-1] < float(self._polqa_target):
                    self._host_deg_file_path += time.strftime('%Y%m%d_%H%M%S', time.localtime()) + "_UL" + ".wav"
                    self._audio_analyzer.copy_from_upv(self._aa_deg_file_path, self._host_deg_file_path)

                    self._host_deg_file_path = self._host_deg_file_path.split(str(time.localtime().tm_year))[0]

            if meas_in_range_dl and meas_in_range_ul:
                self._error.Msg = "Current POLQA result (DL/UL) : %f / %f, POLQA target : %s" % (polqa_result_dl[-1],
                                                                                                 polqa_result_ul[-1],
                                                                                                 self._polqa_target)
                self._logger.info(self._error.Msg)
            else:
                self._error.Msg = "POLQA result is out of range after %d retry. No usable result for this iteration." \
                                  % self._audio_analyzer_meas_retry
                self._logger.error(self._error.Msg)
                self._result_verdict = Global.BLOCKED

            # Maintain the VOIP call for wait_between_measure s before starting a new measurement
            self._logger.info(
                    "Maintain the VOIP call for %d s before starting a new measurement" % self._wait_between_measure)
            time.sleep(self._wait_between_measure)

            # Get elapsed time since call establishment in s
            self._elapsed_time = self.__get_elapsed_time(start_time, time.localtime())

            # Check if call is still connected
            if self._sip_call_api.get_sip_call_state() is not self._uecmd_types.SIP_CALL_STATE.IN_CALL:
                msg = "VOIP call interrupted after %d seconds, state is %s !" % (self._elapsed_time,
                                                                                 self._sip_call_api.get_sip_call_state())
                self.get_logger().error(msg)
                raise DeviceException(DeviceException.INVALID_DEVICE_STATE, msg)
            else:
                self.get_logger().info(
                        "VOIP call still connected after %d seconds!",
                        self._elapsed_time)

        # RELEASE THE CALL
        # Phone1 & 2 : Check call is still active
        if self._phone_calling:
            self._phone_calling.wait_for_state(
                    self._uecmd_types.SIP_CALL_STATE.IN_CALL, self._call_setup_time)

        if self._phone_receiving:
            self._phone_receiving.wait_for_state(
                    self._uecmd_types.SIP_CALL_STATE.IN_CALL, self._call_setup_time)

        if self._phone_releasing:
            self._phone_releasing.release()

            time.sleep(self._wait_btwn_cmd)

            self._phone_releasing.wait_for_state(
                    self._uecmd_types.SIP_CALL_STATE.READY_TO_CALL,
                    self._call_setup_time)
        else:
            self._voip_server.release_call(self._dut_sip_address.split('@')[0])

        if self._phone_receiving:
            self._phone_receiving.wait_for_state(
                    self._uecmd_types.SIP_CALL_STATE.READY_TO_CALL,
                    self._call_setup_time)

        if self._result_verdict is not Global.BLOCKED:
            self._error.Msg = "Median POLQA result (DL/UL) : %f / %f, POLQA target : %s" % (
                float(numpy.median(numpy.array(polqa_result_dl))),
                float(numpy.median(numpy.array(polqa_result_ul))),
                self._polqa_target)

            # Compare the result of POLQA process with POLQA targets
            # Compute test verdict (if POLQA result > POLQA target the test pass,
            # else the test fails)
            if float(numpy.median(numpy.array(polqa_result_dl))) > float(self._polqa_target) \
                    and float(numpy.median(numpy.array(polqa_result_ul))) > float(self._polqa_target):
                self._logger.info(self._error.Msg)
                self._result_verdict = Global.SUCCESS
            else:
                self._logger.error(self._error.Msg)
                self._result_verdict = Global.FAILURE
                self._error.Msg += ""

        return self._result_verdict, self._error.Msg

    def tear_down(self):
        """
        End and dispose the test
        """

        # Call LabAudioQualityBase run_test function
        LabAudioQualityBase.tear_down(self)

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
            if self._wifi_security in ["EAP-WPA", "EAP-WPA2"]:
                # Optional parameters
                eap_method = str(self._wifirouter.get_param_value("EAP_METHOD"))
                eap_user = str(self._wifirouter.get_param_value("EAP_USER"))
                eap_password = str(self._wifirouter.get_param_value("EAP_PASSWORD"))
                phase2_auth = str(self._wifirouter.get_param_value("PHASE2_AUTH"))
                use_certificate = self._wifirouter.get_param_value("CERTIFICAT_NAME")
                certificat_name = str(self._wifirouter.get_param_value("USE_CERTIFICATE"))

                # Compute passphrase for WPA-ENT (will be parsed by ACS Embedded)
                self._wifi_passphrase = eap_method + "-" + phase2_auth \
                                        + "_" + eap_user + "_" + eap_password \
                                        + "_" + certificat_name + "_" + str(use_certificate)
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

    def __get_elapsed_time(self, time_stamp_start, time_stamp_current):
        """
        Get elapsed time between 2 time stamps

        :type time_stamp_start: time.struct_time
        :param time_stamp_start: reference time stamp
        :type time_stamp_current: time.struct_time
        :param time_stamp_current: current time stamp
        """
        elapsed_time = [3600 * 24 * 12 * 365, 3600 * 24 * 12, 3600 * 24, 3600, 60, 1]

        for i in range(len(time_stamp_start) - 3):
            elapsed_time[i] *= time_stamp_current[i] - time_stamp_start[i]

        return sum(elapsed_time)

    def __start_audio_quality_mos(self, test_type, direction, meas_retry, load_setup=True, offline=False):
        """
        Macro function that loads a setup on the UPV audio analyzer, then start a measurement, and returns its value,
        if it's within the admitted range (between 1 and 4.5).
        :type test_type: str
        :param test_type: Type of Audio Quality test (POLQA/PESQ/... )
        :type direction: str
        :param direction: Type of stream on which to perform audio measurement (DL/UL)
        :type meas_retry: int
        :param meas_retry: number of retry
        """

        mos_result_in_range = False
        mos_result = 0
        retry_nb = 0

        while not mos_result_in_range and retry_nb < meas_retry:
            if load_setup:
                # Load POLQA DL setup on RS UPV
                self._audio_analyzer.load_configuration(
                        self._name,
                        None,
                        None,
                        None,
                        direction)

            # Start POLQA measurement
            if self._offline_measurement:
                mos_result = self._audio_analyzer.audio_quality_mos(self._aa_deg_file_path,
                                                                    test_type,
                                                                    direction,
                                                                    offline)
            else:
                mos_result = self._audio_analyzer.audio_quality_mos(self._aa_deg_file_path,
                                                                    test_type,
                                                                    direction)
            # Offline measurement if by default associated with UL
            if offline:
                self._logger.info("UL POLQA result : %s" % str(mos_result))
            else:
                self._logger.info("%s POLQA result : " % direction + str(mos_result))

            # Check if POLQA result is within the expected range
            if 0.5 < mos_result < 5:
                mos_result_in_range = True
            else:
                warning_msg = "MOS value out of range, retrying the current measurement"
                self._logger.warning(warning_msg)

            retry_nb += 1

        if not mos_result_in_range:
            mos_result = 0
            error_msg = "MOS value out of range, ignoring the current measurement"
            self._logger.error(error_msg)

        return [mos_result, mos_result_in_range]
