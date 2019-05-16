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
:summary: This file implements the lab audio voice call base.
:since: 06/10/2010
:author: ccontreras
"""
import os
import time
from acs_test_scripts.Utilities.AudioFrameworkUtilities import AudioFramework
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global, str_to_bool


class LabAudioWcdmaVcBase(UseCaseBase):
    """
    Lab Audio Voice Call base class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call UseCaseBase Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # Set parameters default values
        self._module_id = None
        self._cur_date_time = None
        self._host_deg_file_path = None
        self._result_verdict = None
        self._pesq_result = None
        self._host_ref_file_path = None
        self._error_msg = None
        self._wideband = None

        # Read registrationTimeout from Device_Catalog.xml
        self._registration_timeout = int(self._dut_config.get("registrationTimeout"))

        # Read CELLULAR_NETWORK parameters from BenchConfig.xml
        self._network = \
            global_config.benchConfig.get_parameters("CELLULAR_NETWORK")
        self._apn = self._network.get_param_value("APN")
        self._ssid = self._network.get_param_value("SSID")

        # Read callSetupTimeout from Device_Catalog.xml
        self._call_setup_time = int(self._dut_config.get("callSetupTimeout"))

        # Read NETWORK_SIMULATOR1 parameters from BenchConfig.xml
        self._ns_node = global_config.benchConfig.get_parameters(
            "NETWORK_SIMULATOR1")

        # Read AUDIO_ANALYZER parameters from BenchConfig.xml
        self._audio_analyzer_node = \
            global_config.benchConfig.get_parameters("AUDIO_ANALYZER")

        # Read Band from test case xml file (str)
        self._band = str(self._tc_parameters.get_param_value("CELL_BAND"))

        # Read CELL_SERVICE from test case xml file
        self._cell_service = \
            self._tc_parameters.get_param_value("CELL_SERVICE")

        # Read Band from test case xml file (str)
        self._dl_uarfcn = int(self._tc_parameters.get_param_value("DL_UARFCN"))

        # Read CODEC from test case xml file
        self._codec = str(self._tc_parameters.get_param_value("CODEC"))
        if 'WB' in self._codec:
            self._codec_type = "WB"
        elif 'NB' in self._codec:
            self._codec_type = "NB"

        # Read VC_TYPE from test case xml file
        self._vc_type = str(self._tc_parameters.get_param_value("VC_TYPE"))

        # Read AUDIO_FILE from test case xml file
        self._ref_file = str(self._tc_parameters.get_param_value("AUDIO_FILE"))

        # Create cellular network simulator and retrieve 3G APIs
        self._ns = self._em.get_cellular_network_simulator("NETWORK_SIMULATOR1")
        self._ns_3g = self._ns.get_cell_3g()
        self._vc_3g = self._ns_3g.get_voice_call()

        if self._audio_analyzer_node.get_param_value("Model") in "RS_UPV":
            # Create audio analyzer
            self._audio_analyzer = self._em.get_audio_analyzer("AUDIO_ANALYZER")
            self._ns_speech_source = "SPEECH_OUTPUT"

        elif self._audio_analyzer_node.get_param_value("Model") in "AUDIO_FRAMEWORK":
            # Create an AudioFramework object
            self._audio_framework = AudioFramework()
            self._audio_framework.server_address = (self._audio_analyzer_node.get_param_value("IP"),
                                                    int(self._audio_analyzer_node.get_param_value("server_port")))
            self._audio_framework.logger = self._logger
            self._audio_framework_timeout = 8
            self._ns_speech_source = "ECHO"

        # Instantiate generic UECmd for voiceCall Ucs
        self._modem_api = self._device.get_uecmd("Modem")
        self._networking_api = self._device.get_uecmd("Networking")
        self._voicecall_api = self._device.get_uecmd("VoiceCall")
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self._system_api = self._device.get_uecmd("System")
        self._audioalgocontroller_api = self._device.get_uecmd("AudioAlgoController")

        # Read Keep Record value from test case xml file (str)
        self._keep_record = str_to_bool(self._tc_parameters.get_param_value("KEEP_RECORD"))

        # Get the current date time for degraded audio file name
        self._cur_date_time = time.strftime('%Y%m%d_%H%M%S', time.localtime())

        # Read ConfPath from AUDIO_ANALYZER BenchConfig.xml
        self._aa_conf_path = os.path.join(
            str(self._audio_analyzer_node.get_param_value("ConfPath")))

        # Read DestPath from AUDIO_ANALYZER BenchConfig.xml
        self._aa_dest_path = os.path.join(
            str(self._audio_analyzer_node.get_param_value("DestPath")))

        # Only Audio Quality TCs require a specific audio test file
        if "AQ" in self._name:

            # Construct reference audio file path on bench computer
            self._host_ref_file_path = os.path.join(self._aa_conf_path,
                                                    self._ref_file)

            # Construct degraded audio file path on bench computer
            if "SWITCH" in self._name:
                self._host_deg_file_path = os.path.join(self._aa_conf_path,
                                                        self._name.split('\\')[-1].split('-')[0] + "-" +
                                                        self._name.split('\\')[-1].split('-')[1] + "-" +
                                                        str(self._tc_parameters.get_ucase_name()) + "_" +
                                                        self._cur_date_time + ".wav")
            else:
                self._host_deg_file_path = os.path.join(self._aa_conf_path,
                                                        self._name.split('\\')[-1].split('-')[0] + "-" +
                                                        str(self._tc_parameters.get_ucase_name()) + "_" +
                                                        self._cur_date_time + ".wav")

                # Construct reference audio file path on UPV
                self._aa_ref_file_path = os.path.join(self._aa_dest_path,
                                                      self._ref_file)

                # Construct degraded audio file path on UPV
                self._aa_deg_file_path = os.path.join(self._aa_dest_path,
                                                      "voice_degraded_upv.wav")

                self._option = None

        else:
            # Audio generator signal is handled by UPV
            self._aa_ref_file_path = None

            self._option = "DL"

        #------------------------------------------------------------------------------

    def set_up(self):
        """
        Set up the test configuration
        """

        # Call UseCaseBase Setup function
        UseCaseBase.set_up(self)

        # Connect to cellular network simulator
        self._ns.init()

        # Set the equipment application format WCDMA
        self._ns.switch_app_format("WCDMA")

        if self._audio_analyzer_node.get_param_value("Model") in "RS_UPV":
            # Connect to audio analyzer
            self._audio_analyzer.init()

            if self._aa_ref_file_path:
                # Copy audio test file to UPV
                self._audio_analyzer.copy_to_upv(self._host_ref_file_path,
                                                 self._aa_ref_file_path)

            # Load setup on UPV
            self._audio_analyzer.load_configuration(
                self._name,
                str(self._ns_node.get_param_value("Model")),
                self._aa_ref_file_path,
                self._codec_type,
                self._option)

            if not os.path.exists(os.path.join(self._aa_conf_path, "UPV_traces")):
                os.mkdir(os.path.join(self._aa_conf_path, "UPV_traces"))

        elif self._audio_analyzer_node.get_param_value("Model") in "AUDIO_FRAMEWORK":
            AudioFramework.initialize_fwk(self._audio_framework)

        # Perform Full Preset
        self._ns.perform_full_preset()

        # Set paging service to AMR VOICE
        self._ns.get_cell_3g().set_cell_service(self._cell_service)

        # Set cell off
        self._ns_3g.set_cell_off()

        # Set Cell Band UARFCN (downlink) using Band and DlUarfcn
        self._ns.get_cell_3g().set_band_and_dl_arfcn(str(self._band), self._dl_uarfcn)

        # Set voice call output
        self._vc_3g.set_speech_configuration(self._ns_speech_source)

        # Set cell on
        self._ns_3g.set_cell_on()

        # Adapt attachment procedure to CIRCUIT or PACKET
        if self._cell_service == "CIRCUIT":
            # Check phone registration on equipment before given CAMP_TIMEOUT
            # (in loop compare every second the IMSI returned by the simulator
            # and IMSI read on CDK until comparison is true or timeout expired)
            dut_imsi = \
                self._modem_api.get_imsi(self._registration_timeout)

            RegUtil.check_dut_registration_before_timeout(self._ns.get_cell_3g(),
                                                          self._networking_api,
                                                          self._logger,
                                                          dut_imsi,
                                                          self._registration_timeout)
            self._networking_api.deactivate_pdp_context()

        else:
            # Set the APN
            time.sleep(self._wait_btwn_cmd)
            self._logger.info("Setting APN " + str(self._apn) + "...")
            self._networking_api.set_apn(self._ssid, self._apn)

            # Activate PDP context
            time.sleep(self._wait_btwn_cmd)
            self._logger.info("Active PDP Context...")
            self._networking_api.activate_pdp_context(self._ssid, check=False)

            # Check Data Connection State => ATTACHED before timeout
            RegUtil.check_dut_data_connection_state_before_timeout("PDP_ACTIVE",
                                                                   self._ns.get_cell_3g(),
                                                                   self._networking_api,
                                                                   self._logger,
                                                                   self._registration_timeout,
                                                                   flightmode_cycle=True,
                                                                   blocking=False)

        # Check registration status before registrationTimeout (CDK)
        self._modem_api.check_cdk_registration_bfor_timeout(self._registration_timeout)

        # Release any previous call (Robustness)
        self._voicecall_api.release()

        # Set voice codec using CODEC parameter
        self._vc_3g.set_audio_codec(self._codec)

        return Global.SUCCESS, "No errors"

    #------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """

        # Call UseCaseBase tear_down function
        UseCaseBase.tear_down(self)

        # Set cell off
        self._ns_3g.set_cell_off()

        if self._audio_analyzer_node.get_param_value("Model") in "RS_UPV":
            # Disconnect audio analyzer
            self._audio_analyzer.release()

        # Disconnect network simulator
        self._ns.release()

        return Global.SUCCESS, "No errors"
