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

No license under any patent, copyright, trade secret or other intellectual
property right is granted to or conferred upon you by disclosure or delivery
of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

@organization: INTEL MCG PSI
@summary:  This file is the base Use Case for IMS Audio
@since: 18/03/2014
@author: fbelvezx
"""

import os
from UtilitiesFWK.Utilities import Global, str_to_bool
from acs_test_scripts.UseCase.Networking.LAB_LTE_BASE import LabLteBase


class LabAudioImsVcBase(LabLteBase):
    """
    Lab Audio IMS VC Base class
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        LabLteBase.__init__(self, tc_name, global_config)

        # Read AUDIO_ANALYZER parameters from BenchConfig.xml
        self._audio_analyzer_node = \
            global_config.benchConfig.get_parameters("AUDIO_ANALYZER")

        self._ims = "on"

        self._call_stream_volume_dut = self._tc_parameters .get_param_value("CALL_VOLUME_DUT")

        # Read CODEC from test case xml file
        self._codec = str(self._tc_parameters.get_param_value("CODEC"))
        if 'WB' in self._codec:
            self._codec_type = "WB"
        elif 'NB' in self._codec:
            self._codec_type = "NB"

        # Read Keep Record value from test case xml file (string)
        self._keep_record = str_to_bool(self._tc_parameters.get_param_value("KEEP_RECORD"))

        # Read AUDIO_FILE from test case xml file
        self._ref_file = str(self._tc_parameters.get_param_value("AUDIO_FILE"))

        # Read time between 2 successive measurement
        self._wait_between_measure = int(self._tc_parameters.get_param_value("WAIT_BETWEEN_MEASURE"))

        # Read ConfPath from AUDIO_ANALYZER BenchConfig.xml
        self._aa_conf_path = os.path.join(
            str(self._audio_analyzer_node.get_param_value("ConfPath")))

        # Read DestPath from AUDIO_ANALYZER BenchConfig.xml
        self._aa_dest_path = os.path.join(
            str(self._audio_analyzer_node.get_param_value("DestPath")))

        # Construct reference audio file path on bench computer
        self._host_ref_file_path = os.path.join(self._aa_conf_path,
                                                self._ref_file)

        self._host_deg_file_path = os.path.join(self._aa_conf_path,
                                                self._name.split('\\')[-1].split('-')[0] + "-" +
                                                str(self._tc_parameters.get_ucase_name()) + "_")

        # Construct reference audio file path on UPV
        self._aa_ref_file_path = os.path.join(self._aa_dest_path,
                                              self._ref_file)

        # Construct degraded audio file path on UPV
        self._aa_deg_file_path = os.path.join(self._aa_dest_path,
                                              "voice_degraded_upv.wav")

        # Read callSetupTimeout from Device_Catalog.xml
        self._call_setup_time = \
            int(self._dut_config.get("callSetupTimeout"))

        self._phone_number = \
            str(self._tc_parameters.get_param_value("PHONE_NUMBER"))
        if self._phone_number.upper() == "[PHONE_NUMBER]":
            self._phone_number = str(self._device.get_phone_number())

        # Read CALL_DURATION from test case xml file
        self._call_duration = \
            int(self._tc_parameters.get_param_value("CALL_DURATION"))

        # Read VC_TYPE from testcase xml file
        self._vc_type = str(self._tc_parameters.get_param_value("VC_TYPE"))

        # Read RELEASE_VC_SIDE from testcase xml file
        self._release_vc_type = str(self._tc_parameters.get_param_value("RELEASE_VC_TYPE"))

        self._analog_input_peak_level = self._ns_node.get_param_value("AnalogInputPeakLevel")
        self._analog_output_peak_level = self._ns_node.get_param_value("AnalogOutputPeakLevel")

        self._elapsed_time = 0

        # Instantiate generic UECmd for all use cases
        self._voice_call_api = self._device.get_uecmd("VoiceCall")
        self._networking_api = self._device.get_uecmd("Networking")
        self._modem_api = self._device.get_uecmd("Modem")
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self._system_api = self._device.get_uecmd("System")
        self._audioalgocontroller_api = self._device.get_uecmd("AudioAlgoController")

        # Create voice call 4G API
        self._ns_voice_call_4g = self._ns.get_cell_4g().get_voice_call()

        # Create audio analyzer
        self._audio_analyzer = self._em.get_audio_analyzer("AUDIO_ANALYZER")

        # Number of retry permitted for a single measurement on the audio analyzer
        self._audio_analyzer_meas_retry = 5

    #------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """

        LabLteBase.set_up(self)

        # Connect to audio analyzer
        self._audio_analyzer.init()

        # Perform audio configuration
        self._ns_voice_call_4g.set_audio_codec(self._codec)
        self._ns_voice_call_4g.set_speech_configuration("AUDioboard")

        self._ns_voice_call_4g.calibrate_analog_audio_level(self._analog_input_peak_level,
                                                            self._analog_output_peak_level)

        if self._aa_ref_file_path:
            # Copy audio test file to UPV
            self._audio_analyzer.copy_to_upv(self._host_ref_file_path,
                                             self._aa_ref_file_path)

        return Global.SUCCESS, "No errors"

    #------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """

        # Call LabLteBase Tear down
        LabLteBase.tear_down(self)

        # Disconnect audio analyzer
        self._audio_analyzer.release()

        return Global.SUCCESS, "No errors"
