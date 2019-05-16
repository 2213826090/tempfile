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
:summary: This file implements the AudioComms Audio Glitch detection Base class
:since: 23/09/2013
:author: nprecigx
"""

from UtilitiesFWK.Utilities import Global, str_to_bool
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from Device.DeviceManager import DeviceManager
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Device.UECmd.UECmdTypes import PreferredNetwork

# pylint: disable=E1101


class LabAudioGlitchDetectionBase(UseCaseBase):
    """
    AudioComms Audio CSV Call class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call UseCaseBase Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # Create Audio Analyzer instance
        self._audio_analyzer = self._em.get_audio_analyzer("AudioAnalyzer")

        # Check if IO card is used
        self._use_io_card = str_to_bool(global_config.campaignConfig.get("isIoCardUsed"))

        # Create Whs
        if self._use_io_card:
            self._wired_headset = self._em.get_wired_headset("WiredHeadset")


        # Initialization of the test type
        self._test_type = "glitchdetection"

        # Initialization of test result
        self._result_verdict = Global.FAILURE

        # Initialization of the verdict comment
        self._verdict_comment = "Audio Glitch detection test fail"

        # Initialization of _dut_bt_address
        self._dut_bt_address = "None"

        # Read registrationTimeout from Device_Catalog.xml
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

        # Read callSetupTimeout from Device_Catalog.xml
        self._call_setup_time = \
            int(self._dut_config.get("callSetupTimeout"))

        # Read call origin type from test case xml file (str)
        self._call_origin_type = \
            str(self._tc_parameters.get_param_value("CALL_ORIGIN_TYPE"))

        # Read accessories type from test case xml file (str)
        self._acc_type = \
            str(self._tc_parameters.get_param_value("ACC_TYPE"))

        # Call Stream Volume in percent
        self._call_stream_volume_dut = \
            int(self._tc_parameters.get_param_value("CALL_VOLUME_DUT"))

        # Call Stream Volume in percent
        self._call_stream_volume_ref = \
            int(self._tc_parameters.get_param_value("CALL_VOLUME_REF"))

        # Read call end type from test case xml file (str)
        self._call_end_type = \
            str(self._tc_parameters.get_param_value("CALL_END_TYPE"))

        # Read call type from test case xml file (str)
        self._call_type = str(self._tc_parameters.get_param_value("CALL_TYPE"))

        # Read test call direction type from test case xml file (str)
        self._signal_tested_direction = \
            str(self._tc_parameters.get_param_value("SIGNAL_TESTED_DIRECTION"))

        # Read duration type from test case xml file (int)
        if self._tc_parameters.get_param_value("DURATION"):
            self._call_duration = int(self._tc_parameters.get_param_value("DURATION"))
        else:
            # Call duration by default
            self._call_duration = 5

        # Read Audio Analyzer parameters from bench config xml file (str)
        self._audio_analyzer_node = global_config.benchConfig.\
            get_parameters("AudioAnalyzer")
        self._bt_remote_addr = self._audio_analyzer_node.get_param_value("Bt_mac_addr")

        # Instantiate generic UECmd for voiceCall Ucs
        self._phonemodem_api = self._device.get_uecmd("Modem")
        self._system_api = self._device.get_uecmd("System")

        if self._acc_type.find("BLUETOOTH") != -1:
            # Get UECmdLayer
            self._bt_api = self._device.get_uecmd("LocalConnectivity")

        # Load instance of the PHONE2
        self._phone2 = DeviceManager().get_device("PHONE2")
        # Instantiate system UECmd for Phone2
        self._system_api2 = None

    def set_up(self):
        """
        Set up the test configuration
        """

        # Call UseCaseBase Setup function
        UseCaseBase.set_up(self)

        # Check if we have the second phone available
        if self._phone2 is not None:
            if not self._phone2.is_available():
                self._phone2.switch_on(simple_switch_mode=True)
            self._system_api2 = self._phone2.get_uecmd("System")
        else:
            # We are using this multi UC with only one phone
            error_msg = \
                "This test case requires two phones to be executed !"
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG,
                                     error_msg)

        # Swap 2G 3G or 4G.
        if self._call_type == "2G":
            self._networking_api.set_preferred_network_type(PreferredNetwork.GSM_ONLY)
        elif self._call_type == "3G":
            self._networking_api.set_preferred_network_type(PreferredNetwork.WCDMA_ONLY)
        elif self._call_type == "4G":
            self._networking_api.set_preferred_network_type(PreferredNetwork.LTE_ONLY)
        else:
            if self._call_type not in "VOIP":
                msg = "wrong value of parameter CALL TYPE"
                self._logger.error(msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Audio Analyzer Initialization
        if self._audio_analyzer.initialization(self._device.get_phone_model(), self._test_type) != 0:
            error_msg = \
                "Audio Analyzer Initialization Failed !"
            self._logger.error(error_msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, error_msg)

        if self._acc_type.find("BLUETOOTH") != -1:
            # Get dut bt address
            self._dut_bt_address = self._bt_api.get_bt_adapter_address()

    def run_test(self):
        """
        Execute the test
        """
        # Call UseCaseBase run_test function
        UseCaseBase.run_test(self)

        # Connect Bluetooth device
        if self._acc_type.find("BLUETOOTH") != -1:
            if self._audio_analyzer.connect_bt(self._acc_type, self._dut_bt_address) != 0:
                error_msg = \
                    "Connect Bluetooth failed !"
                self._logger.error(error_msg)
                raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, error_msg)

    def tear_down(self):
        """
        End and dispose the test
        """

        # Call UseCaseBase tear_down function
        UseCaseBase.tear_down(self)

        # Reset Voice Call Volume at 70%
        if self._system_api is not None:
            self._system_api.adjust_specified_stream_volume("VoiceCall", 70)
        if self._system_api2 is not None:
            self._system_api2.adjust_specified_stream_volume("VoiceCall", 70)

    def __compute_test_verdict__(self, audio_analyzer_result):
        """
        Compute the result verdict and the verdict comment

        :type audio_analyzer_result: int
        :param audio_analyzer_result: the audio analyzer return code return by run command
        """

        if audio_analyzer_result == 0:
            self._result_verdict = Global.SUCCESS
            self._verdict_comment = ("Audio Glitch detection test success: in %s with "
                                     "a %s call and %s accessories"
                                     % (self._signal_tested_direction,
                                        self._call_type,
                                        self._acc_type))
        elif audio_analyzer_result == 1:
            self._result_verdict = Global.FAILURE
            self._verdict_comment = ("Audio Glitch detection test fail: Audio Analyzer quality audio "
                                     "problems detected in %s with a %s call and "
                                     "%s accessories"
                                     % (self._signal_tested_direction,
                                        self._call_type,
                                        self._acc_type))
        elif audio_analyzer_result == 2:
            self._result_verdict = Global.FAILURE
            self._verdict_comment = ("Audio Glitch detection test fail: No audio signal in %s with "
                                     "a %s call and %s accessories"
                                     % (self._signal_tested_direction,
                                        self._call_type,
                                        self._acc_type))
        else:
            self._result_verdict = Global.BLOCKED
            self._verdict_comment = ("Audio Glitch detection test fail: Audio Analyzer problems detected "
                                     "in %s with a %s call and %s accessories"
                                     % (self._signal_tested_direction,
                                        self._call_type,
                                        self._acc_type))
