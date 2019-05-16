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
:summary: This file implements the AudioComms accessories change during
call base class.
:since: 13/03/2013
:author: nprecigx
"""

from UtilitiesFWK.Utilities import Global, str_to_bool
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from Device.DeviceManager import DeviceManager
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.TestEquipmentException import TestEquipmentException
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Device.UECmd.UECmdTypes import PreferredNetwork

# pylint: disable=E1101


class LabAudioQualityAccessoriesChangeBase(UseCaseBase):

    """
    AudioComms Audio CSV Call Accessories Change class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call UseCaseBase Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # Create Audio Analyzer
        self._audio_analyzer = self._em.get_audio_analyzer("AudioAnalyzer")

        # Check if IO card is used
        self._use_io_card = str_to_bool(global_config.campaignConfig.get("isIoCardUsed"))

        # Create Whs
        if self._use_io_card:
            self._wired_headset = self._em.get_wired_headset("WiredHeadset")

        # Initialization of the test type
        self._test_type = "audioquality"

        # Initialization of accessories type
        self._acc_type = "None"
        self._previous_acc_type = "None"

        # Initialization of audio_analyzer_result saved
        self._audio_analyzer_result_save = 0
        self._audio_analyzer_result_ul = 0
        self._audio_analyzer_result_dl = 0

        # Initialization of failed transition saved
        self._failed_transition_audio_pb = ""
        self._failed_transition_no_audio = ""

        # Initialization of _dut_bt_address
        self._dut_bt_address = "None"

        # Initialization of test result
        self._result_verdict = Global.SUCCESS

        # Initialization of the verdict comment
        self._verdict_comment = "Audio Quality test fail"

        # Read registrationTimeout from Device_Catalog.xml
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

        # Read callSetupTimeout from Device_Catalog.xml
        self._call_setup_time = \
            int(self._dut_config.get("callSetupTimeout"))

        # Read call origin type from test case xml file (str)
        self._call_origin_type = str(self._tc_parameters.get_param_value("CALL_ORIGIN_TYPE"))

        # Read accessories list from test case xml file (str)
        self._acc_list = str(self._tc_parameters.get_param_value("ACC_LIST"))
        # Split the accessories list, accessories separated by ','
        self._acc_list = self._acc_list.strip('[] ')
        self._acc_list_split = self._acc_list.split(',')
        self._acc_list_split.reverse()

        # Read accessories active list from test case xml file (str)
        self._acc_active_list = str(self._tc_parameters.get_param_value("ACC_ACTIVE_LIST"))
        # Split the accessories active list, accessories separated by ','
        self._acc_active_list = self._acc_active_list.strip('[] ')
        self._acc_active_list_split = self._acc_active_list.split(',')
        self._acc_active_list_split.reverse()

        # Call Stream Volume in percent
        self._call_stream_volume_dut_list = \
            str(self._tc_parameters.get_param_value("CALL_VOLUME_DUT_LIST"))
        # Split the accessories list, accessories separated by ','
        self._call_stream_volume_dut_list = self._call_stream_volume_dut_list.strip('[] ')
        self._call_volume_dut_list_split = self._call_stream_volume_dut_list.split(',')
        self._call_volume_dut_list_split.reverse()

        # Call Stream Volume in percent
        self._call_stream_volume_ref_list = \
            str(self._tc_parameters.get_param_value("CALL_VOLUME_REF_LIST"))
        # Split the accessories list, accessories separated by ','
        self._call_stream_volume_ref_list = self._call_stream_volume_ref_list.strip('[] ')
        self._call_volume_ref_list_split = self._call_stream_volume_ref_list.split(',')
        self._call_volume_ref_list_split.reverse()

        # Read call end type from test case xml file (str)
        self._call_end_type = str(self._tc_parameters.get_param_value("CALL_END_TYPE"))

        # Read call type from test case xml file (str)
        self._call_type = str(self._tc_parameters.get_param_value("CALL_TYPE"))

        # Read test call direction type from test case xml file (s tring)
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
        self._system_api = self._device.get_uecmd("System")
        self._networking_api = self._device.get_uecmd("Networking")

        # Bluetooth present in the list accessories
        if self._acc_list.count("BLUETOOTH") > 0:
            # Get UECmdLayer
            self._bt_api = self._device.get_uecmd("LocalConnectivity")

        # Load instance of the PHONE2
        self._phone2 = DeviceManager().get_device("PHONE2")
        self._system_api2 = None

        self._acc_swap_type = self._tc_parameters.get_param_value("ACC_SWAP_TYPE", 'Default')

        # Copy of accessories list
        self._temp_acc_list_split = None

        # Copy of volume list / active accessory list
        self._temp_call_volume_dut_list_split = None
        self._temp_call_volume_ref_list_split = None
        self._temp_acc_active_list_split = None

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

        if self._acc_list.count("BLUETOOTH") > 0:
            # Get dut bt address
            self._dut_bt_address = self._bt_api.get_bt_adapter_address()

            # Push the first Bluetooth accessories in first
            l_bluetooth_acc_position = 0
            for i in self._acc_list_split:
                if i.find("BLUETOOTH") != -1:
                    # Initialization of _acc_type with the bluetooth accessories
                    self._acc_type = i
                    self._acc_list_split.insert(len(self._acc_list_split), i)
                    self._acc_active_list_split.insert(
                        len(self._acc_active_list_split),
                        self._acc_active_list_split[l_bluetooth_acc_position])
                    self._call_volume_ref_list_split.insert(
                        len(self._call_volume_ref_list_split),
                        self._call_volume_ref_list_split[l_bluetooth_acc_position])
                    self._call_volume_dut_list_split.insert(
                        len(self._call_volume_dut_list_split),
                        self._call_volume_dut_list_split[l_bluetooth_acc_position])
                    break
                l_bluetooth_acc_position += 1

            # Connect Bluetooth device
            if self._audio_analyzer.connect_bt(self._acc_type, self._dut_bt_address) != 0:
                error_msg = \
                    "Connect Bluetooth failed !"
                self._logger.error(error_msg)
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                      error_msg)

        return Global.SUCCESS, "No errors"

    def tear_down(self):
        """
        End and dispose the test
        """

        # Call UseCaseBase tear_down function
        UseCaseBase.tear_down(self)

        # Reset Voice Call Volume at 50%
        if self._system_api is not None:
            self._system_api.adjust_specified_stream_volume("VoiceCall", 70)
        if self._system_api2 is not None:
            self._system_api2.adjust_specified_stream_volume("VoiceCall", 70)

        return Global.SUCCESS, "No errors"

    def _compute_test_verdict_(self, audio_analyzer_result_ul, audio_analyzer_result_dl):
        """
        Computes the test verdict for audio accessory change
        :type audio_analyzer_result_dl: int
        :param audio_analyzer_result_dl: result value for the DL audio test given by the audio analyzer
        :type audio_analyzer_result_ul: int
        :param audio_analyzer_result_ul: result value for the UL audio test given by the audio analyzer

        :return: None
        """

        if self._audio_analyzer_result_save == 3:
            self._result_verdict = Global.BLOCKED
        elif self._audio_analyzer_result_save == 2:
            if (audio_analyzer_result_ul == 3) or (audio_analyzer_result_dl == 3):
                self._result_verdict = Global.BLOCKED
                self._failed_transition_no_audio = self._failed_transition_no_audio + ", " + \
                    self._previous_acc_type + " => " + self._acc_type
                self._audio_analyzer_result_save = 3
            elif (audio_analyzer_result_ul == 2) or (audio_analyzer_result_dl == 2):
                self._result_verdict = Global.FAILURE
                self._failed_transition_no_audio = self._failed_transition_no_audio + ", " + \
                    self._previous_acc_type + " => " + self._acc_type
                self._audio_analyzer_result_save = 2
        elif self._audio_analyzer_result_save == 1:
            if (audio_analyzer_result_ul == 3) or (audio_analyzer_result_dl == 3):
                self._result_verdict = Global.BLOCKED
                self._failed_transition_no_audio = self._failed_transition_no_audio + ", " + \
                    self._previous_acc_type + " => " + self._acc_type
                self._audio_analyzer_result_save = 3
            elif (audio_analyzer_result_ul == 2) or (audio_analyzer_result_dl == 2):
                self._result_verdict = Global.FAILURE
                self._failed_transition_no_audio = self._failed_transition_no_audio + ", " + \
                    self._previous_acc_type + " => " + self._acc_type
                self._audio_analyzer_result_save = 2
            elif (audio_analyzer_result_ul == 1) or (audio_analyzer_result_dl == 1):
                self._result_verdict = Global.FAILURE
                self._failed_transition_audio_pb = self._failed_transition_audio_pb + ", " + \
                    self._previous_acc_type + " => " + self._acc_type
        elif self._audio_analyzer_result_save == 0:
            if (audio_analyzer_result_ul == 3) or (audio_analyzer_result_dl == 3):
                self._result_verdict = Global.BLOCKED
                self._failed_transition_no_audio = self._failed_transition_no_audio + ", " + \
                    self._previous_acc_type + " => " + self._acc_type
                self._audio_analyzer_result_save = 3
            elif (audio_analyzer_result_ul == 2) or (audio_analyzer_result_dl == 2):
                self._result_verdict = Global.FAILURE
                self._failed_transition_no_audio = self._failed_transition_no_audio + ", " + \
                    self._previous_acc_type + " => " + self._acc_type
                self._audio_analyzer_result_save = 2
            elif (audio_analyzer_result_ul == 1) or (audio_analyzer_result_dl == 1):
                self._result_verdict = Global.FAILURE
                self._failed_transition_audio_pb = self._failed_transition_audio_pb + ", " + \
                    self._previous_acc_type + " => " + self._acc_type
                self._audio_analyzer_result_save = 1
        else:
            self._audio_analyzer_result_save = 0
            self._result_verdict = Global.SUCCESS

        # Now, the verdict comment will be updated, according to the outcome of the audio test
        if self._audio_analyzer_result_save == 0:
            self._verdict_comment = ("Audio Quality test success: in %s with "
                                     "a %s call with %s accessories sequence"
                                     % (self._signal_tested_direction,
                                        self._call_type,
                                        str(self._tc_parameters.get_param_value("ACC_LIST"))))
        elif self._audio_analyzer_result_save == 1:
            self._verdict_comment = ("Audio Quality test fail: Audio Analyzer quality audio "
                                     "problems detected in %s with a %s call with "
                                     "%s accessories sequence"
                                     % (self._signal_tested_direction,
                                        self._call_type,
                                        self._failed_transition_audio_pb))
        elif self._audio_analyzer_result_save == 2:
            self._verdict_comment = ("Audio Quality test fail: No audio signal in %s with "
                                     "a %s call with %s accessories sequence"
                                     % (self._signal_tested_direction,
                                        self._call_type,
                                        self._failed_transition_no_audio))
        elif self._audio_analyzer_result_save == 3:
            self._verdict_comment = ("Audio Quality test fail: Exception in executable in %s with "
                                     "a %s call with %s accessories sequence"
                                     % (self._signal_tested_direction,
                                        self._call_type,
                                        self._failed_transition_no_audio))
        else:
            self._verdict_comment = ("Audio Quality test fail: Audio Analyzer problems detected "
                                     "in %s with a %s call"
                                     % (self._signal_tested_direction,
                                        self._call_type))