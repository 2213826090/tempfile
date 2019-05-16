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
:summary: This file implements the AudioComms Audio Quality Base class
:since: 06/03/2013
:author: nprecigx
"""

import time
from UtilitiesFWK.Utilities import Global, str_to_bool
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil
from Device.DeviceManager import DeviceManager
from ErrorHandling.TestEquipmentException import TestEquipmentException
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Device.UECmd.UECmdTypes import PreferredNetwork

# pylint: disable=E1101


class LabAudioQualityBase(UseCaseBase):
    """
    AudioComms Audio CSV Call class.
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
        if self._use_io_card and global_config.benchConfig.has_parameter("WiredHeadset"):
            self._wired_headset = self._em.get_wired_headset("WiredHeadset")

        # Initialization of test result
        self._result_verdict = Global.BLOCKED

        # Initialization of the verdict comment
        self._verdict_comment = "Audio Quality test fail"

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
        self._audio_analyzer_node = global_config.benchConfig. \
            get_parameters("AudioAnalyzer")

        # Initialization of the test type
        if self._audio_analyzer_node.get_param_value("Model") in "APx585":
            self._test_type = "audioquality"
        elif self._audio_analyzer_node.get_param_value("Model") in "RS_UPV":
            self._test_type = self._name
        else:
            error_msg = \
                "Audio Analyzer model not supported or incorrect"
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, error_msg)

        # Instantiate generic UECmd for voiceCall Ucs
        self._phonemodem_api = self._device.get_uecmd("Modem")
        self._networking_api = self._device.get_uecmd("Networking")
        self._system_api = self._device.get_uecmd("System")
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self._voice_call_api = self._device.get_uecmd("VoiceCall")

        if self._acc_type.find("BLUETOOTH") != -1:
            # Get UECmdLayer
            self._bt_api = self._device.get_uecmd("LocalConnectivity")

            # Get BT device MAC address
            self._bt_remote_addr = self._audio_analyzer_node.get_param_value("Bt_mac_addr")

        # Load instance of the PHONE2
        self._phone2 = DeviceManager().get_device("PHONE2")

        # Instantiate system UECmd for Phone2
        self._system_api2 = None

        # Time elapsed during voice call
        self._elapsed_time = 0

        # Boolean determining whether a network simulator will be used
        self._is_network_simulator_used = False

        # Time to wait between 2 successive measurement
        self._wait_between_measure = 0

        if "NETWORK_SIMULATOR1" in global_config.benchConfig.get_parameters_name():
            self._is_network_simulator_used = True
            # Read CELLULAR_NETWORK parameters from BenchConfig.xml
            self._network = \
                global_config.benchConfig.get_parameters("CELLULAR_NETWORK")
            self._apn = self._network.get_param_value("APN")
            self._ssid = self._network.get_param_value("SSID")
            self._ns_node = global_config.benchConfig.get_parameters(
                "NETWORK_SIMULATOR1")
            self._ns = self._em.get_cellular_network_simulator("NETWORK_SIMULATOR1")

            if "2G" in self._call_type:
                self._ns_cell = self._ns.get_cell_2g()
            elif "3G" in self._call_type:
                self._ns_cell = self._ns.get_cell_3g()
            elif "4G" in self._call_type:
                self._ns_cell = self._ns.get_cell_4g()
            else:
                msg = "Wrong value for parameter CALL_TYPE (Must be one of the following: 2G, 3G, 4G)"
                self._logger.error(msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

            self._analog_input_peak_level = self._ns_node.get_param_value("AnalogInputPeakLevel")
            self._analog_output_peak_level = self._ns_node.get_param_value("AnalogOutputPeakLevel")

            self._vc = self._ns_cell.get_voice_call()

            # Read call parameters from test case xml file
            self._cell_band = \
                int(self._tc_parameters.get_param_value("CELL_BAND", 1))

            self._codec = str(self._tc_parameters.get_param_value("CODEC"))

            # In case of long lasting call, update the waiting time between measurements
            if self._call_duration > 60:
                # Read time between 2 successive measurement
                self._wait_between_measure = int(self._tc_parameters.get_param_value("WAIT_BETWEEN_MEASURE"))

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

        # Audio Analyzer Initialization
        if self._audio_analyzer.initialization(self._device.get_phone_model(), self._test_type) != 0:
            error_msg = \
                "Audio Analyzer Initialization Failed !"
            self._logger.error(error_msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, error_msg)

        if self._acc_type.find("BLUETOOTH") != -1:
            # Get dut bt address
            self._dut_bt_address = self._bt_api.get_bt_adapter_address()

        if self._is_network_simulator_used:
            # Perform network configuration
            # Connect to equipment
            self._ns.init()

            # Perform a full preset
            self._ns.perform_full_preset()

            # Flight mode activation
            self._networking_api.set_flight_mode("on")

            # Set cell off
            self._ns_cell.set_cell_off()

            if self._call_type in "2G" or self._call_type in "3G":
                # Set the APN
                time.sleep(self._wait_btwn_cmd)
                self._logger.info("Setting APN " + str(self._apn) + "...")
                self._networking_api.set_apn(self._ssid, self._apn)

            elif self._call_type in "4G":
                self._logger.info("Setting APN for IMS " + str(self._apn) + " on protocol IPV4 ")
                # APN for AP centric
                self._networking_api.set_apn(self._ssid, self._apn, None, None, "IP", None, "default")
                # APN for BP centric
                self._networking_api.set_apn("ims", "ims", None, None, "IP", None, "ims", False, False)

                # Load cell configuration
                self._ns.load_cell_config("COMMON_LTE")
                self._ns.load_cell_config(self._cell_band)

                # Load IMS server configuration
                self._ns.load_cell_config("COMMON_IMS")

                self._ns_cell.set_ims_on()

                time.sleep(10)
            else:
                msg = "Wrong value for parameter CALL_TYPE (Must be one of the following: 2G, 3G, 4G)"
                self._logger.error(msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

            # Perform audio configuration
            self._vc.set_audio_codec(self._codec)
            self._vc.set_speech_configuration("AUDioboard")
            self._vc.calibrate_analog_audio_level(self._analog_input_peak_level,
                                                  self._analog_output_peak_level)

            if self._call_type in "4G":
                # Set DAU on
                self._ns_cell.get_data().set_epc_on()
                self._ns_cell.set_ims_on()

            # Set Cell on
            self._ns_cell.set_cell_on()

            # Phone has to see the cell off!
            self._phonemodem_api.check_cdk_no_registration_bfor_timeout(self._registration_timeout)

            # Flight mode deactivation
            self._networking_api.set_flight_mode("off")

            time.sleep(self._wait_btwn_cmd)

            # Check registration state is connected using
            # registrationTimeout from Device_Catalog.xml
            self._logger.info("Check registration state is connected")
            self._phonemodem_api.check_cdk_registration_bfor_timeout(self._registration_timeout)

            if self._call_type in "4G":
                # Check IMS connection state
                self._ns_cell.get_data().check_ims_connection_state("REG", self._registration_timeout)

            time.sleep(self._wait_btwn_cmd)

        else:

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

        # Connect Bluetooth device
        if self._acc_type.find("BLUETOOTH") != -1:
            if self._audio_analyzer.connect_bt(self._acc_type, self._dut_bt_address) != 0:
                error_msg = \
                    "Connect Bluetooth failed !"
                self._logger.error(error_msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, error_msg)

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

    def __compute_test_verdict__(self, apx_result):
        """
        Compute the result verdict and the verdict comment

        :type apx_result: int
        :param apx_result: the apx585 return code return by run command
        """

        if (apx_result == 0):
            self._result_verdict = Global.SUCCESS
            self._verdict_comment = ("Audio Quality test success: in %s with "
                                     "a %s call and %s accessories"
                                     % (self._signal_tested_direction,
                                        self._call_type,
                                        self._acc_type))
        elif (apx_result == 1):
            self._result_verdict = Global.FAILURE
            self._verdict_comment = ("Audio Quality test fail: APx585 quality audio "
                                     "problems detected in %s with a %s call and "
                                     "%s accessories"
                                     % (self._signal_tested_direction,
                                        self._call_type,
                                        self._acc_type))
        elif (apx_result == 2):
            self._result_verdict = Global.FAILURE
            self._verdict_comment = ("Audio Quality test fail: No audio signal in %s with "
                                     "a %s call and %s accessories"
                                     % (self._signal_tested_direction,
                                        self._call_type,
                                        self._acc_type))
        elif (apx_result == 3):
            self._result_verdict = Global.BLOCKED
            self._verdict_comment = ("Audio Quality test fail: Apx585 executable exception occurred")
        else:
            self._result_verdict = Global.BLOCKED
            self._verdict_comment = ("Audio Quality test fail: APx585 problems detected "
                                     "in %s with a %s call and %s accessories"
                                     % (self._signal_tested_direction,
                                        self._call_type,
                                        self._acc_type))

    def _get_elapsed_time(self, time_stamp_start, time_stamp_current):
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
