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
:summary: This file implements the APx Calibration for VOIP call
:since: 16/07/2013
:author: nprecigx
"""

import time
from UtilitiesFWK.Utilities import Global, str_to_bool
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from Device.DeviceManager import DeviceManager
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.TestEquipmentException import TestEquipmentException
from ErrorHandling.DeviceException import DeviceException

# pylint: disable=E1101


class LabAudioAPxCalibrationVOIP(UseCaseBase):
    """
    AudioComms calibration APx class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call UseCaseBase Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # Create APx
        self._audio_analyzer = self._em.get_audio_analyzer("AudioAnalyzer")

        # Check if IO card is used
        self._use_io_card = str_to_bool(global_config.campaignConfig.get("isIoCardUsed"))

        # Create Whs
        if self._use_io_card:
            self._wired_headset = self._em.get_wired_headset("WiredHeadset")

        # Initialization of the test type
        self._test_type = "calibration"

        # Initialization of test result in uplink
        self._result_verdict_ul = Global.FAILURE
        # Initialization of test result in downlink
        self._result_verdict_dl = Global.FAILURE
        # Initialization of test result
        self._result_verdict_final = Global.FAILURE

        # Call Stream Volume
        self._call_stream_volume = 70

        # Initialization of the verdict comment
        self._verdict_comment = "APx Calibration fail"

        # Read registrationTimeout from Device_Catalog.xml
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

        # Read callSetupTimeout from Device_Catalog.xml
        self._call_setup_time = \
            int(self._dut_config.get("callSetupTimeout"))

        # Read accessories type from test case xml file (str)
        self._acc_type = str(self._tc_parameters.get_param_value("ACC_TYPE"))

        # Call Stream Volume in percent
        self._call_stream_volume_dut = \
            int(self._tc_parameters.get_param_value("CALL_VOLUME_DUT"))

        # Call Stream Volume in percent
        self._call_stream_volume_ref = \
            int(self._tc_parameters.get_param_value("CALL_VOLUME_REF"))

        # Read Audio Analyzer parameters from bench config xml file (str)
        self._audio_analyser = global_config.benchConfig.\
            get_parameters("AudioAnalyzer")
        self._bt_remote_addr = self._audio_analyser.get_param_value("Bt_mac_addr")

        # Read dut sip address from test case xml file (str)
        self._dut_sip_address = \
            str(self._tc_parameters.get_param_value("DUT_SIP_ADDRESS"))

        # Read ref phone sip address from test case xml file (str)
        self._peer_sip_address = \
            str(self._tc_parameters.get_param_value("PEER_SIP_ADDRESS"))

        # Instantiate generic UECmd for voiceCall Ucs
        self._networking_api = self._device.get_uecmd("Networking")
        self._sip_call_api = self._device.get_uecmd("SipCall")
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self._system_api = self._device.get_uecmd("System")

        if self._acc_type.find("BLUETOOTH") != -1:
            # Get UECmdLayer
            self._bt_api = self._device.get_uecmd("LocalConnectivity")
            # Get dut bt address
            self._dut_bt_address = self._bt_api.get_bt_adapter_address()
        else:
            self._dut_bt_address = "None"

        # Load instance of the PHONE2
        self._phone2 = DeviceManager().get_device("PHONE2")

        if self._phone2 is not None:
            if not self._phone2.is_available():
                self._phone2.switch_on(simple_switch_mode=True)
            self._voice_call_api2 = self._phone2.get_uecmd("VoiceCall")
            self._system_api2 = self._phone2.get_uecmd("System")

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
        # Call UseCaseBase Setup function
        UseCaseBase.set_up(self)

        if self._phone2 is not None:
            self._networking_api2 = self._phone2.get_uecmd("Networking")
            self._sip_call_api2 = self._phone2.get_uecmd("SipCall")
        else:
            error_msg = \
                "This test case requires two phones to be executed !"
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, error_msg)

        # APx Initialization
        if self._audio_analyzer.initialization(self._device.get_phone_model(), self._test_type) != 0:
            error_msg = \
                "APX Initialization Failed !"
            self._logger.error(error_msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, error_msg)

        # Setup phone type ( calling, receiving, releasing phone)
        self._phone_calling = self._sip_call_api
        self._phone_receiving = self._sip_call_api2
        self._calling_phone_number = self._peer_sip_address
        self._phone_releasing = self._sip_call_api2

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
        # Call UseCaseBase run_test function
        UseCaseBase.run_test(self)

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

        # Configure Audio output to acc_type given with the test_case except in Bluetooth test case
        if self._acc_type.find("BLUETOOTH") != -1:
            if self._audio_analyzer.connect_bt(self._acc_type, self._dut_bt_address) != 0:
                error_msg = \
                    "Connect Bluetooth failed !"
                self._logger.error(error_msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, error_msg)

        # Configure Audio output to acc_type given with the test_case
        if self._acc_type == "EARPIECE":
            self._sip_call_api.switch_to_earpiece()
            if self._use_io_card:
                self._wired_headset.unplug_whs()
                self._wired_headset.unplug_headphone()
            time.sleep(self._wait_btwn_cmd)
        elif self._acc_type == "HEADSET":
            self._sip_call_api.switch_to_earpiece()
            if self._use_io_card:
                self._wired_headset.unplug_headphone()
                self._wired_headset.plug_whs()
            time.sleep(self._wait_btwn_cmd)
        elif self._acc_type == "HEADPHONE":
            self._sip_call_api.switch_to_earpiece()
            if self._use_io_card:
                self._wired_headset.unplug_whs()
                self._wired_headset.plug_headphone()
            time.sleep(self._wait_btwn_cmd)
        elif self._acc_type == "SPEAKER":
            self._sip_call_api.switch_to_speaker()
            time.sleep(self._wait_btwn_cmd)
        elif self._acc_type == "BLUETOOTHHSP" or self._acc_type == "BLUETOOTHHFP":
            self._sip_call_api.switch_to_bluetooth()
        elif self._acc_type == "BLUETOOTHA2DP":
            self._sip_call_api.switch_to_bluetooth()
        else:
            self._logger.error("Unknown accessories")
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Unknown accessories")

        time.sleep(self._wait_btwn_cmd)

        if self._acc_type.find("BLUETOOTH") == -1:
            # Set Voice Call Volume
            self._system_api.adjust_specified_stream_volume("VoiceCall",
                                                            self._call_stream_volume_dut)
        else:
            # Set Voice Call Volume
            self._system_api.adjust_specified_stream_volume("Bluetooth",
                                                            self._call_stream_volume_dut)

        self._system_api2.adjust_specified_stream_volume("VoiceCall",
                                                         self._call_stream_volume_ref)
        # Launch Apx Calibration test on Uplink
        apx_result = self._audio_analyzer.calibration(self._device.get_phone_model(),
                                                      "VOIP",
                                                      self._acc_type,
                                                      "UL",
                                                      self._dut_bt_address)

        if apx_result == 0:
            self._result_verdict_ul = Global.SUCCESS
            self._verdict_comment = "APx Calibration success in UL"
        else:
            self._result_verdict_ul = Global.FAILURE
            self._verdict_comment = "APx Calibration fail in UL"

        # Launch Apx Calibration test on Downlink
        apx_result = self._audio_analyzer.calibration(self._device.get_phone_model(),
                                                      "VOIP",
                                                      self._acc_type,
                                                      "DL",
                                                      self._dut_bt_address)

        if apx_result == 0:
            self._result_verdict_dl = Global.SUCCESS
            self._verdict_comment += " and APx Calibration success in DL"
        else:
            self._result_verdict_dl = Global.FAILURE
            self._verdict_comment += " and APx Calibration fail in DL"

        # Compute final verdict
        if self._result_verdict_ul == Global.FAILURE or \
           self._result_verdict_dl == Global.FAILURE:
            self._result_verdict_final = Global.FAILURE
        else:
            self._result_verdict_final = Global.SUCCESS

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

        return self._result_verdict_final, self._verdict_comment

    def tear_down(self):
        """
        End and dispose the test
        """

        # Call LabAudioQualityBase run_test function
        UseCaseBase.tear_down(self)

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

        return Global.SUCCESS, "No errors"
