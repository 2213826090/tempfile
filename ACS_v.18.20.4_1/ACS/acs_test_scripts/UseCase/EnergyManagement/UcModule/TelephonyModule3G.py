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
:summary: This module represent telephony module for 3G, a module is the combination of uecmds and equipment used.
:author: vgombert
:since: July 08 2013
"""
import time

import acs_test_scripts.Device.UECmd.UECmdTypes as UECmdTypes
from acs_test_scripts.UseCase.EnergyManagement.UcModule.OverMind import OverMind
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT


class TelephonyModule3g():
    """
    init.
    """
    __LOG_TAG = "[3GMODULE]\t"

    def __init__(self, simulator_name="NETWORK_SIMULATOR1"):
        """
        describe basic parameters to set up simulator in order to register to a network and do a circuit call.
        only support 1 equipment setting for now
        """
        #-----------------------------------------------------------------------
        overmind = OverMind()
        self.__logger = LOGGER_TEST_SCRIPT
        self.__logger.info(self.__LOG_TAG + "INIT")
        device = overmind.get_instance(overmind.DEVICE)
        tc_params = overmind.get_instance(overmind.TC_PARAMETERS)
        bench_config = overmind.get_instance(overmind.BENCH_CONFIG)
        dut_config = overmind.get_instance(overmind.DUT_CONFIGURATION)
        eq_manager = overmind.get_instance(overmind.EQUIPMENT_MANAGER)

        network = bench_config.get_parameters("CELLULAR_NETWORK")
        self.__apn = network.get_param_value("APN")
        self.__ssid = network.get_param_value("SSID")

        # band selection
        self.__cell_band = tc_params.get_param_value("CELL_BAND")
        # cell service like WCDMA
        self.__cell_service = tc_params.get_param_value("CELL_SERVICE")
        # Read TCH_ARFCN from test case xml file
        self.__ul_uarfcn = tc_params.get_param_value("UL_UARFCN", default_cast_type=int)
        # cell power ,higher value mean better registration
        self.__cell_power = tc_params.get_param_value("CELL_POWER", default_cast_type=int)
        # who do the call ? simulator or phone
        self.__call_origin = tc_params.get_param_value("CALL_ORIGIN", "PHONE")
        # number to call
        self.__phone_number = tc_params.get_param_value("PHONE_NUMBER", "OOOO12121121")
        # codec for voicecall
        self.__voice_coder_rate = tc_params.get_param_value("VOICE_CODER_RATE")
        # registration timeout from device catalog
        self.__registration_timeout = int(dut_config.get("registrationTimeout"))

        # Create cellular network simulator and retrieve 3G voice call and data interfaces
        self.__ns = eq_manager.get_cellular_network_simulator(simulator_name)
        self.__ns_cell_3g = self.__ns.get_cell_3g()
        self.__ns_voice_call_3g = self.__ns_cell_3g.get_voice_call()
        self.__ns_data_3g = self.__ns_cell_3g.get_data()

        self.__voicecall_api = device.get_uecmd("VoiceCall", True)
        self.__modem_api = device.get_uecmd("Modem", True)
        self.__networking_api = device.get_uecmd("Networking", True)

        # persistent tag avoid doing some persistent operation during b2b
        self.__eq_setup_done = False
        self.__board_setup_done = False

    def release_eq(self):
        """
        Release equipment used.
        """
        # Set cell off
        if self.__ns_cell_3g is not None:
            self.__ns_cell_3g.set_cell_off()

        # DisConnect from equipment
        if self.__ns is not None:
            self.__ns.release()

#----------------------------------------------------------------------

    def set_up_eq(self):
        """
        Initialize the simulator
        firstly turn it off during setup.
        does not turn it on
        """
        # Configure CMU
        # Connect to cellular network simulator
        self.__ns.init()
        if not self.__eq_setup_done:
            self.__ns.switch_app_format("WCDMA")
            # Perform Full Preset
            self.__ns.perform_full_preset()
            # Set cell off
            self.__ns_cell_3g.set_cell_off()
            # Set cell band using CELL_BAND parameter
            self.__ns_cell_3g.set_band(self.__cell_band)
            # Set cell service using CELL_SERVICE parameter
            self.__ns_cell_3g.set_cell_service(self.__cell_service)
            # Set Traffic Channel Arfcn using TCH_ARFCN parameter
            self.__ns_cell_3g.set_uplink_arfcn(self.__ul_uarfcn)
            # Set cell power using CELL_POWER parameter
            self.__ns_cell_3g.set_cell_power(self.__cell_power)
            # Set Audio codec using VOICE_CODER_RATE parameter
            if self.__voice_coder_rate is not None:
                self.__ns_voice_call_3g.set_audio_codec(self.__voice_coder_rate)

            self.__eq_setup_done = True

    def establish_call(self, timeout=120, robustness=False):
        """
        establish call depending of the choice you made in this class init.
        Not that you need to do a registration first.
        """
        if robustness:
            # Release any previous call (Robustness)
            self.__voicecall_api.release()
        elif self.__voicecall_api.get_state() == UECmdTypes.VOICE_CALL_STATE.ACTIVE:  # pylint: disable=E1101
            self.__logger.info(self.__LOG_TAG + "active call detected, skip establishing a new call")
            return
        #             CALL DONE BY PHONE
        if self.__call_origin == "PHONE":
            # Dial using PHONE_NUMBER parameter
            self.__voicecall_api.dial(self.__phone_number)

            # Check call state "CONNECTED" before callSetupTimeout seconds
            self.__ns_voice_call_3g.check_call_connected(timeout,
                                                        blocking=False)
            # Wait for state "active" before callSetupTimeout seconds
            # pylint: disable=E1101
            self.__voicecall_api.wait_for_state(
                UECmdTypes.VOICE_CALL_STATE.ACTIVE, timeout)
        #             CALL DONE BY SIMULATOR
        elif self.__call_origin == "SIMULATOR":
            # Mobile Terminated originate call
            self.__ns_voice_call_3g.mt_originate_call()

            # pylint: disable=E1101
            # Wait for state "incoming before callSetupTimeout seconds
            stop_time = timeout + time.time()
            while self.__voicecall_api.get_state() != UECmdTypes.VOICE_CALL_STATE.INCOMING:
                if time.time() > stop_time:
                    break

            # Answer call
            self.__voicecall_api.answer()

            if robustness:
                # Check call state "CONNECTED" before callSetupTimeout seconds
                self._ns_voice_call_3g.check_call_connected(timeout, blocking=False)

                # Wait for state "active" before callSetupTimeout seconds
                self.__voicecall_api.wait_for_state(
                    UECmdTypes.VOICE_CALL_STATE.ACTIVE, timeout)

    def register_to_network(self):
        """
        force a registration to the network.
        Turning on cell at the same time
        """
        # Set cell on
        self.__ns_cell_3g.set_cell_on()
        if not  self.__board_setup_done:
        # Adapt attachment procedure to CIRCUIT or PACKET
            if "CIRCUIT" not in self.__cell_service:
                # Set the APN
                self._logger.info("Setting APN " + str(self.__apn) + "...")
                self._networking_api.set_apn(self.__ssid, self.__apn)
                # Activate PDP context
                self._logger.info("Active PDP Context...")
                self._networking_api.activate_pdp_context(self._ssid, check=False)
            self.__board_setup_done = True

        # check registration from board
        registration_state = ["roaming", "registered"]
        self.__modem_api.check_cdk_state_bfor_timeout(registration_state, self.__registration_timeout)

    def is_call_active(self):
        """
        check call state depending of the choice you made in this class init

        :rtype: boolean
        :return: True if call is on going, False otherwise
        """
        # get the call state
        result = False
        # Check cs call state
        call_state_tmp = self.__voicecall_api.get_state()
        # pylint: disable=E1101
        if call_state_tmp == UECmdTypes.VOICE_CALL_STATE.ACTIVE:
            result = True

        return result
