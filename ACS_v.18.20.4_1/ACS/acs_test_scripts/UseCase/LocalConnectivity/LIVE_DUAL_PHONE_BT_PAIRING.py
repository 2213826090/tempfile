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
:summary: This file implements the LIVE_DUAL_PHONE_BT_PAIRING
:author: npan2
:since:12/10/2011
"""

import time
from acs_test_scripts.UseCase.LocalConnectivity.LIVE_DUAL_PHONE_BT_BASE import LiveDualPhoneBTBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Device.UECmd.UECmdTypes import BT_BOND_STATE, BT_PINVARIANT
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LiveDualPhoneBTPairing(LiveDualPhoneBTBase):

    """
    Live BT Pairing test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LiveBTBase init function
        LiveDualPhoneBTBase.__init__(self, tc_name, global_config)

        # Read INITIATOR_LIST from test case xml file
        self._initiator_list = \
            str(self._tc_parameters.get_param_value("INITIATOR_LIST"))
        # Read REPLY_LIST_1 from test case xml file
        self._reply1_list = \
            str(self._tc_parameters.get_param_value("REPLY_LIST_1"))
        # Read REPLY_LIST_2 from test case xml file
        self._reply2_list = \
            str(self._tc_parameters.get_param_value("REPLY_LIST_2"))

        # Read DISCOVERY_MODE_1 from test case xml file
        self._discov_mode1 = \
            str(self._tc_parameters.get_param_value("DISCOVERY_MODE_1"))
        # Read CODEPIN_1 from test case xml file
        self._pin1 = \
            str(self._tc_parameters.get_param_value("CODEPIN_1"))
        # Read PASSKEY_1 from test case xml file
        self._passkey1 = \
            str(self._tc_parameters.get_param_value("PASSKEY_1"))

        # Read DISCOVERY_MODE_2 from test case xml file
        self._discov_mode2 = \
            str(self._tc_parameters.get_param_value("DISCOVERY_MODE_2"))
        # Read CODEPIN_2 from test case xml file
        self._pin2 = \
            str(self._tc_parameters.get_param_value("CODEPIN_2"))
        # Read PASSKEY_2 from test case xml file
        self._passkey2 = \
            str(self._tc_parameters.get_param_value("PASSKEY_2"))

        # Read DURATION from test case xml file
        self._duration = \
            int(self._tc_parameters.get_param_value("DURATION"))

        self._ini_list = []
        self._reply_list1 = []
        self._reply_list2 = []

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LiveDualPhoneBTBase.set_up(self)

        # Split to create a list of Initiator in case of multiple pairing
        self._ini_list = self._initiator_list.strip().split(",")
        # Split to create a list of phone_1 pairing request reply
        self._reply_list1 = self._reply1_list.strip().split(",")
        # Split to create a list of phone_2 pairing request reply
        self._reply_list2 = self._reply2_list.strip().split(",")

        # Check lists have the same number of iteration
        if (not (range(len(self._ini_list)) == range(len(self._reply_list1))
           == range(len(self._reply_list2)))):
            msg = "INITIATOR_LIST, REPLY_LIST_1 and REPLY_LIST_2"\
                  + " shall have same size"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Change scan mode on both devices
        if self._discov_mode1.upper() == "NOT_DISCOV":
            self._bt_api.set_bt_discoverable("none", 0)
        elif self._discov_mode1.upper() == "DISCOV_PAIR":
            self._bt_api.set_bt_discoverable("connectable", 0)
        elif self._discov_mode1.upper() == "DISCOV_ALL":
            self._bt_api.set_bt_discoverable("both", 0)
        else:
            msg = "Wrong value of DISCOVERY_MODE_1"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        if self._discov_mode2.upper() == "NOT_DISCOV":
            self._bt_api2.set_bt_discoverable("none", 0)
        elif self._discov_mode2.upper() == "DISCOV_PAIR":
            self._bt_api2.set_bt_discoverable("connectable", 0)
        elif self._discov_mode2.upper() == "DISCOV_ALL":
            self._bt_api2.set_bt_discoverable("both", 0)
        else:
            msg = "Wrong value of DISCOVERY_MODE_2"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Start a first BT scan on both devices to list visible remote
        self._bt_api.bt_scan_devices()
        self._bt_api2.bt_scan_devices()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # pylint: disable=E1101

        # Call UseCase base run_test function
        LiveDualPhoneBTBase.run_test(self)

        # Loop on list of initiator
        for iteration in range(len(self._ini_list)):

            # Remove leading and trailing whitespace
            curr_ini = self._ini_list[iteration].strip().lower()

            # Set Initiator and responder of the pairing request
            if curr_ini in "phone1":
                initiator_name = "phone1"
                initiator_api = self._bt_api
                initiator_addr = self._phone1_addr
                initiator_rep = int(self._reply_list1[iteration].strip())
                initiator_pin = self._pin1
                initiator_pass = self._passkey1
                responder_name = "phone2"
                responder_api = self._bt_api2
                responder_addr = self._phone2_addr
                responder_rep = int(self._reply_list2[iteration].strip())
                responder_pin = self._pin2
                responder_pass = self._passkey2

            elif curr_ini in "phone2":
                initiator_name = "phone2"
                initiator_api = self._bt_api2
                initiator_addr = self._phone2_addr
                initiator_rep = int(self._reply_list2[iteration].strip())
                initiator_pin = self._pin2
                initiator_pass = self._passkey2
                responder_name = "phone1"
                responder_api = self._bt_api
                responder_addr = self._phone1_addr
                responder_rep = int(self._reply_list1[iteration].strip())
                responder_pin = self._pin1
                responder_pass = self._passkey1
            else:
                msg = "Initiator value <%s> is not valid" % curr_ini
                self._logger.error(msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

            # Start pairing sequence
            responder_api.wait_for_pairing(initiator_addr, 1,
                                           responder_rep, responder_pin, responder_pass)
            time.sleep(self._wait_btwn_cmd)
            pair_result, pinvar = initiator_api.pair_to_device(responder_addr,
                                                               1, initiator_rep, initiator_pin, initiator_pass)

            if pair_result == BT_BOND_STATE.BOND_NONE:
                msg = "[PHONE1]%s is not paired with %s" \
                    % (initiator_name, responder_name)
                self._logger.info(msg)
            elif pair_result == BT_BOND_STATE.BOND_BONDED:
                msg = "[PHONE1]%s is paired with %s" \
                    % (initiator_name, responder_name)
                self._logger.info(msg)
            else:
                msg = "[PHONE1]Unexpected return value %s" % pair_result
                self._logger.info(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            # In case of success pairing, wait for DURATION and
            # check devices are still paired
            if pair_result == BT_BOND_STATE.BOND_BONDED:
                for i in range(self._duration):
                    if not i % 10:
                        msg = "[PHONE1]Wait for %d seconds" % (self._duration - i)
                        self._logger.info(msg)
                    time.sleep(1)

                # Retrieve list of paired devices
                pairedflag = False
                list_paired_devices = self._bt_api.list_paired_device()
                for element in list_paired_devices:
                    if str(element.address).upper() \
                            == str(self._phone2_addr).upper():
                        self._logger.info("Still paired with device %s after %s s"
                                          % (self._phone2_addr, self._duration))
                        pairedflag = True
                        break

                if not pairedflag:
                    msg = "Pairing to device %s fail" % self._phone2_addr
                    self._logger.info(msg)
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            # analyze of pairing state for final verdict
            if not self._is_verdict_pass(pair_result, pinvar, iteration):
                msg = "[PHONE1]Unexpected pairing verdict %s" % pair_result
                self._logger.info(msg)
                self._display_param(pair_result, pinvar, iteration, "info")
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
            else:
                msg = "[PHONE1]pairing Verdict is PASS"
                self._logger.info(msg)
                self._display_param(pair_result, pinvar, iteration, "debug")

            # If Responder is NOT discoverable
            # Stop "WaitForPair" activity on Responder
            self._stop_remaining_activity(iteration, responder_api)

        # after end of loop, Remove pairing
        time.sleep(self._wait_btwn_cmd)
        list_paired_devices = self._bt_api.list_paired_device()
        for element in list_paired_devices:
            if str(element.address).upper() == str(self._phone2_addr).upper():
                self._logger.info("Unpair both devices")
                self._bt_api.unpair_bt_device(self._phone2_addr)
                self._bt_api2.unpair_bt_device(self._phone1_addr)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def _is_verdict_pass(self, pair_result, pinvar, iteration):
        """
        Compute parameter to determine if devices should be paired or not
        Depend on:
          1. both discovery mode
          2. both reply
          3. pincode on both devices
          4. passkey on both devices
          5. pin variant used to pair

        Final state is BONDED if:
          responder discovery mode in [DISCOV_PAIR, DISCOV_ALL]
          AND both reply = 1
          AND
            if (pinvar = PIN), then both pincode shall be identical
            if (pinvar = PASSKEY), then both passkey shall be identical
            if (pinvar in (REPLY, NONE), then pincode and passkey don't care
            if (pinvar = ERROR), then Verdict FAIL

        Otherwise state is NOT BONDED

        Return True if Final verdict is PASS
              otherwise False
        """

        if self._ini_list[iteration].strip().lower() == "phone1":
            discovmode = self._discov_mode2.upper()
        elif self._ini_list[iteration].strip().lower() == "phone2":
            discovmode = self._discov_mode1.upper()
        else:
            msg = "Initiator value <%s> is not valid" \
                % self._ini_list[iteration].strip()
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # pylint: disable=E1101
        if ((discovmode in ("DISCOV_PAIR", "DISCOV_ALL"))
            and (int(self._reply_list1[iteration].strip()) == 1)
                and (int(self._reply_list2[iteration].strip()) == 1)):

            if pinvar == BT_PINVARIANT.PIN:
                if self._pin1 == self._pin2:
                    final_state = BT_BOND_STATE.BOND_BONDED
                else:
                    final_state = BT_BOND_STATE.BOND_NONE
            elif pinvar == BT_PINVARIANT.PASSKEY:
                if self._passkey1 == self._passkey2:
                    final_state = BT_BOND_STATE.BOND_BONDED
                else:
                    final_state = BT_BOND_STATE.BOND_NONE
            elif (pinvar in (BT_PINVARIANT.REPLY,
                             BT_PINVARIANT.NONE)):
                final_state = BT_BOND_STATE.BOND_BONDED
            else:
                return False
        else:
            final_state = BT_BOND_STATE.BOND_NONE

        # Compare Final pairing State with the actual to give the Verdict
        # If equal then verdict is PASS (True)
        # Otherwise FAIL (False)
        if final_state == pair_result:
            return True
        else:
            return False

#------------------------------------------------------------------------------

    def _display_param(self, pair_result, pinvar, iteration, logtyp="info"):
        """
        Function to display in ACS log Pairing details as:
          1. pair_result: pairing result (bonded or not)
          2. pin variant used
          3. which iteration inside run phase

        There are 2 type of display: info or debug

        Return None
        """

        if logtyp == "info":
            self._logger.info("Pairing result = %s" % pair_result)
            self._logger.info("Pairing pin variant = %s" % pinvar)
            self._logger.info("[PHONE1]Parameters are")
            msg = "[PHONE1]Discover mode = %s" % self._discov_mode1
            self._logger.info(msg)
            msg = "[PHONE1]Reply = %s" % int(self._reply_list1[iteration].strip())
            self._logger.info(msg)
            msg = "[PHONE1]pincode = %s" % self._pin1
            self._logger.info(msg)
            msg = "[PHONE1]passkey = %s" % self._passkey1
            self._logger.info(msg)

            self._logger.info("[PHONE2]Parameters are")
            msg = "[PHONE2]Discover mode = %s" % self._discov_mode2
            self._logger.info(msg)
            msg = "[PHONE2]Reply = %s" % int(self._reply_list2[iteration].strip())
            self._logger.info(msg)
            msg = "[PHONE2]pincode = %s" % self._pin2
            self._logger.info(msg)
            msg = "[PHONE2]passkey = %s" % self._passkey2
            self._logger.info(msg)
        elif logtyp == "debug":
            self._logger.debug("Pairing result = %s" % pair_result)
            self._logger.debug("Pairing pin variant = %s" % pinvar)
            self._logger.debug("[PHONE1]Parameters are")
            msg = "[PHONE1]Discover mode = %s" % self._discov_mode1
            self._logger.debug(msg)
            msg = "[PHONE1]Reply = %s" % int(self._reply_list1[iteration].strip())
            self._logger.debug(msg)
            msg = "[PHONE1]pincode = %s" % self._pin1
            self._logger.debug(msg)
            msg = "[PHONE1]passkey = %s" % self._passkey1
            self._logger.debug(msg)

            self._logger.debug("[PHONE2]Parameters are")
            msg = "[PHONE2]Discover mode = %s" % self._discov_mode2
            self._logger.debug(msg)
            msg = "[PHONE2]Reply = %s" % int(self._reply_list2[iteration].strip())
            self._logger.debug(msg)
            msg = "[PHONE2]pincode = %s" % self._pin2
            self._logger.debug(msg)
            msg = "[PHONE2]passkey = %s" % self._passkey2
            self._logger.debug(msg)
        else:
            return

#------------------------------------------------------------------------------

    def _stop_remaining_activity(self, iteration, responder_api):
        """
        If Responder is NOT discoverable
            Stop "wait_for_pair" activity on Responder

        Return None
        """

        if self._ini_list[iteration].strip().lower() == "phone1":
            discovmode = self._discov_mode2.upper()
        elif self._ini_list[iteration].strip().lower() == "phone2":
            discovmode = self._discov_mode1.upper()
        else:
            msg = "Initiator value <%s> is not valid" % self._ini_list[iteration].strip()
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        if discovmode == "NOT_DISCOV":
            if not responder_api.wait_for_pairing_canceled():
                msg = "Pairing canceled status not received"
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
            return
