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
:summary: This file implements the LIVE_DUAL_PHONE_BT_CONNECT
:author: npan2
:since:24/10/2011
"""

import time
from LIVE_DUAL_PHONE_BT_BASE import LiveDualPhoneBTBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LiveDualPhoneBTConnect(LiveDualPhoneBTBase):

    """
    Live Dual phone BT Connect, and send  message
    """

    STR_SEND = "send"
    STR_RECEIVE = "receive"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LiveBTBase init function
        LiveDualPhoneBTBase.__init__(self, tc_name, global_config)

        # Read CODE_PIN from test case xml file
        self._code_pin = \
            str(self._tc_parameters.get_param_value("CODE_PIN"))
        # Read PASSPHRASE from test case xml file
        self._passphrase = \
            str(self._tc_parameters.get_param_value("PASSPHRASE"))
        # Read MESSAGE from test case xml file
        self._message = \
            str(self._tc_parameters.get_param_value("MESSAGE"))
        # Read DURATION from test case xml file
        self._direction = \
            str(self._tc_parameters.get_param_value("DIRECTION")).lower()

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """

        LiveDualPhoneBTBase.set_up(self)

        if self._direction not in [self.STR_SEND, self.STR_RECEIVE]:
            msg = "wrong value of parameter DIRECTION"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Set PHONE2 Discoverable by PHONE1
        self._bt_api2.set_bt_discoverable("both", 0)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # Call UseCase base run_test function
        LiveDualPhoneBTBase.run_test(self)

        # pair two phones
        device_found = False

        # Prepare remote device to respond to pairing request
        self._bt_api2.wait_for_pairing(self._phone1_addr, 1, 1)
        time.sleep(self._wait_btwn_cmd)

        # pair two devices
        self._bt_api.pair_to_device(self._phone2_addr, 1)
        time.sleep(self._wait_btwn_cmd)

        list_paired_devices = \
            self._bt_api.list_paired_device()
        for element in list_paired_devices:
            if str(element.address).upper() == str(self._phone2_addr).upper():
                device_found = True

        if not device_found:
            msg = "Connect to device %s failed" % self._phone2_addr
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # connect and send message
        self._logger.info("Message to send is : " + self._message)
        time.sleep(self._wait_btwn_cmd)
        if self._direction == self.STR_SEND:
            self._logger.info("Start receiving msg")
            self._bt_api2.bt_receive_msg()
            time.sleep(self._wait_btwn_cmd)
            self._logger.info("Start sending msg")
            self._bt_api.bt_send_msg(self._phone2_addr, self._message)
        elif self._direction == self.STR_RECEIVE:
            self._logger.info("Start receiving msg")
            self._bt_api.bt_receive_msg()
            time.sleep(self._wait_btwn_cmd)
            self._logger.info("Start sending msg")
            self._bt_api2.bt_send_msg(self._phone1_addr, self._message)

        # check message
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Start checking msg")

        if self._direction == self.STR_SEND:
            self._bt_api2.bt_check_msg(self._message)
        else:
            self._bt_api.bt_check_msg(self._message)

        self._logger.info("Message received successfully")

        # Unpair both devices
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Unpair both devices")
        self._bt_api.unpair_bt_device(self._phone2_addr)
        self._bt_api2.unpair_bt_device(self._phone1_addr)
        time.sleep(self._wait_btwn_cmd)

        return (Global.SUCCESS,
                "Pairing to device %s success" % self._phone2_addr)
