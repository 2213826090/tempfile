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
:summary: This file implements the LIVE DUAL PHONE BT Service Browsing UC
:since: 03/08/2012
:author: cmichelX
"""

import time
from acs_test_scripts.UseCase.LocalConnectivity.LIVE_DUAL_PHONE_BT_BASE import LiveDualPhoneBTBase
from UtilitiesFWK.Utilities import Global
from Device.DeviceManager import DeviceManager
from acs_test_scripts.Device.UECmd.UECmdTypes import BtServiceClass
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LiveDualBTServiceBrowsing(LiveDualPhoneBTBase):

    """
    Live BT Service Browsing test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LiveBTBase init function
        LiveDualPhoneBTBase.__init__(self, tc_name, global_config)

        # Get device_config
        self._dut2_config = DeviceManager().get_device_config("PHONE2")

        # Get TC Parameters
        self._class_list = \
            self._tc_parameters.get_param_value("CLASS_LIST")

        self._initiator = \
            self._tc_parameters.get_param_value("INITIATOR")

        self._initiator_api = None
        self._responder_addr = None
        self._responder_config = None

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """

        LiveDualPhoneBTBase.set_up(self)

        if self._initiator.strip().lower() in "phone1":
            self._initiator_api = self._bt_api
            self._responder_addr = self._phone2_addr
            self._responder_config = self._dut2_config
        elif self._initiator.strip().lower() in "phone2":
            self._initiator_api = self._bt_api2
            self._responder_addr = self._phone1_addr
            self._responder_config = self._dut_config
        else:
            msg = "Initiator value <%s> is not valid" % self._initiator
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        if self._class_list.upper().strip() == "__USE_DEVICE_SERVICE_LIST__":
            # Get default supported service list from device catalog file
            # Take value of responder device
            self._class_list = str(self._responder_config.get("SupportedServiceList"))

        # Start a first BT scan on initiator to list visible remote
        self._initiator_api.bt_scan_devices()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # Call UseCase base run_test function
        LiveDualPhoneBTBase.run_test(self)

        time.sleep(self._wait_btwn_cmd)

        # start to browse device
        self._logger.info("Service Browsing")
        (result, remote) = self._initiator_api.bt_service_browsing(self._responder_addr)

        if self._class_list is "":
            if not result:
                msg = "Service browsing failed"
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        else:
            classlist = self._class_list.strip().split(";")

            for curr_class in classlist:
                # Remove leading and trailing whitespace
                curr_class = curr_class.strip()
                # Search service class in the list
                if BtServiceClass.BT_SERVICE_CLASS[curr_class] not in remote.uuids:
                    msg = "Service %s is not found" % curr_class
                    self._logger.error(msg)
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return Global.SUCCESS, "No errors"
