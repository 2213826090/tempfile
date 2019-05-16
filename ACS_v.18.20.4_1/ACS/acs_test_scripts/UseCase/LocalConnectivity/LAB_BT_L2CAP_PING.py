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
:summary: This file implements the LIVE BT Pairing UC
:author: ssavrimoutou
:since:30/09/2010
"""

import time

from LIVE_BT_BASE import LiveBTBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase


class LabBTL2capPing(LiveBTBase):

    """
    Live BT openclose test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LiveBTBase init function
        LiveBTBase.__init__(self, tc_name, global_config)

        # Read LINUX_DESKTOP_IP_ADDRESS from test case xml file
        self._linux_desktop_ip_address = \
            str(self._tc_parameters.get_param_value("LINUX_DESKTOP_IP_ADDRESS"))

        # Read USER_NAME from test case xml file
        self._linux_desktop_user_name = \
            str(self._tc_parameters.get_param_value("USER_NAME"))

        # Read USER_PASSWORD from test case xml file
        self._linux_desktop_user_password = \
            str(self._tc_parameters.get_param_value("USER_PASSWORD"))

        # Read CODE_PIN from test case xml file
        self._code_pin = \
            str(self._tc_parameters.get_param_value("CODE_PIN"))

        # Read PASSPHRASE from test case xml file
        self._passphrase = \
            str(self._tc_parameters.get_param_value("PASSPHRASE"))

        # Read DEVICE_BT_ADDRESS from test case xml file
        self._device_bt_address = \
            str(self._tc_parameters.get_param_value("DEVICE_BT_ADDRESS"))

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """

        # Call UseCase base set_up function
        # UseCaseBase.set_up(self)
        if self._bt_api.get_bt_power_status():
            self._logger.info("bt is open , close it first .")
            self._bt_api.set_bt_power("off")
        UseCaseBase.set_up(self)

        self._lds_ssh.connect_with_socket("10.239.32.45", "acs", "acs")  # pylint: disable=E1101

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # Call UseCase base run_test function
        LiveBTBase.run_test(self)

        # Init verdict value
        self._error.Code = Global.SUCCESS
        self._error.Msg = \
            "open and close bt success"

        # open and close excute.
        time.sleep(self._wait_btwn_cmd)

        self._bt_api.set_bt_power("on")
        time.sleep(self._wait_btwn_cmd)
        self._logger.info(
            "open bt done here !")
        self._lds_ssh.execute_remote_command("cd /home/acs")  # pylint: disable=E1101
        self._lds_ssh.execute_remote_command("ls ")  # pylint: disable=E1101
        time.sleep(self._wait_btwn_cmd)
        time.sleep(self._wait_btwn_cmd)
        return self._error.Code, self._error.Msg
#------------------------------------------------------------------------------

    def tear_down(self):
        """
        Initialize the test
        """
        self._lds_ssh.disconnect_with_socket()  # pylint: disable=E1101
        # Call UseCase base set_up function
        UseCaseBase.tear_down(self)
        time.sleep(self._wait_btwn_cmd)
        return Global.SUCCESS, "No errors"
