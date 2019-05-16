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
:summary: This file implements a wait for pairing test step
:since:08/01/2014
:author: fbongiax
"""
import time
from acs_test_scripts.Device.UECmd.UECmdTypes import BtConState
from acs_test_scripts.TestStep.Device.Wireless.BT.Base import BtBase
from acs_test_scripts.TestStep.Device.Wireless.BT.Constants import Constants

class BtCheckProfileStatus(BtBase):
    """
    Implements the connect profile test step
    """

    _SLEEP_TIME_SECS = 1
    _DEFAULT_TIMEOUT_SECS = 5

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """

        BtBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._status = None
        self._sleep_time = self._SLEEP_TIME_SECS
        self._start_time = None
        self._timeout = self._pars.timeout if self._pars.timeout else self._DEFAULT_TIMEOUT_SECS

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        BtBase.run(self, context)

        self._start_time = time.time()
        keep_going = True

        while keep_going:
            self._status = self._api.get_bt_connection_state(self._pars.bdaddr, self._pars.profile)
            keep_going = self._keep_going()

        self._raise_error_if_fail_when()

    def _raise_error_if_fail_when(self):
        """
        Raises an error if must fail
        """
        fail_if = str(self._pars.fail_if).lower()
        if fail_if == Constants.PROFILE_CONNECTED and self._status == BtConState.d[BtConState.CONNECTED]:
            self._raise_device_exception("Profile connection state is CONNECTED")
        elif fail_if == Constants.PROFILE_DISCONNECTED and self._status == BtConState.d[BtConState.DISCONNECTED]:
            self._raise_device_exception("Profile connection state is DISCONNECTED")

    def _keep_going(self):
        """
        Decides whether to keep polling the device
        """
        keep_going = not self._status in [BtConState.d[BtConState.CONNECTED], BtConState.d[BtConState.DISCONNECTED]]
        if keep_going and time.time() - self._start_time > self._timeout:
            self._raise_device_exception("Expected status hasn't been reached before timeout")
        return keep_going
