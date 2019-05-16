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
:summary: This file implements a Test Step that Checks the status of a file transfer
 via OPP(explicitly)
:since:05/03/2015
:author: mmaraci
"""

from UtilitiesFWK.Utilities import split_and_strip
from acs_test_scripts.TestStep.Device.Wireless.BT.Constants import Constants
from acs_test_scripts.TestStep.Device.Wireless.BT.Base import BtBase
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Utilities.LocalConnectivityUtilities import opp_accept_file, opp_check_file_cancelled, \
    opp_check_file_waiting_accept, opp_check_status_none


class BtOppCheckStatus(BtBase):
    """
    Implements the test step to checks OPP transfer statuses
    """

    DEFAULT_TIMEOUT = 30.0
    _STATUS_ACCEPTED = "accepted"
    _STATUS_CANCELED = "canceled"
    _STATUS_WAITING_ACCEPT = "waiting_accept"
    _STATUS_NONE = 'none'

    def __init__(self, tc_conf, global_conf, ts_conf, factory):

        BtBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

        if not self._pars.expected_state:
            self._pars.expected_state = self._STATUS_ACCEPTED

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        BtBase.run(self, context)

        if self._pars.timeout is not None:
            timeout = self._pars.timeout
        else:
            timeout = self.DEFAULT_TIMEOUT

        # Split the file names and strip each of them
        file_list = split_and_strip(self._pars.files, Constants.FILE_NAME_SEPARATOR)

        if self._pars.expected_state == self._STATUS_ACCEPTED:
            if not opp_accept_file(self._api, file_list[0], timeout):
                msg = "remote device has not accepted the file on time"
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        elif self._pars.expected_state == self._STATUS_CANCELED:
            if not opp_check_file_cancelled(self._api, file_list[0], timeout):
                msg = "remote device has not canceled the file on time"
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        elif self._pars.expected_state == self._STATUS_WAITING_ACCEPT:
            if not opp_check_file_waiting_accept(self._api, file_list[0], timeout):
                if not opp_accept_file(self._api, file_list[0], timeout):
                    msg = "remote device has not initiated the transfer on time"
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        elif self._pars.expected_state == self._STATUS_NONE:
            if not opp_check_status_none(self._api, timeout):
                msg = "remote device has a status different than none for a type it should not consider"
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)