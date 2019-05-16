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
:summary: This file implements a wait for profile connection test step
:since:08/01/2014
:author: fbongiax
"""
from acs_test_scripts.TestStep.Device.Wireless.BT.Base import BtBase


class BtConnectProfile(BtBase):
    """
    Implements the connect profile test step
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        BtBase.run(self, context)

        if self._pars.connect is None or self._pars.connect == True:
            self._connect()
        else:
            self._disconnect()

    def _connect(self):
        """
        Connect the profile
        """
        if not self._api.connect_bt_device(self._pars.bdaddr, self._pars.profile):
            self._raise_device_exception("Unable to connect profile %s to %s" % (self._pars.profile, self._pars.bdaddr))

    def _disconnect(self):
        """
        Disconnect profile
        """
        if not self._api.disconnect_bt_device(self._pars.bdaddr, self._pars.profile):
            self._raise_device_exception("Unable to disconnect profile %s from %s" % (self._pars.profile,
                                         self._pars.bdaddr))
