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

:organization: INTEL NDG SW DEV
:summary: This file implements a Test Step that retrieve device name as it appears when device is advertising
:since:02/03/2015
:author: msouyrix
"""

from TestStep.Device.Wireless.BT.Base import BtBase


class BtGetAdvertisingName(BtBase):
    """
    Implements the test step which retrieve advertising name
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        BtBase.run(self, context)

        device_name = ""
        device_list = self._api.bt_scan_devices()
        for devTmp in device_list:
            if devTmp.address == self._pars.device_to_check:
                device_name = devTmp.name

        if device_name == "":
            self._logger.debug("BtCheckAdv: device {0} isn't advertising".format(self._pars.device_to_check))
        else:
            self._logger.debug("BtCheckAdv: device {0} is advertising (name={1})".format(self._pars.device_to_check,
                                                                                         device_name))

        context.set_info(self._pars.save_as, device_name)
