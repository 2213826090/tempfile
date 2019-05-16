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

:organization: INTEL PEG-SVE-DSV
:summary: This file implements a Test Step to control Bluetooth tethering adapter "power", although "power" is probably a misnomer.
:since: 01May2014
:author: Val Peterson
"""
from acs_test_scripts.Device.UECmd.UECmdTypes import BT_STATE
from UtilitiesFWK.Utilities import TestConst, split_and_strip
from acs_test_scripts.TestStep.Device.Wireless.BT.Base import BtBase


class BtSetTetheringPower(BtBase):
    """
    Implements the base test step for BT
    """

    # Constants
    STR_SEPARATOR = ","

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        BtBase.run(self, context)

        power = str(self._pars.power).lower()

        # # Get the power sequence (it might just be one single operation)
        power_sequence = split_and_strip(power, self.STR_SEPARATOR)

        for current in power_sequence:
            assert current in [TestConst.STR_ON, TestConst.STR_OFF], \
            "passed value (%s) is invalid at this stage" % current
            self._logger.info("Power %s the Bluetooth tethering" % current)

            self._api.set_bt_tethering_power(current)

            if current == TestConst.STR_ON and self._api.get_bt_tethering_power() != True:
                self._raise_device_exception("set BT tethering ON failure")

            if current == TestConst.STR_OFF and self._api.get_bt_tethering_power() != False:
                self._raise_device_exception("set BT tethering OFF failure")
