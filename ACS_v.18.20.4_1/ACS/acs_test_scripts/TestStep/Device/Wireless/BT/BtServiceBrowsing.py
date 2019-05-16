"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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
:summary: This file implements a Test Step to browse (discover) services on device
:since: 20150107
:author: Julien Reynaud
"""
from Core.TestStep.TestStepParameters import TestStepParameters
from acs_test_scripts.TestStep.Device.Wireless.BT.Base import BtBase
from acs_test_scripts.Device.UECmd.UECmdTypes import BluetoothDevice


class BtServiceBrowsing(BtBase):
    """
    Implements the base test step for BT
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        BtBase.run(self, context)

        bd_address = str(self._pars.bdaddr)
        class_to_browse = str(self._pars.class_to_browse)
        # Because optional
        if class_to_browse is None:
            class_to_browse = ""

        self._logger.info("Browse service class '{0}' on device {1}".format(class_to_browse, bd_address))
        status, bt_dev = self._api.bt_service_browsing(bd_address=bd_address, class_to_browse=class_to_browse)

        if bt_dev is not None and isinstance(bt_dev, BluetoothDevice):
            context.set_nested_info([self._pars.save_as, "ADDRESS"], bt_dev.address)
            context.set_nested_info([self._pars.save_as, "NAME"], bt_dev.name)
            context.set_nested_info([self._pars.save_as, "BONDED"], bt_dev.bonded)
            context.set_nested_info([self._pars.save_as, "INTERFACE"], bt_dev.interface)
            # format list a test step parameter style
            uuids_list_step_style = TestStepParameters.LIST_KEYWORD.join(bt_dev.uuids)
            context.set_nested_info([self._pars.save_as, "UUID"], uuids_list_step_style)