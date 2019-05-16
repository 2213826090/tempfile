"""

:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
otherwise. Any license under such intellectual property rights must be expressed
and approved by Intel in writing.

:summary: This file implements a Test Step to run sync stress from a host using MTP.
    It continuously sync a file for a given time.
:since: 30/09/2014
:author: Pierret David
:organization: NDG DSW
"""

from Core.TestStep.TestStepBase import TestStepBase
from ErrorHandling.DeviceException import DeviceException


class CheckUsbConnected(TestStepBase):
    """
    Check if a USB device is connected on the Host
    """
    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        TestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

        self._em = self._factory.create_equipment_manager()
        self._computer = self._em.get_computer("COMPUTER1")

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        TestStepBase.run(self, context)

        # Fetch params values
        vendor_id = self._pars.vendor_id
        product_id = self._pars.product_id
        command_result = self._pars.save_as

        local_cmd = "lsusb -d{0}:{1}".format(vendor_id, product_id)
        output = self._computer.run_cmd(local_cmd)
        std_out = output["std"]
        std_err = output["err"]
        find = False
        if std_out != "":
            if vendor_id in std_out:
                if product_id is not None or product_id != "":
                        if product_id in std_out:
                            find = True
                        else:
                            find = False
                else:
                    find = True

        if std_err != "":
            msg = "Error at command execution : %s" % str(std_err)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        context.set_info(command_result, str(find))
