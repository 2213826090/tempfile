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

:organization: INTEL NDG
:summary: This file implements commands sending to the modem
:since: 10 september 2014
:author: mSouyrix
"""
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase


class ATCommand(DeviceTestStepBase):
    """
    AT command class
    """
    def __init__(self, tc_name, global_config, ts_conf, factory):
        """
        Constructor
        """
        # Call DeviceTestStepBase base Init function
        DeviceTestStepBase.__init__(self, tc_name, global_config, ts_conf, factory)

        # Get UECmdLayer
        self._modem_api = self._device.get_uecmd("Modem")

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        command_result = self._modem_api.send_at_command(self._pars.serial_device,
                                                         self._pars.command,
                                                         self._pars.timeout)

        # Save the command result in the context variable
        context.set_info(self._pars.save_as, command_result)
