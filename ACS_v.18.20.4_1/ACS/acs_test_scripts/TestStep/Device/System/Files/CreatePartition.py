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
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL NDG SW
:summary: This file implement the creation of a partition on the DUT
:since: 06/10/2014
:author: dpierrex
"""
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from acs_test_scripts.TestStep.Device.System.Files.FileBase import FileBase


class CreatePartition(FileBase):
    """
    create partition on the DUT
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        block_device = self._pars.block_device
        partition_type = self._pars.partition_type
        start = self._pars.start
        end = self._pars.end

        fs_type = self._pars.file_system

        partition_number = self._api.create_partition(block_device, partition_type, start, end)

        self._api.format_partition(block_device, partition_number, fs_type)

        context.set_info(self._pars.save_as, "{0}{1}".format(block_device, partition_number))
