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
:summary: This file implements the mount of partition on device
:since: 14/08/2014
:author: floeselx
"""

from Core.TestStep.TestStepBase import TestStepBase
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.TestStep.Utilities.Files.FileBase import FileBase


class MountHostPartition(FileBase):
    """
    Mount partition on host
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        FileBase.run(self, context)

        # Fetch params values
        partition_name = self._pars.partition
        mount_point = self._pars.mount_point

        self._logger.info("Mount partition {0} on mount point {1}".format(partition_name, mount_point))

        # Mount the partition on device
        self._computer.mount_host_partition(partition_name, mount_point)
        self._logger.info("Successfully mounted partition {0} on mount point {1}".format(partition_name, mount_point))
