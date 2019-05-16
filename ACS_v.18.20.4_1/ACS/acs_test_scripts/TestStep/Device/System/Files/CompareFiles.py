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
:summary: This file implements the comparison of files on platform
:since: 10/07/2014
:author: floeselx
"""
from acs_test_scripts.TestStep.Device.System.Files.FileBase import FileBase
from ErrorHandling.DeviceException import DeviceException


class CompareFiles(FileBase):
    """
    Compare class
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        FileBase.run(self, context)

        # Fetch params values
        src_path = str(self._pars.src_path)
        dest_path = str(self._pars.dest_path)
        src_file_name = str(self._pars.src_file)
        dest_file_name = str(self._pars.dest_file)

        # Build the complete paths
        src_file_name = src_path + src_file_name
        dest_file_name = dest_path + dest_file_name

        # Compare the files on device
        self._logger.info("CompareFiles: compare content of %s and %s" % (src_file_name, dest_file_name))
        status, output = self._api.is_equal(src_file_name, dest_file_name)

        self._logger.info(output)

        if not status:
            raise DeviceException(DeviceException.OPERATION_FAILED, output)
