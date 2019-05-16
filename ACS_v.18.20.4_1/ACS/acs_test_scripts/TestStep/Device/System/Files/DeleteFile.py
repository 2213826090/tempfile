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
:summary: This file implements the deletion of file on device
:since: 10/07/2014
:author: floeselx
"""
from acs_test_scripts.TestStep.Device.System.Files.FileBase import FileBase


class DeleteFile(FileBase):
    """
    Delete file class
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        FileBase.run(self, context)

        # Fetch params values
        path_support = str(self._pars.file_path)
        file_name = str(self._pars.file_name)
        make_secure_copy = self._pars.make_secure_copy

        # Build the complete path
        file_name = path_support + file_name

        # Delete the file on device
        self._logger.debug("Delete {0} on device".format(file_name))
        self._api.delete(file_name, make_secure_copy)

        exists = self._api.exist(file_name)[0]

        if exists:
            self._raise_device_exception("File %s has not been deleted" % file_name)
