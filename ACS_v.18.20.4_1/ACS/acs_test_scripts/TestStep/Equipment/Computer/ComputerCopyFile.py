"""
@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

:organization: INTEL QCTV
:summary: This file implements Test Step for copying a local file to a remote computer
:since 04/03/2016
:author: pblunie
"""
from acs_test_scripts.TestStep.Equipment.Computer.ComputerBase import ComputerBase


class ComputerCopyFile(ComputerBase):
    """
    Implements copy file test step on a remote computer
    """

    def run(self, context):
        """
        Run the test step
        """
        ComputerBase.run(self, context)

        computer_connection = self._computer.init()

        result = self._computer.copy_file_in_remote_path(self._pars.local_filename,
                                                         self._pars.remote_filename)
        self._logger.debug("Copy file result: %s" % result)

        if computer_connection:
            self._computer.release()
