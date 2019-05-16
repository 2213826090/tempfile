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

:summary: This file implements a Test Step for export some PathManager variable to the context
:author: dpierrex
:since 15/10/2014
:organization: INTEL NDG
"""
from Core.PathManager import Paths
from Core.TestStep.TestStepBase import TestStepBase


class ExportPath(TestStepBase):
    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        TestStepBase.run(self, context)

        context.set_nested_info([self._pars.save_paths_as, "EXECUTION_CONFIG"], Paths.EXECUTION_CONFIG)
        context.set_nested_info([self._pars.save_paths_as, "EXTRA_LIB_FOLDER"], Paths.EXTRA_LIB_FOLDER)
        context.set_nested_info([self._pars.save_paths_as, "REPORTS"], Paths.REPORTS)
        context.set_nested_info([self._pars.save_paths_as, "FLASH_FILES"], Paths.FLASH_FILES)
        context.set_nested_info([self._pars.save_paths_as, "CONFIGS"], Paths.CONFIGS)
