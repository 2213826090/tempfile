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

:organization: INTEL MCG PSI
:summary: This file is the process of use case LIVE_SYSTEM_FILE_NAME_OPERATION
:since: 02/27/2012
:author: xzhao24
"""

import time
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global


class LiveSystemFileNameOperation(UseCaseBase):

    """
    Class LiveSystemFileNameOperation.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        UseCaseBase.__init__(self, tc_name, global_config)

        # Get TC Parameters
        self._object_type = self._tc_parameters.get_param_value("OBJECT_TYPE")
        self._previous_name = self._tc_parameters.get_param_value("PREVIOUS_NAME")
        self._following_name = self._tc_parameters.get_param_value("FOLLOWING_NAME")
        self._operation_type = self._tc_parameters.get_param_value("OPERATION_TYPE")

        # Get UECmdLayer
        self._file_api = self._device.get_uecmd("File")

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        UseCaseBase.run_test(self)
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Rename or create file with multi-extension name...")
        result = self._file_api.operate_file_name(self._object_type, self._previous_name,
                                                  self._following_name, self._operation_type)

        return Global.SUCCESS, result
