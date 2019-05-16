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
:summary: This file is the process of use case LIVE_SYSTEM_FILE_OPERATION
:since: 10/28/2011
:author: Wchen61
"""

import time
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global


class LiveSystemFileOperation(UseCaseBase):

    """
    Class LiveSystemFileOperation.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)
        # Get TC Parameters
        self._folder_level = self._tc_parameters.get_param_value("FOLDER_LEVEL")
        self._folder_sub_number = self._tc_parameters.get_param_value("FOLDER_SUB_NUMBER")
        self._folder_name_length = self._tc_parameters.get_param_value("FOLDER_NAME_LENGTH")
        self._file_number = self._tc_parameters.get_param_value("FILE_NUMBER")
        self._file_size = self._tc_parameters.get_param_value("FILE_SIZE")
        self._file_name_length = self._tc_parameters.get_param_value("FILE_NAME_LENGTH")
        self._src_path = self._tc_parameters.get_param_value("SRC_PATH")
        self._dst_path = self._tc_parameters.get_param_value("DST_PATH")
        self._copy = self._tc_parameters.get_param_value("COPY")

        # Get UECmdLayer
        self._file_api = self._device.get_uecmd("File")

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # after 4 attempts
        # Call UseCase base Run function
        UseCaseBase.run_test(self)
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Create files...")
        result = self._file_api.operate(self._folder_level, self._folder_sub_number, self._folder_name_length,
                                        self._file_number, self._file_size, self._file_name_length, self._src_path, self._dst_path, self._copy)

        return Global.SUCCESS, result
