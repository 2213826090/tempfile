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
:summary: This file implements a Test Step that Cleans up OPP
:since:18/12/2013
:author: fbongiax
"""

from UtilitiesFWK.Utilities import split_and_strip
from acs_test_scripts.TestStep.Device.Wireless.BT.Base import BtBase
from acs_test_scripts.TestStep.Device.Wireless.BT.Constants import Constants


class BtOppCleanup(BtBase):
    """
    Implements the test step to clean up OPP

    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        BtBase.run(self, context)
        self._api.bt_opp_clean_notification_list()

        self._remove_files_if_needed()

    def _remove_files_if_needed(self):
        """
        Remove files from the storage if files attribute exists
        """
        if self._pars.remove_files:
            file_list = split_and_strip(self._pars.files, Constants.FILE_NAME_SEPARATOR)
            for file_name in file_list:
                self._api.bt_opp_init(file_name)
