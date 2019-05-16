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

:organization: INTEL NDG SW
:summary: This file contains the File Base class
:since 10/07/2014
:author: floeselx
"""

from acs_test_scripts.TestStep.Device.System.Files.Constants import Constants
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase


class FileBase(DeviceTestStepBase):
    """
    Implements the base test step for Files
    Attributes:
        DEVICE (string): @see DeviceTestStepBase
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

        # gets handle to the API
        self._api = self._device.get_uecmd(Constants.STR_FILES_UECMD)
        self._phonesystem_api = self._device.get_uecmd(Constants.STR_PHONESYSTEM_UECMD)
