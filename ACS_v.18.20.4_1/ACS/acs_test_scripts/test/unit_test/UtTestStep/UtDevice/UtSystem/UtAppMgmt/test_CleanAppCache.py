# @PydevCodeAnalysisIgnore
# pylint: disable=E0602,W0212,C0103,C0111
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
:summary: Unit test module
:since: 23/10/14
:author: Lu, GangX
"""
from mock import MagicMock
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Device.System.AppMgmt.CleanAppCache import CleanAppCache
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global


class CleanAppCacheTestCase(UTestTestStepBase):

    def setUp(self):
        UTestTestStepBase.setUp(self)

    def test_get_package_name_ko(self):
        teststep = CleanAppCache(None, None, None, MagicMock())
        teststep._pars.app_name = "android.package.myapp"

        teststep._app_api.get_path_of_device_app.return_value = ""

        with self.assertRaises(DeviceException):
            teststep.run(self._context)

    def test_get_package_name_ok_clear_cache_ko(self):
        teststep = CleanAppCache(None, None, None, MagicMock())
        teststep._pars.app_name = "android.package.myapp"

        teststep._app_api.get_path_of_device_app.return_value = "/data/data/com.android.packagename"
        teststep._app_api.clear_cache_data_of_device_app.return_value = Global.FAILURE, "error clear cache"

        with self.assertRaises(DeviceException):
            teststep.run(self._context)

    def test_get_package_name_ok_clear_cache_ok(self):
        teststep = CleanAppCache(None, None, None, MagicMock())
        teststep._pars.app_name = "android.package.myapp"

        teststep._app_api.get_path_of_device_app.return_value = "/data/data/com.android.packagename"
        teststep._app_api.clear_cache_data_of_device_app.return_value = Global.SUCCESS, "success clear cache"

        teststep.run(self._context)
