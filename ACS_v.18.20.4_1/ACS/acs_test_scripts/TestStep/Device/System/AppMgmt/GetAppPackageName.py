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
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL MCG PSI
:summary: This file implements a Test Step to get package name from an application installer
:since: 10/12/2014
:author: vdechefd
"""

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from acs_test_scripts.Utilities.DeviceApplication import AndroidApplication


class GetAppPackageName(DeviceTestStepBase):

    """
    Get package name from an application installer
    This step is for Android platform only, as AndroidApplication module is used to get app's data
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Initialize test step
        """
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._application = AndroidApplication(logger=self._logger)

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        app_path = self._pars.app_installer_path
        self._application.set_application(app_path)
        package_name = self._application.get_package_name()
        context.set_info(self._pars.output, package_name)

        msg = "Package name is {0} for {1}".format(package_name, app_path)
        self.ts_verdict_msg = msg
        self._logger.info(msg)
