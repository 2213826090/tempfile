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
:summary: This file implements a Test Step to stop an application
:since: 10/12/2014
:author: vdechefd
"""

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from UtilitiesFWK.Utilities import TestConst
from time import sleep as time_sleep

class AppEnableDisable(DeviceTestStepBase):

    """
    Enable or disable an application running on the device
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Initialize test step
        """
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._app_api = self._device.get_uecmd("AppMgmt")

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        package_name = self._pars.app_package_name
        action = self._pars.action.lower()
        
        #Convert action string to boolean
        if action.lower() == TestConst.STR_ENABLE:
            action_asked = True
        else:
            action_asked = False
        self._app_api.app_enable_disable(package_name, action_asked)
        
        #Check package status. It can take some time, so we give it some time to complete.
        new_status = None
        timeout = 10
        while new_status != action:
            new_status = self._app_api.get_app_status(package_name)
            if new_status == action:
                msg = "Package {0} is {1}".format(package_name, action)
                self.ts_verdict_msg = msg
                break;
            else:
                timeout = timeout - 1
                if timeout == 0: 
                    msg = "Package {0} is {1} instead of {2}".format(package_name, new_status, action)
                    self.ts_verdict_msg = msg
                    self._raise_device_exception(msg)
            time_sleep(1)
