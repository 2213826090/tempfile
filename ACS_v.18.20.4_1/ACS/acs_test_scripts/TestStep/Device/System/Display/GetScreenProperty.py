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

:organization: INTEL MCG DRD
:summary: This file implements a Test Step to set screen properties.
:since:10/03/2014
:author: emarchan
"""

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from UtilitiesFWK.Utilities import TestConst

class GetScreenProperty(DeviceTestStepBase):
    """
    Set screen properties.
    """
    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._api = self._device.get_uecmd("PhoneSystem")

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        # Compute which information we want about the screen
        property_to_get = self._pars.property.upper()
        # We assert here the possible values instead of checking the "default" case in the big
        # if/elif following in order to make it more testable.
        assert property_to_get in ["STATE", "TIMEOUT", "BACKLIGHT", "RESOLUTION"], \
                                "this is impossible %s" % property_to_get

        # Where the information will be stored into the context?
        variable_to_set = self._pars.save_as

        # Get the value
        if property_to_get == "STATE":
            if self._api.get_screen_status():
                value = TestConst.STR_ON
            else:
                value = TestConst.STR_OFF
        elif property_to_get == "TIMEOUT":
            value = self._api.get_screen_timeout()
        elif property_to_get == "BACKLIGHT":
            value = self._api.get_backlight_level()
        elif property_to_get == "RESOLUTION":
            value = self._api.get_screen_resolution()

        # We have the value, let's save it into the context
        context.set_info(variable_to_set, value)
        self.ts_verdict_msg = "VERDICT: %s stored as {0}".format(value) % variable_to_set
        self._logger.debug(self.ts_verdict_msg)
