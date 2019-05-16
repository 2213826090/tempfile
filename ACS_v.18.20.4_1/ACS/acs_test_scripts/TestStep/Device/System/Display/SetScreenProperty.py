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
:since:07/03/2014
:author: emarchan
"""

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from UtilitiesFWK.Utilities import str_to_bool_ex, is_number

STR_INVALID_VALUE = "Invalid value"
STR_STATE = "STATE"
STR_TIMEOUT = "TIMEOUT"
STR_BRIGHTNESS = "BRIGHTNESS"
STR_BACKLIGHT = "BACKLIGHT"

class SetScreenProperty(DeviceTestStepBase):
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

        property_to_set = self._pars.property.upper()
        # We assert here the possible values instead of checking the "default" case in the big
        # if/elif following in order to make it more testable.
        assert property_to_set in [STR_STATE, STR_TIMEOUT, STR_BRIGHTNESS, STR_BACKLIGHT], \
                                "this is impossible %s" % property_to_set

        value = self._pars.value

        # Call the method that will handle to property to set.

        # Properties with boolean
        if property_to_set == STR_STATE:
            self._manage_state(value)
            return

        # Properties with numbers
        if not is_number(value):
            self._raise_config_exception(STR_INVALID_VALUE)
        if property_to_set == STR_TIMEOUT:
            self._api.set_screen_timeout(int(value))
        elif property_to_set == STR_BACKLIGHT:
            self._api.set_backlight_level(value)
        elif property_to_set == STR_BRIGHTNESS:
            self._api.set_display_brightness(int(value))

    def _manage_state(self, value):
        """
        Handles the screen ON / OFF
        :type value: str
        :param value: Value to set (ON/OFF)
        """
        if str_to_bool_ex(value):
            value_to_set = "1"
        elif str_to_bool_ex(value) is False:
            value_to_set = "0"
        else:
            self._raise_config_exception(STR_INVALID_VALUE)
        self._api.set_phone_screen_lock_on(value_to_set)
