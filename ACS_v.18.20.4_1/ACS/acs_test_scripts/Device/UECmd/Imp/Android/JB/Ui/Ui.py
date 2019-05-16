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
:summary: This file implements the Ui UEcmd for Android device
:since: 12 sep 2012
:author: sfusilie
"""

from acs_test_scripts.Device.UECmd.Imp.Android.Common.Ui.Ui \
    import Ui as UiICS
from acs_test_scripts.Device.UECmd.Imp.Android.JB.Ui.UiUtilities.ActionBuilder \
    import ActionBuilder
from acs_test_scripts.Device.UECmd.Imp.Android.JB.Ui.UiUtilities.UiUtilityFactory \
    import UiUtilitiesFactory


class Ui(UiICS):

    """
    :summary: Ui UEcommands operations for Android platform
    """

    def __init__(self, phone):
        """
        Constructor
        """
        UiICS.__init__(self, phone)

    def init(self, error_policy=None, enable_ui_automator_log=False):
        """
        Used to establish connection to a port, and/or all other action to do to initialize the Ui uecmd instance

        :type error_policy: str
        :param error_policy: (Optional) Used for Android to manage error displaying

        :type: enable_ui_automator_log: boolean
        :param: enable_ui_automator_log: Enable or not ui automator on the device
        """
        UiICS.init(self, error_policy)

        monkey_port = self._device.get_monkey_port()
        self._factory = UiUtilitiesFactory(monkey_port, self._device, global_config=self.__global_config)
        self._builder = ActionBuilder(self._factory)

        # Hanlde UI Automator needs
        if enable_ui_automator_log:
            monkey = self._factory.get_monkey()
            monkey.enable_ui_automator_log()
