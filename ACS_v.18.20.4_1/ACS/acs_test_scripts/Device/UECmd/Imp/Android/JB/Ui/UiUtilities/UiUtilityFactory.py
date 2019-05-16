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
:since: 12 Sep 2012
:author: sfusilie
"""

from TouchAction import TouchAction
from DragAction import DragAction
from DragMonkeyAction import DragMonkeyAction
from TypeAction import TypeAction
from TypeMonkeyAction import TypeMonkeyAction
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Ui.UiUtilities.UiUtilitiesFactory \
    import UiUtilitiesFactory as CommonUiUtilitiesFactory


class UiUtilitiesFactory(CommonUiUtilitiesFactory):

    def __init__(self, monkey_port, device, global_config):
        CommonUiUtilitiesFactory.__init__(self, monkey_port, device, global_config)
        CommonUiUtilitiesFactory.NAMES_AND_METHODS["drag_monkey"] = "get_drag_monkey_action"
        CommonUiUtilitiesFactory.NAMES_AND_METHODS["type_monkey"] = "get_type_monkey_action"

    def get_touch_action(self, parameters, timeout=None):
        """
        Returns a new C{TouchAction} instance.
            - C{parameters[0]}: x coordinate
            - C{parameters[1]}: y coordinate
            - C{parameters[2]}: timeout (optional)
                If present, this timeout overrides the C{timeout}
                method parameter.
        :type parameters: list
        :param parameters: the list of parameter for the I{action} creation.
            The list expected content is described above.
        :type timeout: int
        :param timeout: the optional timeout to apply for this action.
        """
        if timeout is None or timeout == "None":
            timeout = self.__type_timeout
        instance = TouchAction(self.get_monkey(),
                               self.__device,
                               parameters,
                               timeout)
        return instance

    def get_drag_action(self, parameters, timeout=None):
        """
        Returns a new C{DragAction} instance.
            - C{parameters[0]}: int : the start point x coordinate
            - C{parameters[1]}: int : the start point y coordinate
            - C{parameters[2]}: int : the end point x coordinate
            - C{parameters[3]}: int : the end point x coordinate
            - C{parameters[4]}: int : timeout (optional)
                If present, this timeout overrides the C{timeout}
                method parameter.
        """
        if timeout is None or timeout == "None":
            timeout = self.__type_timeout
        instance = DragAction(self.get_monkey(), self.__device, parameters, timeout)
        return instance

    def get_drag_monkey_action(self, parameters, timeout=None):
        """
        Returns a new C{CommonDragAction} instance.
            - C{parameters[0]}: int : the start point x coordinate
            - C{parameters[1]}: int : the start point y coordinate
            - C{parameters[2]}: int : the end point x coordinate
            - C{parameters[3]}: int : the end point x coordinate
            - C{parameters[4]}: int : timeout (optional)
                If present, this timeout overrides the C{timeout}
                method parameter.
        """
        if timeout is None or timeout == "None":
            timeout = self.__type_timeout
        instance = DragMonkeyAction(self.get_monkey(), parameters, timeout)
        return instance

    def get_type_action(self, parameters, timeout=None):
        """
        Returns a new C{TypeAction} instance.
            - C{parameters[0]}: text: the text to type
        :type parameters: list
        :param parameters: the list of parameter for the I{action} creation.
            The list expected content is described above.
        :type timeout: int
        :param timeout: the optional timeout to apply for this action.
        """
        if timeout is None or timeout == "None":
            timeout = self.__type_timeout

        # Build the instance
        instance = TypeAction(self.get_monkey(),
                              self.__device,
                              parameters,
                              timeout,
                              self.__global_config.benchConfig,
                              self.__global_config.deviceConfig)
        return instance

    def get_type_monkey_action(self, parameters, timeout=None):
        """
        Returns a new C{TypeMonkeyAction} instance.
            - C{parameters[0]}: text: the text to type
        :type parameters: list
        :param parameters: the list of parameter for the I{action} creation.
            The list expected content is described above.
        :type timeout: int
        :param timeout: the optional timeout to apply for this action.
        """
        if timeout is None or timeout == "None":
            timeout = self.__type_timeout

        # Build the instance
        instance = TypeMonkeyAction(self.get_monkey(),
                                    self.__device,
                                    parameters,
                                    timeout,
                                    self.__global_config.benchConfig,
                                    self.__global_config.deviceConfig)
        return instance
