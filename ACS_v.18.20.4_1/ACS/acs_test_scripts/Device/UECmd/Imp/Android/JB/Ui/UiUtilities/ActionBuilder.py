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
:since: 2014/02/11
:author: vdechefd
"""

from acs_test_scripts.Device.UECmd.Imp.Android.Common.Ui.UiUtilities.ActionBuilder \
    import ActionBuilder as CommonActionBuilder


class ActionBuilder(CommonActionBuilder):

    """
    This class inherits from ActionBuilder in Common package
    """

    def __init__(self, factory=None, device=None):
        """
        Constructor.

        :type device: DeviceModel
        :param device: the device model to be used by this object.

        :rtype: UiUtilitiesFactory
        :param factory: the factory instance to used.
        """
        CommonActionBuilder.__init__(self, factory, device)
        CommonActionBuilder.NAMES_AND_METHODS["type_monkey"] = "_init_type_monkey_action"

    def _init_type_monkey_action(
            self,
            action,
            operation_set_parameters,
            timeout=None):
        """
        Initializes and return a new C{CommonTypeAction} instance with:
            - standard action parameters
            - parameters that may be provided from an operation set
        Please note that the given I{operation set} parameter list
        is modified. It is the responsibility of the client class/method
        to create a copy of it when needed.

        If some parameters are missing from C{action} list
        and are required for the I{action} instantiation then
        C{operation_set_parameters} will be used as an additional
        parameter in order to create the instance.

        :type action: list
        :param action: the basic list of parameters
            for the action.

        :type operation_set_parameters: list
        :param operation_set_parameters: a list containing all additional
            parameters for the whole B{operation set}

        :type timeout: int
        :param timeout: the optional timeout to apply for this action.

        :rtype: TypeAction
        :return: a new corresponding C{UiAction} instance.
        """
        factory = self.get_factory()
        if not action["parameters"] or len(action["parameters"]) == 0:
            action["parameters"] = []
            action["parameters"].append(operation_set_parameters.pop(0))
        return factory.get_type_monkey_action(action, timeout)
