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
:summary: This file implements the Ui UEcmd for Android phone
:since: 14/12/2011
:author: ssavrimoutou
"""

import os

from acs_test_scripts.Device.UECmd.Imp.Android.Common.Base import Base
from acs_test_scripts.Device.UECmd.Interface.Ui.IUi import IUi
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Ui.UiUtilities.ActionBuilder \
    import ActionBuilder
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Ui.UiUtilities.UiUtilitiesFactory \
    import UiUtilitiesFactory
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Ui.UiUtilities.UiActionRunner \
    import UiActionRunner


class Ui(Base, IUi):

    """
    :summary: Ui UEcommands operations for Android platforms
    using an monkey to send key event to the I{DUT}.
    """

    RAISE_EXCEPTIONS_POLICY = "RAISE_EXCEPTION"
    """
    The I{raise exception} management policy,
    when an UI action related error occurs,
    an exception is raised.
    """

    LOG_ERROR_MANAGEMENT_POLICY = "LOG_WARNING"
    """
    An alternate error management policy, simply
    logs warnings on errors.
    """

    DEFAULT_ERROR_MANAGEMENT_POLICY = LOG_ERROR_MANAGEMENT_POLICY
    """
    The error management policy to use by default.
    """

    ALLOWED_ERROR_MANAGEMENT_POLICIES = (
        RAISE_EXCEPTIONS_POLICY,
        LOG_ERROR_MANAGEMENT_POLICY)
    """
    The list of all allowed error management policy names.
    """

    def __init__(self, phone):
        """
        Constructor.
        """

        Base.__init__(self, phone)
        IUi.__init__(self, phone)
        self._logger = phone.get_logger()

        self._factory = None
        self._builder = None
        self._runner = None
        self._isInitialized = False
        self.__error_management = Ui.DEFAULT_ERROR_MANAGEMENT_POLICY
        self.__global_config = None

    def set_global_config(self, global_config):
        self.__global_config = global_config

    @property
    def isInitialized(self):
        return self._isInitialized

    def init(self, error_policy=None, enable_ui_automator_log=False):
        """
        Used to establish connection to a port, and/or all other action to do to initialize the Ui uecmd instance

        :type error_policy: str
        :param error_policy: (Optional) Used for Android to manage error displaying

        :type: enable_ui_automator_log: boolean
        :param: enable_ui_automator_log: Enable or not ui automator on the device
        """
        # Initialize policy
        if error_policy in Ui.ALLOWED_ERROR_MANAGEMENT_POLICIES:
            self.__error_management = error_policy

        # Initialize UI object
        monkey_port = self._device.get_monkey_port()
        self._factory = UiUtilitiesFactory(monkey_port, self._device,
                                           global_config=self.__global_config)
        self._builder = ActionBuilder(self._factory)
        self._runner = UiActionRunner(self._logger,
                                      self._device_logger,
                                      self._device.get_config("waitBetweenCmd", 5, float))
        error_manager = self._factory.get_error_manager_for_name(
            self.__error_management)
        self._runner.set_error_manager(error_manager)
        self._isInitialized = True

    def release(self):
        """
        Used to disconnect from a port, and/or all other action to do to release the Ui uecmd instance
        """
        # Release the monkey instance.
        # First action to do, in case of an exception
        # that could occur before that otherwise
        self._factory.get_monkey().disconnect()

    def run_action_from_script(self, script_name):
        """
        Execute UI action defined from xml scripts

        :type script_name: str
        :param script_name: The path to the script which contains all UI sequences to execute
        """

        # Check if the script exists
        if not os.path.isfile(script_name):
            error_msg = "Ui script '%s' not found !" % str(script_name)
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        # Build the action list
        actions = self._builder.build_action_list_from_script(script_name)
        # Run the actions
        return self._runner.run_actions(actions)

    def run_operation_set(self, operation_set_name, parameters=None):
        """
        Execute operation set defined from Device OpDictionary

        :type operation_set_name: str
        :param operation_set_name: The name of the operation set, it MUST be defined in the OpDictionary

        :type parameters: list
        :param parameters: the list of parameters for this operation set, it MUST be defined in the OpDictionary
        """

        # Build actions from given operation set
        op_set_action = self._builder.build_operation_set(operation_set_name, parameters)
        # Run the actions
        return self._runner.run_actions(op_set_action)
