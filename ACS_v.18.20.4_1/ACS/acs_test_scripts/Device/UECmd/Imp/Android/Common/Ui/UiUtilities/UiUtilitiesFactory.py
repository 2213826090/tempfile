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
:since: 2011/09/08
:author: asebbane
"""

from OperationSet import OperationSet
from WaitAction import WaitAction
from DragAction import DragAction
from ExecAction import ExecAction
from KeyDownAction import KeyDownAction
from KeyPressAction import KeyPressAction
from KeyUpAction import KeyUpAction
from LongTouchAction import LongTouchAction
from DoubleTouchAction import DoubleTouchAction
from ScreenshotAction import ScreenshotAction
from ReplayAction import ReplayAction
from TypeAction import TypeAction
from WakeAction import WakeAction
from TouchAction import TouchAction
from ErrorHandling.AcsToolException import AcsToolException
from UtilitiesFWK.Utilities import Global
from UiActionRunner import UiActionRunner
from UiUtilities import AcsConfigException
from UiUtilities import RaiseExceptionErrorManager
from UiUtilities import LogWarningErrorManager
from UiUtilities import ActionResultListener
from MonkeyUtilities import MonkeyUtilities


class UiUtilitiesFactory(object):

    """
    A class for I{UI} actions creation and initialization.
    """

    # The matching between action names and the corresponding
    # initialization methods.
    NAMES_AND_METHODS = {
        "touch": "get_touch_action",
        "longtouch": "get_long_touch_action",
        "type": "get_type_action",
        "drag": "get_drag_action",
        "keypress": "get_key_press_action",
        "screenshot": "get_screenshot_action",
        "wait": "get_wait_action",
        "wake": "get_wake_action",
        "exec": "get_exec_action",
        "keyup": "get_key_up_action",
        "keydown": "get_key_down_action",
        "replay": "get_replay_action",
        "doubletouch": "get_double_touch"
    }

    # The list of all error management policies.
    ERROR_MANAGEMENT_POLICY_NAMES = (
        "RAISE_EXCEPTION",
        "LOG_WARNING")

    def get_allowed_action_names(self):
        """
        Returns a list of all action names that
        this factory is able to instantiate.
        .. note:: the action names are not the classe names.

        :rtype: list
        :return: the list of action names.
        """
        return UiUtilitiesFactory.NAMES_AND_METHODS.keys()

    def __init__(self, monkey_port, device, global_config):
        self.__monkey_port = monkey_port
        self.__device = device
        self.__logger = self.__device.get_logger()
        self.__device_name = self.__device.get_name()
        self.__wait_btwn_op = self.__device.get_config("waitBetweenCmd", 5, float)
        self.__monkey_util = None
        self.__type_timeout = self.__device.get_ui_type_timeout()
        self.__global_config = global_config


    def get_type_timeout(self):
        """
        Returns device UI type timeout
        """
        return self.__type_timeout


    def get_logger(self):
        """
        Returns this object's configured logger instance.

        :return: the logger instance.
        """
        return self.__logger

    def get_device(self):
        """
        Return this factory's configured device instance.
        :rtype: DeviceModel
        :return: the device instance used by this object.
        """
        return self.__device

    def _get_factory_method_for_name(self, name):
        """
        Returns the initialization method corresponding
        to the given action name.

        :type name: str
        :param name: the action name.
        """
        method = None
        try:
            method = UiUtilitiesFactory.NAMES_AND_METHODS[name]
        except KeyError:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Invalid action name: %s" % (str(name)))
        return method

    def get_exception_error_manager(self):
        """
        Returns a new C{RaiseExceptionErrorManager} instance.

        :rtype: RaiseExceptionErrorManager
        :return: a new C{RaiseExceptionErrorManager} instance.
        """
        return RaiseExceptionErrorManager(self.__logger)

    def get_log_error_manager(self):
        """
        Returns a new C{LogWarningErrorManager} instance.

        :rtype: LogWarningErrorManager
        :return: a new C{LogWarningErrorManager} instance.
        """
        return LogWarningErrorManager(self.__logger)

    def get_error_manager_for_name(self, name):
        """
        Returns a C{UiErrorManager} instance suitable
        for the given error management policy name.

        :type name: str
        :param name: the error management policy name.

        :rtype: UiErrorManager
        :return: a new C{UiErrorManager} instance
        """
        error_manager = None
        if name in UiUtilitiesFactory.ERROR_MANAGEMENT_POLICY_NAMES:
            if name == "RAISE_EXCEPTION":
                error_manager = self.get_exception_error_manager()
            elif name == "LOG_WARNING":
                error_manager = self.get_log_error_manager()
        else:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Unknown error management policy: %s" % (str(name)))
        return error_manager

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
        instance = DragAction(self.get_monkey(), parameters, timeout)
        return instance

    def get_exec_action(self, parameters, timeout=None):
        """
        Returns a new C{ExecAction} instance.
            - C{parameters[0]} : str : command
            - C{parameters[1]} : int : timeout (optional).
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
        # Add the device instance to the parameters
        parameters["device"] = self.__device
        # Build the instance
        instance = ExecAction(self.get_monkey(), parameters, timeout)
        return instance

    def get_key_press_action(self, parameters, timeout=None):
        """
        Returns a new C{KeyPressAction} instance.
            - C{parameters[0]} : key code
        :type parameters: list
        :param parameters: the list of parameter for the I{action} creation.
            The list expected content is described above.
        :type timeout: int
        :param timeout: the optional timeout to apply for this action.
        """
        if timeout is None or timeout == "None":
            timeout = self.__type_timeout

        instance = KeyPressAction(self.get_monkey(), parameters, timeout)
        return instance

    def get_key_down_action(self, parameters, timeout=None):
        """
        Returns a new C{KeyDownAction} instance.
            - C{parameters[0]} : key code
        :type parameters: list
        :param parameters: the list of parameter for the I{action} creation.
            The list expected content is described above.
        :type timeout: int
        :param timeout: the optional timeout to apply for this action.
        """
        if timeout is None or timeout == "None":
            timeout = self.__type_timeout
        instance = KeyDownAction(self.get_monkey(), parameters, timeout)
        return instance

    def get_key_up_action(self, parameters, timeout=None):
        """
        Returns a new C{KeyUpAction} instance.
            - C{parameters[0]} : key code
        :type parameters: list
        :param parameters: the list of parameter for the I{action} creation.
            The list expected content is described above.
        :type timeout: int
        :param timeout: the optional timeout to apply for this action.
        """
        if timeout is None or timeout == "None":
            timeout = self.__type_timeout
        instance = KeyUpAction(self.get_monkey(), parameters, timeout)
        return instance

    def get_long_touch_action(self, parameters, timeout=None):
        """
        Returns a new C{LongTouchAction} instance.
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
        instance = LongTouchAction(self.get_monkey(), parameters, timeout)
        return instance

    def get_screenshot_action(self, parameters, timeout=None):
        """
        Returns a new C{ScreenshotAction} instance.
            - C{parameters[0]} : file name
            - C{parameters[1]} : file extension
                (optional if no more parameter is given,
                mandatory otherwise : may be <None>)
            - C{parameters[2]} : the timeout (optional)
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
        # Add the device instance to the parameters
        parameters["device"] = self.__device
        # Build the instance
        instance = ScreenshotAction(self.get_monkey(), parameters, timeout)
        return instance

    def get_double_touch(self, parameters, timeout=None):
        """
        Returns a new C{DoubleTouchAction} instance.
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
        instance = DoubleTouchAction(self.get_monkey(), parameters, timeout)
        return instance

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

    def get_wait_action(self, parameters, timeout=None):
        """
        Returns a new C{WaitAction} instance.
        The C{parameters} may be None or an empty list.
        :type parameters: list
        :param parameters: the list of parameter for the I{action} creation.
            The list expected content is described above.
        :type timeout: int
        :param timeout: the optional timeout to apply for this action.
        """
        if timeout is None or timeout == "None":
            timeout = self.__type_timeout
        instance = WaitAction(self.get_monkey(), parameters, timeout)
        return instance

    def get_replay_action(self, parameters, timeout=None):
        """
        Returns a new C{ReplayAction} instance.
        The C{parameters} may be None or an empty list.
        :type parameters: list
        :param parameters: the list of parameter for the I{action} creation.
            The list expected content is described above.
        :type timeout: int
        :param timeout: the optional timeout to apply for this action.
        """
        if timeout is None or timeout == "None":
            timeout = self.__type_timeout

        # Add the device instance to the parameters
        parameters["device"] = self.__device

        instance = ReplayAction(self.get_monkey(), parameters, timeout)
        return instance

    def get_wake_action(self, parameters, timeout=None):
        """
        Returns a new C{WakeAction} instance.
        The C{parameters} may be None or an empty list.
        :type parameters: list
        :param parameters: the list of parameter for the I{action} creation.
            The list expected content is described above.
        :type timeout: int
        :param timeout: the optional timeout to apply for this action.
        """
        if timeout is None or timeout == "None":
            timeout = self.__type_timeout
        instance = WakeAction(self.get_monkey(), parameters, timeout)
        return instance

    def get_action_for_name(self, action_name, parameters, timeout=None):
        """
        :type action_name: str
        :param action_name: the action's name

        :type parameters: list
        :param parameters: the action parameters

        :type timeout: int
        :param timeout: the optional timeout to apply for this action.
        """
        factory_method = self._get_factory_method_for_name(action_name)

        action = getattr(self, factory_method)(parameters, timeout)
        return action

    def get_iteration(self, action_list):
        """
        Returns a new C{OperationSet} instance.

        :type iteration: int
        :param iteration: number of time to run actions

        :type action_list: list
        :param action_list: a list of C{UiAction}s

        :rtype: OperationSet
        :return: a new I{operation set} instance.
        """
        op_set = OperationSet(self.get_monkey(), action_list, self.__wait_btwn_op)
        op_set.set_name("Iteration")
        return op_set

    def get_ui_action_runner(self, logger, logcat_logger):
        """
        Returns a new C{UiActionRunner} instance.

        :type logger: Log
        :param logger: the logger instance

        :param logcat_logger: the logcat logger
        """
        return UiActionRunner(logger, logcat_logger, self.__wait_btwn_op)

    def get_operation_set(self, actions=None):
        """
        Returns a new C{OperationSet} instance.

        :type actions: list
        :param actions: a list of C{UiAction}s

        :rtype: OperationSet
        :return: a new I{operation set} instance.
        """
        if not isinstance(actions, list):
            # Set default value if actions is not a list
            actions = []

        return OperationSet(self.get_monkey(), actions, self.__wait_btwn_op)

    def get_action_result_listener(self):
        """
        Returns a new C{ActionResultListener} instance.
        """
        return ActionResultListener()

    def get_monkey(self):
        """
        Returns the C{MonkeyUtilities} instance to use with this object.
        :rtype: MonkeyUtilities
        :return: the C{MonkeyUtilities} instance
        """
        if self.__monkey_util is None:
            monkey = self._initialize_new_monkey_instance()
            self.set_monkey(monkey)
        return self.__monkey_util

    def set_monkey(self, monkey):
        """
        Sets the C{MonkeyUtilities} used by this object
        to the given value.

        :type monkey: MonkeyUtilities
        :param monkey: the new C{MonkeyUtilities} to use
        """
        self.__monkey_util = monkey

    def _initialize_new_monkey_instance(self):
        """
        Returns a new C{MonkeyUtilities} instance.

        This method will allow test optimization (with
        usage of mock implementation).

        :rtype: MonkeyUtilities
        :return: a new C{MonkeyUtilities} instance.
        """
        if self.__monkey_port is None:
            raise AcsConfigException(AcsConfigException.PROHIBITIVE_BEHAVIOR, "Monkey port cannot be <None>")
        if not isinstance(self.__monkey_port, int):
            self.__monkey_port = int(self.__monkey_port)

        oom_protection = self.__device.get_config("enableMonkeyOomProtection", True, "str_to_bool")

        monkey_util = MonkeyUtilities(
            host="localhost",
            port=self.__monkey_port,
            device=self.get_device(),
            protect_from_oom=oom_protection)

        result, status = monkey_util.connect()
        if result == Global.FAILURE:
            raise AcsToolException(AcsToolException.OPERATION_FAILED, "Cannot start monkey util: %s" % status)
        return monkey_util

