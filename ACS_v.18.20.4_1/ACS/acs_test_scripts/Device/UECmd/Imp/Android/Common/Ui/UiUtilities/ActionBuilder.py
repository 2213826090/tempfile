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

import random
import copy

from ErrorHandling.AcsConfigException import AcsConfigException
from UiAction import UiAction
from DictionaryParser import DictionaryParser
from UiUtilitiesFactory import UiUtilitiesFactory
from UiUtilities import XmlScriptParser


class ActionBuilder(object):

    """
    This class is intended for C{Action} instantiation.
    It uses an C{XmlScriptParser} and a C{UiUtilitiesFactory}
    in order to build the actions.

    The goal of this class is to add some specific I{Actions}
    parameters processing when needed and to decrease the
    complexity induced by I{operation sets} with parameter
    that can come either from:
    - UI sequence
    - operation set

    This class shall provide one C{init_..._action()} method
    for each action that:
    - is part of an operation set
    - and takes a parameter from an operation set
    In that case the C{NAMES_AND_METHODS} class attribute
    dictionary shall contain a key indicating the I{action}'s
    short name with the corresponding value being the
    name of the initialization method that will be used
    to initialize this action.
    """

    # The matching between action names and the corresponding
    # initialization methods
    NAMES_AND_METHODS = {
        "keydown": "_init_key_down_action",
        "keypress": "_init_key_press_action",
        "keyup": "_init_key_up_action",
        "type": "_init_type_action",
        "wait": "_init_wait_action",
    }

    def __init__(self, factory=None, device=None):
        """
        Constructor.

        :type device: DeviceModel
        :param device: the device model to be used by this object.

        :rtype: UiUtilitiesFactory
        :param factory: the factory instance to used.
        """
        self.__parser = None
        self.__factory = factory
        self.__last_random_cmd = None
        self.__iter_random_array = None
        self.__cmd_list = None
        self.__dictionary_parser = None
        self.__device = device

    def set_parser(self, parser):
        """
        Sets this object I{parser} to the given instance.

        :type parser: XmlScriptParser
        :param parser: the new I{parser} instance.
        """
        self.__parser = parser

    def get_parser(self):
        """
        Returns this object I{parser} instance.

        :rtype: XmlScriptParser
        :return: the parser instance
        """
        return self.__parser

    def get_action_list(self):
        """
        Returns the I{action list} built during the previous
        call to C{build_action_list_from_script}.
        """
        return self.__cmd_list

    def get_device(self):
        """
        Return this factory's configured device instance.
        :rtype: DeviceModel
        :return: the device instance used by this object.
        """
        if self.__device is None:
            # Try to retrieve a device instance from the factory
            self.__device = self.__factory.get_device()
        return self.__device

    def get_dictionary_parser(self):
        """
        Sets the operation set dictionary parser to use.
        :rtype: DictionaryParser
        :param factory: the dictionary parser instance to use.
        """
        # We implement lazy initialization because
        # the XML file is parsed at instantiation time
        # of DictionaryParser.
        if self.__dictionary_parser is None:
            self.__dictionary_parser = DictionaryParser(self.get_device())
            # Parse the OpDictionary
            self.__dictionary_parser.parse_operation_config()

        return self.__dictionary_parser

    def set_factory(self, factory):
        """
        Sets the factory to use to the given instance.
        :rtype: UiUtilitiesFactory
        :param factory: the factory instance to use.
        """
        self.__factory = factory

    def get_factory(self):
        """
        Returns the factory instance used by this object.
        :rtype: UiUtilitiesFactory
        :return: the factory
        """
        return self.__factory

    def build_action_list_from_script(self, script_path):
        """
        Builds and returns the list of actions described in
        the given XML I{script} file.

        This object's I{action list} is updated and can be
        retrieved later.

        :type script_path: str
        :param script_path: the path to the XML file

        :rtype: list
        :return: the list of actions
        """
        # Initialize local variables
        self.set_parser(XmlScriptParser(script_path))
        cmd_nodes = self.get_parser().parse()
        cmd_list = []

        # OperationSet on action nodes
        for cmd in cmd_nodes:
            if cmd[0] == "ACTION":
                action = self.handle_action_node(cmd[1])
                cmd_list.extend(action)
            elif cmd[0] == "ITERATION":
                iteration = self.handle_iteration_node(cmd[1])
                cmd_list.append(iteration)

        self.__cmd_list = cmd_list

        # Return the action list as a convenience method.
        return cmd_list

    def build_operation_set(self,
                            action_name,
                            parameters=None,
                            timeout=None,
                            iterations=None,
                            triglog=None,
                            stop_iter_on_first_success=False):
        """
        Returns a C{OperationSet} instance initialized with
        the given parameters.

        :type action_name: str
        :param action_name: the action's short name
        :type parameters: list
        :param parameters: the list of parameters for this action
        :type timeout: float
        :param timeout: the timeout value for the action
        :type iterations: int
        :param iterations: the iteration count for the action
        :type triglog: str
        :param triglog: the LogCat trigger tag.

        :rtype: OperationSet
        :return: the C{OperationSet} list corresponding to the given operation set.
        """
        operation_set = \
            self._handle_operation_set(action_name,
                                       parameters,
                                       timeout,
                                       iterations,
                                       triglog,
                                       stop_iter_on_first_success)
        # Return the action list as a convenience
        return operation_set

    def handle_iteration_node(self, iteration_node):
        """
        Returns an C{OperationSet} instance built from the given
        XML action node.

        :param iteration_node: the XML node corresponding to the action
        :type iteration_node: NodeList

        :rtype: OperationSet
        :return: the corresponding {OperationSet} instance.
        """
        (iteration_number, actions) = self.__parser.parse_iteration_node(iteration_node)

        actions_list = []
        for action in actions:
            action_instance = self.handle_action_node(action[1])
            actions_list.extend(action_instance)

        iteration_instance = self.__factory.get_iteration(actions_list)
        iteration_instance.set_iteration_count(iteration_number)
        return iteration_instance

    def handle_action_node(self, action_node):
        """
        Returns an C{Action} instance built from the given
        XML action node.
        If the XML node corresponds to a simple action, then the
        list will contain only one element.
        It the XML node corresponds to an operation set, then the
        list will contain all the unitary actions corresponding
        to this operation set.

        :param action_node: the XML node corresponding to the action
        :type action_node: NodeList

        :rtype: list
        :return: the corresponding {Action} instances in a list.
        """
        action = self.__parser.parse_action_node(action_node)
        iterations = self.get_numeric_iteration_value(action["iteration"])
        simple_action_names = self.__factory.get_allowed_action_names()

        if action["command"] in simple_action_names:
            # Simple action
            # Returned value is a list
            action = self._handle_simple_action(
                action["command"],
                action,
                action["timeout"],
                iterations,
                action["triglog"],
                action["log"],
                action["stop_iteration_on_success"])
        else:
            # Operation set
            action = self._handle_operation_set(
                action["command"],
                action["parameters"],
                action["timeout"],
                iterations,
                action["triglog"],
                action["stop_iteration_on_success"])
        actions = [action]
        return actions

    def _handle_simple_action(
            self,
            action_name,
            parameters,
            timeout,
            iterations,
            triglog,
            log_message,
            stop_iteration_on_first_success):
        """
        Returns a UiAction subclass instance from the given parameters.

        :type action_name: str
        :param action_name: the action's short name
        :type parameters: list
        :param parameters: the list of parameters for this action
        :type timeout: float
        :param timeout: the timeout value for the action
        :type iterations: int
        :param iterations: the iteration count for the action
        :type triglog: str
        :param triglog: the LogCat trigger tag.

        :rtype: UiAction
        :return: a C{UiAction} instance built from the given parameters
        """
        action_instance = self.__factory.get_action_for_name(
            action_name,
            parameters,
            timeout)
        action_instance.set_iteration_count(iterations)
        action_instance.set_trigger_log(triglog)
        action_instance.set_log_message(log_message)
        action_instance.set_stop_iter_on_first_success(stop_iteration_on_first_success)
        return action_instance

    def _get_next_operation_set_parameter(self, op_set_parameters):
        """
        Returns the new value to use as I{operation set} parameter when
        building a new action.
        This method actually changes the given list content.

        :type op_set_parameters: list
        :param op_set_parameters: the parameter list

        :rtype: object
        :return: the new value to use when building action or C{None}
        """
        new_parameter_value = None
        if op_set_parameters:
            new_parameter_value = op_set_parameters.pop(0)
        return new_parameter_value

    def _handle_operation_set(
            self,
            op_set_name,
            parameters,
            timeout,
            iterations,
            triglog,
            stop_iteration_on_first_success):
        """
        :type op_set_name: str
        :param op_set_name: the operation set's short name
        :type parameters: list
        :param parameters: the list of parameters for this action
        :type timeout: float
        :param timeout: the timeout value for the action
        :type iterations: int
        :param iterations: the iteration count for the action
        :type triglog: str
        :param triglog: the LogCat trigger tag.

        :rtype: list
        :return: the C{UiAction} list corresponding to the given
            operation set.
        """

        # Initialize local variables
        parser = self.get_dictionary_parser()
        operation_set_parameters = parser.retrieve_operation_set(op_set_name)
        specific_actions = ActionBuilder.NAMES_AND_METHODS.keys()
        action_list = []
        # Default iteration for simple actions
        default_iteration_count = 1
        # Ensure that parameters is actually a list
        if not isinstance(parameters, list):
            parameters = [parameters]
        # Parameters copy
        parameters_copy = copy.deepcopy(parameters)
        # Iterate on the actions represented as lists
        for action in operation_set_parameters["operations"]:
            action = action.copy()
            action_name = action["command"]
            if action_name in specific_actions:
                # We have a specific initialization method for this one
                initialization_method = self._get_initialization_method_for_name(action_name)
                # We create the action
                action_instance = getattr(self, initialization_method)(action, parameters_copy, action.get('timeout'))
                action_instance.set_trigger_log = action["triglog"]
                action_instance.set_log_message(action["log"])
            else:
                # Simple action
                action_timeout = action.get("timeout")
                # timeout must be different from None, empty str or 0 values
                if not action_timeout:
                    action_timeout = self.__factory.get_type_timeout()
                action_timeout = float(action_timeout)
                action_instance = self._handle_simple_action(
                    action_name,
                    action,
                    action_timeout,
                    default_iteration_count,
                    action["triglog"],
                    action["log"],
                    stop_iteration_on_first_success)
            # Add the action to the list
            action_list.append(action_instance)
        # Create the operation set
        operation_set = self._init_operation_set(action_list)
        operation_set.set_iteration_count(iterations)
        operation_set.set_name(op_set_name)
        # TIMEOUT
        if timeout:
            (timeout, fail_on_timeout) = operation_set.split_timeout(timeout)
            operation_set.set_timeout(timeout)
            operation_set.set_fail_on_timeout(fail_on_timeout)
        elif operation_set_parameters.get("timeout", ""):
            #get value specified value from ui dictionary
            operation_set.set_timeout(operation_set_parameters["timeout"])
        else:
            operation_set.set_timeout(self.__factory.get_type_timeout())

        # TRIGLOG
        if triglog is not None:
            operation_set.set_trigger_log(triglog)
        else:
            operation_set.set_trigger_log(operation_set_parameters.get("triglog"))
        operation_set.set_stop_iter_on_first_success(stop_iteration_on_first_success)
        # Return the action list
        return operation_set

    def _get_initialization_method_for_name(self, name):
        """
        Returns the class corresponding to the given name
        """
        method = None
        try:
            method = ActionBuilder.NAMES_AND_METHODS[name]
        except KeyError:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Invalid action name: %s" % (str(name)))
        return method

    def _init_type_action(
            self,
            action,
            operation_set_parameters,
            timeout=None):
        """
        Initializes and return a new C{TypeAction} instance with:
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
        return factory.get_type_action(action, timeout)

    def _init_key_press_action(
            self,
            action,
            operation_set_parameters,
            timeout=None):
        """
        Initializes and return a new C{KeyPressAction} instance with:
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

        :rtype: KeyPressAction
        :return: a new corresponding C{UiAction} instance.
        """
        factory = self.get_factory()
        if not action["parameters"]:
            action["parameters"] = []
        if len(action["parameters"]) == 0:
            action["parameters"].append(operation_set_parameters.pop(0))
        return factory.get_key_press_action(action, timeout)

    def _init_wait_action(
            self,
            action,
            operation_set_parameters,
            timeout=None):
        """
        Initializes and return a new C{WaitAction} instance with:
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

        :rtype: WaitAction
        :return: a new corresponding C{UiAction} instance.
        """
        factory = self.get_factory()
        if not action["parameters"]:
            action["parameters"] = []
        if len(action["parameters"]) == 0:
            action["parameters"].append(operation_set_parameters.pop(0))
        return factory.get_wait_action(action, timeout)

    def _init_key_up_action(
            self,
            action,
            operation_set_parameters,
            timeout=None):
        """
        Initializes and return a new C{KeyUpAction} instance with:
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

        :rtype: KeyUpAction
        :return: a new corresponding C{UiAction} instance.
        """
        factory = self.get_factory()
        if not action["parameters"]:
            action["parameters"] = []
        if len(action["parameters"]) == 0:
            action["parameters"].append(operation_set_parameters.pop(0))
        return factory.get_key_up_action(action, timeout)

    def _init_operation_set(self, actions):
        """
        Initializes and returns a new C{OperationSet}.

        :type actions: list
        :param actions: the I{operation set}'s actions

        :rtype: OperationSet
        :return: an new I{operation set}.
        """
        factory = self.get_factory()
        operation_set = factory.get_operation_set(actions)
        return operation_set

    def _init_key_down_action(
            self,
            action,
            operation_set_parameters,
            timeout=None):
        """
        Initializes and returns a new C{KeyDownAction} instance with:
            - standard action parameters
            - parameters that may be provided by an I{operation set}
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
            parameters for the whole B{operation set}.

        :type timeout: int
        :param timeout: the optional timeout to apply for this action.

        :rtype: KeyDownAction
        :return: a new corresponding C{UiAction} instance.
        """
        factory = self.get_factory()
        if not action["parameters"]:
            action["parameters"] = []
        if len(action["parameters"]) == 0:
            action["parameters"].append(operation_set_parameters.pop(0))
        return factory.get_key_down_action(action, timeout)

    def get_numeric_iteration_value(self, iteration):
        """
        Returns a numeric value built from the given C{iteration} parameter.
        The C{iteration} parameter can be either one of these:
        - C{int} value
        - C{str} value corresponding to an integer
        - C{str} an expression indicating a random value, e.g:
            - [RAND(1, 8)] : a random value between 1 and 8
        :param iteration: the iteration expression (as described above)
        :type iteration: str

        :rtype: int
        :return: an integer value built from the given parameter
        """
        iteration_value = 1
        if iteration is not None:
            if isinstance(iteration, str) and iteration.startswith("[RAND("):
                            # Random type => generate it
                iteration = iteration.strip("[RAND(")
                iteration = iteration.strip(")]")

                if iteration == self.__last_random_cmd:
                    random_count = len(self.__iter_random_array)
                    if self.__iter_random_array is None or random_count == 0:
                        self.__iter_random_array = range(int(iteration))
                else:
                    self.__iter_random_array = range(int(iteration))
                    self.__last_random_cmd = iteration

                iteration_value = random.choice(self.__iter_random_array)
                self.__iter_random_array.remove(iteration_value)
            else:
                # Std type, get it
                iteration_value = int(iteration)
        return iteration_value

