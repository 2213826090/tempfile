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
import lxml.etree as et
import UtilitiesFWK.Utilities as Utils

from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global
from UiAction import UiAction
from WaitAction import WaitAction


def run_actions(actions, action_runner, stop_on_failure=False):
    """
    Runs all actions that are contained in the given C{iterable}.
    :type actions: list
    :param actions: the list of actions to execute
    """
    final_result = Global.SUCCESS
    message = ""

    # Check whether actions is a list or not
    if type(actions) not in (list, tuple):
        need_sleep = actions.SLEEP_BEFORE_EXECUTION
        # Run actions as a single action
        (final_result, message) = action_runner.run_action(action=actions, sleep_before_action=need_sleep)
        if stop_on_failure and final_result != Global.SUCCESS:
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, message)
    else:
        # Run actions as a list
        error_message = []
        last_action_pos = len(actions) - 1
        for pos, action in enumerate(actions):
            # if action is a wait, no need to sleep
            need_sleep = action.SLEEP_BEFORE_EXECUTION
            if pos > 0 and last_action_pos > 1:
                # no need to sleep if previous action is a wait
                need_sleep = need_sleep and not isinstance(actions[pos - 1], WaitAction)
            (status, message) = action_runner.run_action(action, need_sleep)
            if status != Global.SUCCESS:
                if stop_on_failure:
                    raise DeviceException(DeviceException.OPERATION_SET_ERROR, message)
                else:
                    error_message.extend(message)
            final_result = final_result | status

        if final_result == Global.FAILURE:
            message = error_message

    return final_result, message


class RandomRange(object):

    """
    This class is intended for random values and range
    expressions management.
    """
    previous_range_exp = None
    my_range = None

    @classmethod
    def get_value_from_range(cls, range_expression):
        """
        Returns an I{int} build from the given range expression.
        The range_expression can be expressed in the following way:
            - [RAND(1,7)]: a random value from 1 to 7
            - [RAND(0,5,2)]: a random value between 0 to 5 with step of 2
                             0, 2, 4 for this exemple

        :type range_expression: str
        :param range_expression: the range expression.

        :rtype: int
        :return: a random value contained in the given range.
        """

        if RandomRange.previous_range_exp != range_expression or\
           not RandomRange.my_range:
            RandomRange.previous_range_exp = range_expression
            range_expression = range_expression.strip("[RAND(")
            range_expression = range_expression.strip(")]")
            range_expression = range_expression.split(",")

            if len(range_expression) == 3:
                RandomRange.my_range = range(int(range_expression[0]),
                                             int(range_expression[1]) + 1,
                                             int(range_expression[2]))
            elif len(range_expression) == 2:
                RandomRange.my_range = range(int(range_expression[0]),
                                             int(range_expression[1]) + 1)
            else:
                RandomRange.my_range = [0]

        choice = random.choice(RandomRange.my_range)
        RandomRange.my_range.remove(choice)
        return choice


class ActionResultListener(object):

    """
    A class that implements action listeners.
    """

    def __init__(self):
        """
        Constructor.
        """
        self.__previous = None
        self.__result = None

    def result_changed(self, action):
        """
        Changes the listened action's result.
        :type action: UiAction
        :param action: the action that this object listens to
        :rtype: None
        """
        self.__previous = self.__result
        self.__result = action.get_result()

    def get_result(self):
        """
        Returns this listener's last result.
        :rtype: object
        :return: the last result
        """
        return self.__result

    def get_previous(self):
        """
        Returns this listener's last result.
        :rtype: object
        :return: the previous result
        """
        return self.__previous


class UiErrorManager(object):

    """
    This is a base class for error management in the UI Framework.
    """

    COMMANDS_TO_CHECK = (
        "LongTouchAction", "TouchAction",
        "ScreenshotAction", "WakeAction",
        "KeyPressAction", "KeyDownAction",
        "KeyUpAction", "OperationSet",
        "ExecAction", "ReplayAction",
        "DoubleTouchAction")

    def __init__(self, logger):
        """
        Initializes this instance.
        """
        self.__logger = logger

    def get_logger(self):
        """
        Returns the logger instance used by this object.
        """
        return self.__logger

    def check_action_result(self, action):
        """
        A method that shall be overriden by subclasses.
        Checks the result of the given I{action} and
        perform the expected operations.
        """
        pass


class RaiseExceptionErrorManager(UiErrorManager):

    """
    Implement an error management that raises exceptions when
    error occurs.
    """

    def __init__(self, logger):
        """
        Initializes this instance.
        """
        UiErrorManager.__init__(self, logger)

    def check_action_result(self, action):
        """
        A method that shall be overriden by subclasses.
        Checks the result of the given I{action} and
        perform raises an exception if the action
        result was a failure.

        :raise DeviceException: if the given action's result was a failure.
        """
        # Initialize local variables
        logger = self.get_logger()
        # Get the action result
        result = action.get_result()
        # Get the action name
        command = str(action.__class__.__name__)
        # Check the error
        if command in UiErrorManager.COMMANDS_TO_CHECK:
            # Process error
            # Cannot import Global.FAILURE use hard-wired value instead
            if result == -1:
                msg = "%s execution failed." % action.get_name()
                logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
            else:
                return Global.SUCCESS, "Success"
        elif result is None:
            return Global.SUCCESS, "Success"
        else:
            return result, "No check requested"


class LogWarningErrorManager(UiErrorManager):

    """
    Implement an error management that raises exceptions when
    error occurs.
    """

    def __init__(self, logger):
        """
        Initializes this instance.
        """
        UiErrorManager.__init__(self, logger)

    def check_action_result(self, action):
        """
        A method that shall be overriden by subclasses.
        Checks the result of the given I{action} and
        perform raises an exception if the action
        result was a failure.

        :raise DeviceException: if the given action's result was a failure.
        """
        # Initialize local variables
        logger = self.get_logger()
        # Get the action result
        result = action.get_result()
        # Get the action name
        command = str(action.__class__.__name__)
        # Check the error
        if command in UiErrorManager.COMMANDS_TO_CHECK:
            # Process error
            if result == Global.FAILURE:
                msg = "%s execution failed." % action.get_name()
                logger.error(msg)
                return Global.FAILURE, msg
            else:
                return Global.SUCCESS, "Success"
        elif result is None:
            return Global.SUCCESS, "Success"
        else:
            return result, "No check requested"


class XmlScriptParser(object):

    """
    This class is intended for XML parsing.
    """

    def __init__(self, script_path):
        """
        Constructor.
        """
        self.__script = script_path
        self.__doc = None

    def parse(self):
        try:
            parsed_doc = et.parse(self.__script)
            self.__doc = parsed_doc.getroot()
        except et.XMLSyntaxError:
            _, error_msg, _ = Utils.get_exception_info()
            raise AcsConfigException(AcsConfigException.XML_PARSING_ERROR, error_msg)
        return self.__parse_cmds(self.__doc)

    def __parse_cmds(self, node):
        nodes = []

        for child in node:
            if child.tag in ["ACTION", "ITERATION"]:
                nodes.append((child.tag, child))
        return nodes

    def __get_text(self, nodelist):
        """
        Returns the concatenated text of the given XML nodes.
        :rtype: list
        :param nodelist: the XML node list

        :rtype: str
        :return: the str representation of the nodes
        """
        rc = []
        for node in nodelist:
            rc.append(node.text)
        return ''.join(rc)

    def get_action_nodes(self):
        """
        Returns the list of all I{ACTION} XML nodes.
        :rtype: NodeList
        :return: the list of all I{ACTION} XML nodes
        """
        ui_seq = None
        actions = []
        try:
            ui_seq = et.parse(self.__script)
        except et.XMLSyntaxError:
            _, error_msg, _ = Utils.get_exception_info()
            raise AcsConfigException(AcsConfigException.XML_PARSING_ERROR, error_msg)
        actions = ui_seq.xpath("//ACTION")
        return actions

    def parse_iteration_node(self, iteration_node):
        """
        Returns a C{tuple} containing all relevant data from the
        given C{iteration_node} parameter.

        The returned tuple contains the following elements:
            - iteration
            - actions list

        :type iteration_node: NodeList
        :param iteration_node: the XML node

        :rtype: tuple
        :return: the data from the given XML node
        """
        iteration = int(iteration_node.get("nb", 0))
        actions = self.__parse_cmds(iteration_node)

        return (iteration, actions)

    def parse_action_node(self, action_node):
        """
        Returns a C{dictionnary} containing all relevant data from the
        given C{action_node} parameter.

        The returned dictionnary contains the following elements:
            - command
            - parameters
            - timeout
            - triglog
            - iteration

        :type action_node: NodeList
        :param action_node: the XML node

        :rtype: dictionnary
        :return: the data from the given XML node
        """
        action = {}
        # Get the action name
        action["command"] = None
        nodes_cmd = action_node.xpath("./CMD")
        if nodes_cmd:
            action_name = self.__get_text(nodes_cmd)
            action["command"] = str(action_name).strip()
        else:
            raise AcsConfigException(
                AcsConfigException.XML_PARSING_ERROR,
                "Did not find the CMD node of "
                "the action, check your script")

        # Get the action parameters
        action["parameters"] = None
        nodes_param = action_node.xpath("./PARAMETERS")
        if nodes_param:
            parameters = self.__get_text(nodes_param)
            action["parameters"] = parameters.split(';')

        # Get the timeout
        action["timeout"] = None
        nodes_timeout = action_node.xpath("./TIMEOUT")
        if nodes_timeout:
            timeout = self.__get_text(nodes_timeout)
            action["timeout"] = str(timeout).strip()

        # Get the trigger tag
        action["triglog"] = None
        nodes_triglog = action_node.xpath("./TRIG_LOG")
        if nodes_triglog:
            triglog = self.__get_text(nodes_triglog)
            action["triglog"] = str(triglog).strip()

        # Get the log tag
        action["log"] = None
        nodes_log = action_node.xpath("./LOG")
        if nodes_log:
            log = self.__get_text(nodes_log)
            action["log"] = str(log).strip()

        # Get the iteration count
        action["iteration"] = None
        nodes_iterations = action_node.xpath("./ITERATION")
        if nodes_iterations:
            iteration = self.__get_text(nodes_iterations)
            action["iteration"] = str(iteration).strip()

        # Get the stop iteration on success parameter
        action["stop_iteration_on_success"] = False
        nodes_stop_iter_on_1st_success = action_node.xpath("./STOP_ITER_ON_FIRST_SUCCESS")
        if nodes_stop_iter_on_1st_success:
            stop_iteration_on_success = self.__get_text(nodes_stop_iter_on_1st_success)
            action["stop_iteration_on_success"] = Utils.str_to_bool(str(stop_iteration_on_success).strip())

        return action
