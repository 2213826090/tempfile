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

import time

from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global
from UiAction import UiAction
from UiUtilities import run_actions


class OperationSet(UiAction):

    """
    This class implements an I{operation set}.
    """

    SLEEP_BEFORE_EXECUTION = False

    def __init__(self, monkey, actions, time_between_operation=0):
        """
        Initializes this instance.
        """
        UiAction.__init__(self, monkey, 0)
        self.__action_runner = self
        self.__actions = actions
        self.__name = None
        self.__time_btwn_op = time_between_operation

    def get_name(self):
        """
        Returns this operation set's name.
        :rtype: str
        :return: the name
        """
        return self.__name

    def set_name(self, name):
        """
        Sets this operation set's name to the given value.

        :type name: str
        :param name: the new name
        """
        self.__name = name

    def get_actions(self):
        """
        Returns this object's associated I{actions}.

        :rtype: list
        :return: this object's C{UiAction} list
        """
        return self.__actions

    def can_redo(self):
        """
        Returns a C{bool} indicating whether this action can
        be re-done or not.
        :rtype: bool
        :return: C{True}
        """
        return True

    def can_undo(self):
        """
        Returns a C{bool} indicating whether this action can
        be undone or not.
        :rtype: bool
        :return: C{False}
        """
        return False

    def _redo(self):
        """
        Re-runs this operation set.
        """
        return self._do()

    def _do(self):
        """
        Runs this I{operation set}.
        """
        # Get the object responsible of action execution
        runner = self.get_action_runner()
        # Get the list of actions to run
        actions = self.get_actions()
        # Iterate on actions
        final_result, _ = run_actions(actions, runner, False)

        self._set_result(final_result)

        return final_result

    def get_action_runner(self):
        """
        Returns the I{action runner} to use for action execution.
        """
        if self.__action_runner is None:
            self.__action_runner = self
        return self.__action_runner

    def set_action_runner(self, action_runner):
        """
        Sets the I{action runner} to use for action execution
        to the given instance.

        :type action_runner: UiActionRunner
        :param action_runner: the I{action runner} to use
        """
        self.__action_runner = action_runner

    def check_action_result(self, action):
        """
        Checks the result of the given I{action}.

        Not valid results are:
            - Global.FAILURE
        :type action: UiAction
        :param action: the action to check.

        :raise DeviceException: if the result is not correct.
        """
        result = action.get_result()
        if result == Global.FAILURE:
            raise DeviceException(DeviceException.OPERATION_FAILED, "The %s execution failed." % (action.get_name()))

    def run_action(self, action, sleep_before_action=True):
        """
        Runs the given action.

        :type action: UiAction
        :param action: the action to execute
        """
        final_result = Global.SUCCESS
        # Add tempo before action
        if sleep_before_action:
            LOGGER_TEST_SCRIPT.debug("Time between operations: %s" % self.__wait_btwn_op)
            time.sleep(self.__wait_btwn_op)
        # Run the action once
        action.do()
        # Check the result
        action, result = self.check_action_result(action)
        final_result = final_result | result
        # Check whether some more iterations are required
        index = 1
        iterations = action.get_iteration_count()
        # reevaluate sleep for next iterations
        sleep_before_action = action.SLEEP_BEFORE_EXECUTION
        # Iterate on the action
        while index < iterations:
            if self.get_stop_iter_on_first_success() and result == Global.SUCCESS:
                break

            index += 1
            if sleep_before_action:
                LOGGER_TEST_SCRIPT.debug("Time between operations: %s" % self.__wait_btwn_op)
                time.sleep(self.__wait_btwn_op)
            action.redo()
            action, result = self.check_action_result(action)
            final_result = final_result | result

        # Return the result
        return (action, result)

    def run_actions(self, actions):
        """
        Runs the given actions.

        :type action: list
        :param action: the list of action to execute
        """
        # Check whether actions is a list or not
        return run_actions(actions, self, True)
