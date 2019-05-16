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
from UtilitiesFWK.Utilities import Global
from UiAction import UiAction
from OperationSet import OperationSet
from UiUtilities import run_actions


class UiActionRunner(object):

    """
    A class that is able to run actions.
    """

    def __init__(self, logger, logcat_logger, time_between_operation=0):
        """
        Constructor.
        """
        self._logger = logger
        self._logcat_logger = logcat_logger
        self.__error_manager = None
        self.__wait_btwn_op = time_between_operation

    def get_error_manager(self):
        """
        Returns the C{UiErrorManager} instance
        used by this object.
        """
        return self.__error_manager

    def set_error_manager(self, error_manager):
        """
        Sets the C{UiErrorManager} used by this object
        to the given instance.
        :type error_manager:  UiErrorManager
        :param error_manager: the new C{UiErrorManager}
        """
        self.__error_manager = error_manager

    def log_action(self, action, index=1):
        """
        Logs information about the given C{action}.
        """
        if isinstance(action, OperationSet):
            self.log_operation_set(action, index)
        else:
            self.log_simple_action(action, index)

    def log_simple_action(self, action, index=1):
        """
        Logs the given action as a simple I{action}.

        :type index: int
        :param index: the iteration number

        :type action: UiAction
        :param action: the I{action} to run
        """
        log_message = action.get_log_message()
        if log_message:
            log_message = "(%s)" % log_message
        else:
            log_message = ""

        msg = "USE CASE cmd  : %s %s, timeout (%.2f)" % (action.get_name(), log_message, action.get_timeout())

        if index > 1:
            msg += " (x %d)" % index

        self._logger.info(msg)

        parameters = action.get_parameters()
        if parameters is not None:
            for parameter in parameters:
                self._logger.info("USE CASE param: %s" % str(parameter))

    def log_operation_set(self, action, index=1):
        """
        Logs the given action as an I{operation set}.

        :type index: int
        :param index: the iteration number

        :type action: OperationSet
        :param action: the I{operation set} to run
        """
        classname = str(action.__class__.__name__)
        op_set_name = action.get_name()

        msg = "USE CASE cmd  : %s, timeout (%.2f)" % (classname, action.get_timeout())
        if op_set_name:
            msg += " (%s)" % op_set_name
        if index > 1:
            msg += " (x %d)" % index

        self._logger.info(msg)

    def check_action_result(self, action):
        """
        Checks the result of the given I{action}.

        Not valid results are:
            - Global.FAILURE
        :type action: UiAction
        :param action: the action to check.

        :raise DeviceException: if the result is not correct.
        """
        error_manager = self.get_error_manager()
        return error_manager.check_action_result(action)

    def check_logcat(self, action, timeout):
        """
        Checks that the trigger message from the given action
        has actually been received, until timeout seconds.
        If action as no trig log, it will wait for timeout seconds

        :type action: UiAction
        :param action: the C{action} from wich the trigger message
            will be retrieved.
        """
        trig_log = action.get_trigger_log()
        message = []

        if trig_log is not None:
            ack_message = False
            ack_message_number = 0

            self._logger.debug("Checking UI action in logcat")
            start_time = time.time()

            triggered_log = {}
            while ((time.time() - start_time) < float(timeout)) and not ack_message:
                ack_message_number = 0
                for log in trig_log:
                    messages = \
                        self._logcat_logger.get_message_triggered_status(log)
                    triggered_log[log] = list(messages)
                    if messages:
                        if action.get_trig_logs_condition() == "AND":
                            ack_message_number += 1
                        elif action.get_trig_logs_condition() == "OR":
                            ack_message_number += 1
                            break
                        else:
                            ack_message_number += 1

                    time.sleep(0.5)

                if action.get_trig_logs_condition() == "AND" and\
                   ack_message_number >= len(trig_log):
                    # AND condition, and we got all trig log
                    ack_message = True
                elif ack_message_number > 0 and\
                        action.get_trig_logs_condition() != "AND":
                    # no logs contion or OR condition
                    # as soon we got a message, that's ok
                    ack_message = True

            # Adapt printout to each situation:
            # With an OR condition: TRIG_1 OR TRIG_2
            # TRIG_1 = OK; No check on TRIG_2
            # TRIG_1 = NOK; TRIG_2 = OK; Warning on TRIG_1
            # TRIG_1 = NOK; TRIG_2 = NOK, Warning on TRIG_1, Error on TRIG_2
            is_first_log = True
            for log in triggered_log.keys():
                if not triggered_log[log] and action.get_trig_logs_condition() == "OR" and is_first_log:
                    msg = "DID NOT TRIG: \"%s\"" % log
                    self._logger.warning(msg)
                    message.append(msg)
                    is_first_log = False
                elif not triggered_log[log]:
                    msg = "DID NOT TRIG: \"%s\"" % log
                    self._logger.error(msg)
                    message.append(msg)
                else:
                    msg = "TRIG: \"%s\"" % (log)
                    self._logger.info(msg)
                    msg = "TRIG: \"%s\" IN %s" % (log, triggered_log[log])
                    self._logger.debug(msg)

            if not ack_message and action.get_fail_on_timeout():
                action._set_result(Global.FAILURE)  # pylint: disable=W0212
                result, output = self.check_action_result(action)
                message.append(output)
            else:
                result = Global.SUCCESS
                message = "Success"
        else:
            #no need to wait
            result, output = self.check_action_result(action)
            message.append(output)

        return result, message

    def _prepare(self, action):
        """
        Runs I{pre-execution} tasks on the action before execution
        """
        action_class_name = action.__class__.__name__

        # Add trig log if needed
        triglog = action.get_trigger_log()
        if triglog:
            for log in triglog:
                self._logcat_logger.add_trigger_message(log)

        if action_class_name == "OperationSet":
            action.set_action_runner(self)


    def end_logcat(self, action):
        """
        Asks the I{log cat reader} to delete the trigger message.

        :type action: UiAction
        :param action: the C{action} from wich the trigger message
            will be retrieved.
        """
        triglog = action.get_trigger_log()
        # If we are triggering logcat delete it
        if triglog:
            for log in triglog:
                self._logcat_logger.remove_trigger_message(log)


    def run_action(self, action, sleep_before_action=True):
        """
        Runs the given C{action}.
        :type action: UiAction
        :param action: the C{action} to run
        """
        final_result = Global.SUCCESS
        error_message = []
        # Log the action
        self.log_action(action)
        # Prepare the action
        self._prepare(action)
        # Add tempo before action
        if sleep_before_action:
            LOGGER_TEST_SCRIPT.debug("Time between operations: %s" % self.__wait_btwn_op)
            time.sleep(self.__wait_btwn_op)
            # Run the action once
        final_result |= action.do()
        # Check the result
        result, output = self.check_action_result(action)
        final_result |= result
        if result == Global.SUCCESS:
            result, output = self.check_logcat(action, action.get_timeout())
            final_result |= result
            error_message.extend(output)
        else:
            error_message.append(output)

        # Check whether some more iterations are required
        index = 1
        iterations = action.get_iteration_count()
        # reevaluate sleep for next iterations
        sleep_before_action = action.SLEEP_BEFORE_EXECUTION
        # Iterate on the action
        while index < iterations:
            if action.get_stop_iter_on_first_success() and result == Global.SUCCESS:
                break

            index += 1
            self.log_action(action, index)
            if sleep_before_action:
                LOGGER_TEST_SCRIPT.debug("Time between operations: %s" % self.__wait_btwn_op)
                time.sleep(self.__wait_btwn_op)
            final_result |= action.redo()

            result, output = self.check_action_result(action)
            final_result |= result

            if result == Global.SUCCESS:
                result, output = self.check_logcat(action, action.get_timeout())
                final_result |= result
                error_message.extend(output)
            else:
                error_message.append(output)

        # End all logcat triggers
        self.end_logcat(action)

        # Return the result
        if final_result == Global.FAILURE:
            output = error_message

        return (final_result, output)


    def run_actions(self, actions):
        """
        Runs all actions that are contained in the given C{iterable}.
        :type actions: list
        :param actions: the list of actions to execute
        """
        return run_actions(actions, self, False)
