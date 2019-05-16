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
:summary: UECmd based file for all Android Application Framework BaseV2
:since: 07/03/2013
:author: vdechefd
"""
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Base import Base
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsToolException import AcsToolException
from UtilitiesFWK.Utilities import str_to_bool


class BaseV2(Base):

    """
    Class that handle all generic operations
    Using Acs Agent v2
    """
    STATE = "STATE"
    """
    Key from embedded that represent the state of an async uecmd.
    """
    FINISHED = "FINISHED"
    """
    State when an async uecmd is finished.
    """
    ERROR = "ERROR"
    """
    State when an async uecmd has failed.
    """
    CANCELLED = "CANCELLED"
    """
    State when an async uecmd has been cancelled.
    """

    __is_agent_used = False

    def __init__(self, device):
        Base.__init__(self, device)
        self.__async_module = "acscmd.utils.AsyncModule"
        self.__autolog_module = "acscmd.em.autolog.AutologModule"

    def __format_intent_action_cmd(self, target_class, target_method, cmd_args, is_system=False):
        """

        :param target_class: Class to call in the acs agent
        :param target_method: Method to call in the acs agent
        :param cmd_args: Args needed by the ue command
        :param is_system: Is the command user or system

        :return: ue command chain string to call by run_command
        """
        BaseV2.__is_agent_used = True
        intent_action_cmd = self._device.get_acs_agent().get_intent_action_cmd(is_system)

        cmd = "-a {0} --es class {1} --es method {2} {3}".format(intent_action_cmd,
                                                                 target_class,
                                                                 target_method,
                                                                 cmd_args)

        return cmd

    def _internal_exec_with_retry_v2(self, target_class, target_method, cmd_args="",
                                     timeout=None, remove_triggered_message=True,
                                     op_code=None, max_retry=5, uecmd_error_type=None, is_system=False):
        """
        Call to _internal_exec_with_retry with Acs AgentV2 formalism
        """
        cmd = self.__format_intent_action_cmd(target_class, target_method, cmd_args, is_system)

        return self._internal_exec_with_retry(cmd, timeout, remove_triggered_message,
                                              op_code, True, False, max_retry, uecmd_error_type)

    def _internal_exec_multiple_v2(self, target_class, target_method, cmd_args="",
                                   timeout=None, remove_triggered_message=True, op_code=None, is_system=False):
        """
        Call to _internal_exec_multiple with Acs AgentV2 formalism
        """
        cmd = self.__format_intent_action_cmd(target_class, target_method, cmd_args, is_system)

        return self._internal_exec_multiple(cmd, timeout, remove_triggered_message, op_code, True, False)

    def _internal_exec_v2(self, target_class, target_method, cmd_args="",
                          timeout=None, remove_triggered_message=True, op_code=None, is_system=False):
        """
        Call to _internal_exec with Acs AgentV2 formalism
        """
        cmd = self.__format_intent_action_cmd(target_class, target_method, cmd_args, is_system)

        return self._internal_exec(cmd, timeout, remove_triggered_message, op_code, True, False)

    def finalize(self):
        if BaseV2.__is_agent_used and self._device.is_available():
            self._logger.debug("Finalizing UECommands ...")
            BaseV2.__is_agent_used = False
            cmd = "-a intel.intent.action.acs.finish_modules"
            return self._internal_exec(cmd, timeout=10, broadcast=True)
        else:
            return None

    def get_async_result(self, cmd_id):
        """
        Internal method that get ouptut of an async scheduled cmd

        :type  cmd_id: str
        :param cmd_id: task id to be read

        :rtype: dict
        :return: scheduled command output
        """
        output = {}
        file_path = self._task_folder + "/" + cmd_id
        cmd = "adb shell cat " + file_path
        tmp_output = self._exec(cmd)
        # case where we get result from local directory
        if self.is_shell_output_ok(tmp_output) and (tmp_output.find(self.FIELDS_SEPARATOR) != -1 and
                                                    tmp_output.find(self.PARAM_SEPARATOR) != -1):
            for element in tmp_output.split(self.FIELDS_SEPARATOR):
                if len(element) > 0:
                    value = element.split(self.PARAM_SEPARATOR)
                    if len(value) > 1:
                        # clean text with potential false separator
                        formated_key = value[0].replace(self.ALTERNATIVE_FIELDS_SEPARATOR,
                                                        self.FIELDS_SEPARATOR).\
                            replace(self.ALTERNATIVE_PARAM_SEPARATOR,
                                    self.PARAM_SEPARATOR)

                        formated_value = value[1].replace(self.ALTERNATIVE_FIELDS_SEPARATOR,
                                                          self.FIELDS_SEPARATOR).\
                            replace(self.ALTERNATIVE_PARAM_SEPARATOR,
                                    self.PARAM_SEPARATOR)
                        output[formated_key] = formated_value
            # deLete task file
            cmd = "adb shell rm " + file_path
            self._exec(cmd)

        elif not self.is_shell_output_ok(tmp_output):
            output = self._internal_exec_v2(self.__async_module, "getAsyncResult", "--es id %s" % cmd_id)

            for keys in output.keys():
                output[keys] = self._format_text_ouput(output[keys])

        else:
            output = {self.OUTPUT_MARKER: tmp_output}

        # if error key is found and your task
        if output.get(self.STATE) == self.ERROR:
            txt = "an error occur during async execution of task %s : %s" % (cmd_id, output[self.OUTPUT_MARKER])
            self._logger.error(txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, txt)

        return output

    def clean_daemon_files(self):
        """
        Method that delete all daemon files
        """
        self._internal_exec_v2(self.__async_module, "cancelAllTask")
        self._exec("adb shell rm %s/TASK_*" % self._task_folder, raise_error=False)
        # clean task generated in COS
        self._exec("adb shell rm /logs/*TASK_* /data/*TASK_*", raise_error=False)

    def start_auto_logger(self, first_delay, polling_delay, logger_type="sequenced"):
        """
        Internal method that start an autolog sequenced function to poll every x seconds

        :type  first_delay: int
        :param first_delay: delay before starting the first log.

        :type  polling_delay: int
        :param polling_delay: delay in seconds between 2 logs.

        :type  logger_type: str
        :param logger_type: logger type to start, support only 'sequenced'.
        """
        if logger_type == "sequenced":
            function = "startSequenceLogger"
        else:
            txt = "not supported logger type" % logger_type
            self._logger.error(txt)
            raise AcsToolException(AcsToolException.OPERATION_FAILED, txt)

        params = "--ei pollingDelay %s --ei startDelay %s" % (polling_delay, first_delay)
        self._internal_exec_v2(self.__autolog_module, function, params, is_system=True)

    def start_auto_logger_on_reboot(self, polling_delay, logger_type="sequenced"):
        """
        Internal method that start an autolog sequenced function to poll every x seconds
        only after next reboot

        :type  polling_delay: int
        :param polling_delay: delay in seconds between 2 logs.

        :type  logger_type: str
        :param logger_type: logger type to start, support only 'sequenced'.
        """
        if logger_type == "sequenced":
            function = "startSequenceLoggerOnReboot"
        else:
            txt = "not supported logger type" % logger_type
            self._logger.error(txt)
            raise AcsToolException(AcsToolException.OPERATION_FAILED, txt)

        params = "--ei pollingDelay %s" % (polling_delay)
        self._internal_exec_v2(self.__autolog_module, function, params, is_system=True)

    def stop_auto_logger(self, logger_type="sequenced"):
        """
        method that stop an autolog function.

        :type  logger_type: str
        :param logger_type: logger type to stop , only support 'sequenced'.
        """
        if logger_type == "sequenced":
            function = "stopSequenceLogger"
        else:
            txt = "not supported logger type" % logger_type
            self._logger.error(txt)
            raise AcsToolException(AcsToolException.OPERATION_FAILED, txt)
        self._internal_exec_v2(self.__autolog_module, function, is_system=True)

    def reset_running_log(self):
        """
        reset all data from running logs.
        Allow them to start from an empty file.
        """
        self._internal_exec_v2(self.__autolog_module, "resetRunningLog", is_system=True)

    def add_fct_to_auto_logger(self, method_name, logger_type="sequenced"):
        """
        add a method to a logger.
        This method will be used for logging once the logger started.

        :type  method_name: str
        :param method_name: name of a logger method in AutologModule.

        :type  logger_type: str
        :param logger_type: logger type to stop , only support 'sequenced'.
        """
        if logger_type == "sequenced":
            function = "addMethodToSequenceLogQueue"
        else:
            txt = "not supported logger type" % logger_type
            self._logger.error(txt)
            raise AcsToolException(AcsToolException.OPERATION_FAILED, txt)
        params = "--es name %s" % method_name
        self._internal_exec_v2(self.__autolog_module, function, params, is_system=True)

    def set_persistent_autolog(self, state):
        """
        activate/deactivate the used of persistent logs.
        persistent logs will restart if a reboot happen

        :type  state: boolean
        :param state: true to use persistent log, false otherwise
        """
        if state:
            state = 1
        else:
            state = 0
        params = "--ei persistentOverReboot %s" % state
        self._internal_exec_v2(self.__autolog_module, "setPersistentLogger", params, is_system=True)

    def set_autolog_duration(self, duration):
        """
        set the logging duration

        :type  state: int
        :param state: duration in second > 0
        """
        params = "--ei duration %s" % duration
        self._internal_exec_v2(self.__autolog_module, "setLoggerDuration", params, is_system=True)

    def clean_autolog(self):
        """
        clean autolog execution, remove persistent option and programmed loggers.
        """
        self._internal_exec_v2(self.__autolog_module, "clean", is_system=True)
        # clean autolog storage
        cmd = "adb shell rm -r %s" % self._autolog_folder
        self._exec(cmd, raise_error=False)

    def _get_autolog_result(self, name):
        """
        Internal method that get the result from autolog functions.

        :type  name: str
        :param name: file which contains autolog uecmd result

        :type  reset_logs: boolean
        :param reset_logs: set to true if you want to reset the logs after
                            getting the value to avoid pulling the same data each time

        :rtype: list of (dict, str)
        :return: return a list containing tuples of (special stamp, log)
                    special stamp contains the following keys ("AUTOLOG_HOURS", "REBOOT", "REBOOT_TIME")
        """
        # Move import here to avoid problem on other uecmd when libxml is missing
        from lxml import etree
        from acs_test_scripts.Utilities.EMUtilities import EMConstant as CST

        # add a retry mechanism
        cmd = "adb shell cat %s/%s" % (self._autolog_folder, name)
        for i in range(4):
            if i == 3:
                tmp_txt = "problem to access file %s/%s, is this file existing? check if something happened before logging start or read permission on the target files" % (self._autolog_folder, name)
                self._logger.error(tmp_txt)
                raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)

            # set a big timeout to avoid timeout if file size is too big
            output = self._exec(cmd, timeout=300)
            if not self.is_shell_output_ok(output):
                self._logger.error("retrying getting autolog result")
            else:
                break

        output = output.split(self.AUTO_LOG_NEW_ENTRY_SEPARATOR)
        result = []
        for element in output:
            special_dict = {}
            if element.find("<GLOBAL_INFO") != -1:
                # get the str with autolog general info
                element = element.strip()
                xml_txt = element[element.find("<GLOBAL_INFO"): element.find("/>") + 2]

                document = etree.fromstring(xml_txt)
                for tag in [CST.AUTOLOG_TIME, CST.REBOOT, CST.REBOOT_TIME]:
                    # check every tag
                    tag_value = document.get(tag)
                    if tag is not None and tag_value is not None:
                        if tag in [CST.AUTOLOG_TIME, CST.REBOOT_TIME]:
                            tag_value = int(tag_value)
                        else:
                            tag_value = str_to_bool(tag_value)
                        special_dict.update({tag: tag_value})

                    # clean text from evaluate tag
                    element = element.replace(xml_txt, "").strip()

                if special_dict != {}:
                    formated_output = element.replace(
                        self.CARRIAGE_RETURN_MARKER, "\n")
                    result.append((special_dict, formated_output.strip()))

        return result
