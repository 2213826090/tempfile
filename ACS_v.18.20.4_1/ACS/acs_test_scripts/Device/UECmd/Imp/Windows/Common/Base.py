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
:summary: UECmd based file for all Windows Application Framework Base
:since: 01/03/2014
:author: vgombert, dgonza4x
"""
from ErrorHandling.AcsBaseException import AcsBaseException


class Base(object):
    """
    Class that handle all generic operations
    """

    __MODULES_AND_CLASSES_NAME = {
        'Networking': ("Intel.Acs.TestFmk.Networking", "Intel.Acs.TestFmk.Networking.NetworkingActivity"),
        'Connectivity': ("Intel.Acs.TestFmk.MBConnectivity", "Intel.Acs.TestFmk.MBConnectivity.MBActivity"),
        'WifiConnectivity': ("Intel.Acs.TestFmk.WifiConnectivity", "Intel.Acs.TestFmk.WifiConnectivity.WifiActivity"),
        'EnergyManagement': ("Intel.Acs.TestFmk.EnergyManagement", "Intel.Acs.TestFmk.EnergyManagement.EMActivity"),
        'Display': ("Intel.Acs.TestFmk.DeviceSystem", "Intel.Acs.TestFmk.DeviceSystem.DisplayActivity"),
        'DeviceSystem': ("Intel.Acs.TestFmk.DeviceSystem", "Intel.Acs.TestFmk.DeviceSystem.DeviceSystemActivity"),
        'BluetoothConnectivity': (
            "Intel.Acs.TestFmk.BluetoothConnectivity", "Intel.Acs.TestFmk.BluetoothConnectivity.BTActivity"),
        'FTP': ("Intel.Acs.TestFmk.Networking", "Intel.Acs.TestFmk.Networking.FtpActivity"),
        'UIAction': ("Intel.Acs.TestFmk.UIControl", "Intel.Acs.TestFmk.UIControl.UIAction"),
        'UIMusic': ("Intel.Acs.TestFmk.UIControl", "Intel.Acs.TestFmk.UIControl.UIMusic"),
        'UICamera': ("Intel.Acs.TestFmk.UIControl", "Intel.Acs.TestFmk.UIControl.UICamera"),
        'UIBT': ("Intel.Acs.TestFmk.UIControl", "Intel.Acs.TestFmk.UIControl.UIBT"),
        'UIInformation': ("Intel.Acs.TestFmk.UIControl", "Intel.Acs.TestFmk.UIControl.UIInformation"),
        'UIActionCenter': ("Intel.Acs.TestFmk.UIControl", "Intel.Acs.TestFmk.UIControl.UIActionCenter"),
        'UIInternetExplorer': ("Intel.Acs.TestFmk.UIControl", "Intel.Acs.TestFmk.UIControl.UIInternetExplorer"),
        'UIMicrosoftEdge': ("Intel.Acs.TestFmk.UIControl", "Intel.Acs.TestFmk.UIControl.UIMicrosoftEdge"),
        'UIStore': ("Intel.Acs.TestFmk.UIControl", "Intel.Acs.TestFmk.UIControl.UIStore"),
        'UIEmail': ("Intel.Acs.TestFmk.UIControl", "Intel.Acs.TestFmk.UIControl.UIEmail"),
        'UIAlarm': ("Intel.Acs.TestFmk.UIControl", "Intel.Acs.TestFmk.UIControl.UIAlarm"),
        'UICalendar': ("Intel.Acs.TestFmk.UIControl", "Intel.Acs.TestFmk.UIControl.UICalendar")
        }

    UECMD_TIMEOUT_RESULT_MSG = "Did not get ue command result from the board in time."

    # TODO: declare all the constant that are used in uecmd reply here
    # TODO: create an object to represent uecmd reply instead of dictionary

    def __init__(self, device):
        """
        Initializes this instance.
        """
        self._device = device
        self._uecmd_default_timeout = int(self._device.get_uecmd_timeout())
        self._logger = self._device.get_logger()
        self._module_name = "Intel.Acs.TestFmk.MBConnectivity"
        self._class_name = "Intel.Acs.TestFmk.MBConnectivity.MBActivity"

    def _get_module_and_class_names(self, name=None):
        """
        Returns the names of the Agent module and class
        that handles method call for the given domain C{name}.

        :type name: str
        :param name: the domain name. Possible values:
            - Networking
            - Connectivity
            - WifiConnectivity
            - ...

        :rtype: tuple
        :return: the module name and the class name as tuple.

        :raise AcsBaseException: If the parameter name is not a valid key of
        the __MODULES_AND_CLASSES_NAME dictionary
        """
        if name in self.__MODULES_AND_CLASSES_NAME:
            # if the "name" parameter is part of the __MODULES_AND_CLASSES_NAME
            # dictionary keys.
            return self.__MODULES_AND_CLASSES_NAME[name]
        elif name is None:
            # if no parameter was passed to the method.
            return self._module_name, self._class_name
        else:
            # if the passed "name" parameter is not a key of the
            # __MODULES_AND_CLASSES_NAME dictionary.
            raise AcsBaseException(AcsBaseException.INVALID_PARAMETER,
                                   "The \"name\" parameter must be a \
                                   valid key of the \
                                   \"__MODULES_AND_CLASSES_NAME\" \
                                   dictionary")

    def _internal_exec(self, cmd, timeout=None):
        """
        Internal method that execute shell command on device

        :type  cmd: str
        :param cmd: cmd to be executed

        :type  timeout: int
        :param timeout: timeout of the executed command

        :rtype: str
        :return: the output str.
        """
        # Set a default timeout if none was provided
        if timeout is None:
            timeout = self._uecmd_default_timeout

        results = self._device.run_cmd(cmd, timeout, False, False)
        return results

    def _internal_uecmd_exec(self, module_name, class_name, method_name, args="", timeout=None):
        """
        Internal method that execute UE command on device

        :type  module_name: str
        :param module_name: name of the module of the UE command to execute

        :type  class_name: str
        :param class_name: name of the class where the UE command to execute is defined

        :type  cmd: str
        :param cmd: command name to be executed

        :type  args: str
        :param args: command parameters and values, set like "param1=val1 param2=val2 ..."

        :type  timeout: int
        :param timeout: timeout of the executed command

        :rtype: str
        :return: the output str.
        """
        # Set a default timeout if none was provided
        if timeout is None:
            timeout = self._uecmd_default_timeout

        results = self._device.run_uecmd(module_name, class_name, method_name, args, timeout)

        return results

    def finalize(self):
        pass
         # self._logger.warning("finalize not yet implemented on windows")

    def get_async_result(self, task_id):
        """
        Internal method that get ouptut of an async scheduled cmd.
        it is not designed to poll the response, if the task is not finish , the call
        will be blocking.

        :type  task_id: str
        :param task_id: task id to be read

        :rtype: dict
        :return: scheduled command output
        """
        result = self._device.get_async_result(task_id)
        return result

    def schedule_uecmd(self, module_name, class_name, method_name, args, start_delay, repeat_delay="",
                       total_duration=""):
        """
        start an asynchronous uecmd and return a task id to follow the result

        :type  module_name: str
        :param module_name: uecmd module

        :type  class_name: str
        :param class_name: class of uecmd

        :type  method_name: str
        :param method_name: uecmd function

        :type  args: str
        :param args: args to pass to the uecmd

        :type  start_delay: int
        :param start_delay: time in second before starting the uecmd

        :type  repeat_delay: int
        :param repeat_delay: [optional] time in second before repeating the uecmd. If used, total_duration must be set too.

        :type  total_duration: int
        :param total_duration: [optional] the maximum duration time in second allowed to repeat the uecmd. If used, repeat_delay must be set too.

        :rtype: str
        :return: scheduled task id
        """
        return self._device.run_async_uecmd(
            module_name, class_name, method_name, args, start_delay, repeat_delay, total_duration)

    def clean_daemon_files(self):
        """
        Method that delete all daemon files
        """
        self._device.clean_async_uecmds()
