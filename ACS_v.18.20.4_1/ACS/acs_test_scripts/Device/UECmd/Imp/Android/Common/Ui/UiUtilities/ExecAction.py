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

from UiAction import UiAction


class ExecAction(UiAction):

    """
    This class implements operations required in order to
    perform a command execution.
    """

    def __init__(self, monkey, parameters, default_timeout):
        """
        Constructor.
        Expected parameters:
            - C{parameters[0]} : object : device instance
            - C{parameters[1]} : str : command
            - C{parameters[2]} : int : timeout (optional)

        :type default_timeout: int
        :param default_timeout: the default time out value to use when
            none is specified
        :type parameters: list
        :param parameters: the list of parameters as described above.
        """
        UiAction.__init__(self, monkey, default_timeout)
        self.set_parameters(parameters["parameters"])
        self._parameters_processed = False
        self.__cmd_start = "adb shell am "
        self.__cmd = None
        self.__device_instance = parameters["device"]

    def _do_process_parameters(self):
        """
        Update this action attributes from the parameters
        given at instantiation time.
        """
        # Update the command
        self.__cmd = self._parameters[0]
        # Check if we need to replace by the sim device number
        if "[PHONE_NUMBER]" in self.__cmd:
            self.__cmd = \
                self.__cmd.replace(
                    "[PHONE_NUMBER]",
                    self.__device_instance.get_phone_number())
        # Update the timeout
        if len(self._parameters) > 1:
            (timeout, fail_on_timeout) = self.split_timeout(self._parameters[1])
            self.set_timeout(timeout)
            self.set_fail_on_timeout(fail_on_timeout)
        else:
            self.set_timeout(self._default_timeout)
        self._parameters_processed = True

    def _do(self):
        """
        Runs this action.
        """
        if not self.parameters_processed():
            self.process_parameters()

        if self.__cmd.startswith("start"):
            cmd = "%s %s" % (self.__cmd_start, self.__cmd)
        else:
            cmd = self.__cmd
        result, _output = self.__device_instance.run_cmd(cmd, self.get_timeout())
        self._set_result(result)
        return result

    def _redo(self):
        """
        Re-runs this action.
        """
        if not self.parameters_processed():
            self.process_parameters()
        return self._do()

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
        This feature may be implemented later.
        :return: C{False}
        """
        return False
