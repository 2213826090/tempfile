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
from UtilitiesFWK.Utilities import Global


class TypeAction(UiAction):

    """
    This class implements the action of typing some text
    in a text field.
    """

    def __init__(self, monkey, device, parameters, default_timeout, bench_config, device_config):
        """
        Constructor.
        Expected parameters:
            - C{parameters[0]}: text: the text to type

        :type default_timeout: int
        :param default_timeout: the default time out value to use when
            none is specified
        :type parameters: list
        :param parameters: the list of parameters as described above.
        """
        UiAction.__init__(self, monkey, default_timeout)
        self.set_parameters(parameters["parameters"])
        self._parameters_processed = False
        self._device = device
        self._text = None
        self.__bench_config = bench_config
        self.__device_config = device_config

    def _do_process_parameters(self):
        """
        Update this action attributes from the parameters
        given at instantiation time.
        """

        # Update the text
        text = self._parameters[0]

        if text.upper() == "[RANDOM_NAME]":
            import acs_test_scripts.Utilities.NameGenerator as gen
            text = gen.generate(1, 10)
        elif text.upper() == "[PHONE_NUMBER]":
            text = self._device.get_phone_number()
        elif text.strip().startswith("[BC:") or text.strip().startswith("[bc:"):
            parameters = text.strip().replace("[bc:", "")
            parameters = parameters.replace("[BC:", "")
            parameters = parameters.replace("]", "")
            parameters = parameters.split(":")
            if len(parameters) == 2 and \
               self.__bench_config.has_parameter(parameters[0]):
                bc_param = self.__bench_config.get_parameters(parameters[0])
                text = bc_param.get_param_value(parameters[1])
            else:
                text = "Wrong parameter: %s" % text
        elif text.strip().startswith("[DC:"):
            parameters = text.strip().replace("[dc:", "")
            parameters = parameters.replace("[DC:", "")
            parameters = parameters.replace("]", "")
            parameters = parameters.split(":")
            if len(parameters) == 2 and \
               parameters[0] in self.__device_config:
                dc_param = self.__device_config[parameters[0]]
                text = dc_param.get(parameters[1])
            else:
                text = "Wrong parameter: %s" % text

        self._text = text
        # Update the timeout
        self.set_timeout(self._default_timeout)
        self._parameters_processed = True

    def _do(self):
        """
        Runs this action.
        """
        if not self.parameters_processed():
            self.process_parameters()
        self.get_monkey().type(self._text, self._default_timeout)
        self._can_undo = True
        self._set_result(Global.SUCCESS)
        return Global.SUCCESS

    def _redo(self):
        """
        Re-runs this action.
        """
        if not self.parameters_processed():
            self.process_parameters()
        return self._do()

    def _undo(self):
        """
        Undoes this action.
        """
        text_length = len(self._text)
        for _i in range(text_length):
            self.get_monkey().press("KEYCODE_FORWARD_DEL")
        self._can_undo = False

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
        :return:
            - C{True} if this action can be undone
            - C{False} otherwise.
        """
        return self._can_undo
