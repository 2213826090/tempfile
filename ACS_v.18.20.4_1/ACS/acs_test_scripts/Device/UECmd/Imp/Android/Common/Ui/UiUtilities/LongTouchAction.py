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
from UiUtilities import RandomRange


class LongTouchAction(UiAction):

    """
    This class implements the I{long touch} action.
    """

    def __init__(self, monkey, parameters, default_timeout):
        """
        Constructor.
        Expected parameters:
            - C{parameters[0]}: x coordinate
            - C{parameters[1]}: y coordinate
            - C{parameters[2]}: timeout (optional)

        Each coordinate can be expressed as:
            - C{int} : an integer
            - C{str} : a integer value given as str.
        :type default_timeout: int
        :param default_timeout: the default time out value to use when
            none is specified
        :type parameters: list
        :param parameters: the list of parameters as described above.
        """
        UiAction.__init__(self, monkey, default_timeout)
        self.set_parameters(parameters["parameters"])
        self._parameters_processed = False
        self.__x = None
        self.__y = None

    def _do_process_parameters(self):
        """
        Update this action attributes from the parameters
        given at instantiation time.
        """
        # Update the x coordinate
        x_coordinate = self._parameters[0]
        if type(x_coordinate == unicode):
            x_coordinate = str(x_coordinate)
        if(isinstance(x_coordinate, str) and x_coordinate.startswith("[RAND(")):
            self.__x = RandomRange.get_value_from_range(x_coordinate)
        else:
            self.__x = int(x_coordinate)
        # Update the y coordinate
        y_coordinate = self._parameters[1]
        if type(y_coordinate == unicode):
            y_coordinate = str(y_coordinate)
        if(isinstance(y_coordinate, str) and y_coordinate.startswith("[RAND(")):
            self.__y = RandomRange.get_value_from_range(y_coordinate)
        else:
            self.__y = int(y_coordinate)
        # Update the timeout
        if len(self._parameters) > 2:
            # Check whether there is a timeout parameter
            (timeout, fail_on_timeout) = self.split_timeout(self._parameters[2])
            self.set_timeout(timeout)
            self.set_fail_on_timeout(fail_on_timeout)
        else:
            # If not, use default value
            self.set_timeout(self._default_timeout)
        self._parameters_processed = True

    def _do(self):
        """
        Runs this action.
        """
        if not self.parameters_processed():
            self.process_parameters()
        result = self.get_monkey().long_touch(self.__x, self.__y, self._default_timeout)
        self._set_result(result)
        return result

    def _redo(self):
        """
        Re-runs this action.
        """
        if not self.parameters_processed():
            self.process_parameters()
        return self._do()

    def _undo(self):
        """
        This action cannot be undone.
        """
        pass

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
