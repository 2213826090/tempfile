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


class DragAction(UiAction):

    """
    This class implements the I{drag} action.
    """

    def __init__(self, monkey, parameters, default_timeout):
        """
        Constructor.
        Expected parameters:
            - C{parameters[0]}: int : the start point x coordinate
            - C{parameters[1]}: int : the start point y coordinate
            - C{parameters[2]}: int : the end point x coordinate
            - C{parameters[3]}: int : the end point x coordinate
            - C{parameters[4]}: int : timeout (optional)

        :type default_timeout: int
        :param default_timeout: the default time out value to use when
            none is specified
        :type parameters: list
        :param parameters: the list of parameters as described above.
        """
        UiAction.__init__(self, monkey, default_timeout)
        self.set_parameters(parameters["parameters"])
        self._parameters_processed = False
        self._start_x = None
        self._start_y = None
        self._end_x = None
        self._end_y = None

    def _do_process_parameters(self):
        """
        Update this action attributes from the parameters
        given at instantiation time.
        """
        # Update the start x coordinate
        self._start_x = int(str(self._parameters[0]))
        # Update the start y coordinate
        self._start_y = int(str(self._parameters[1]))
        # Update the end x coordinate
        self._end_x = int(str(self._parameters[2]))
        # Update the end y coordinate
        self._end_y = int(str(self._parameters[3]))
        # Update the timeout
        if len(self._parameters) > 4:
            # Check whether there is a timeout parameter
            (timeout, fail_on_timeout) = self.split_timeout(self._parameters[4])
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
        self.get_monkey().drag(
            self._start_x,
            self._start_y,
            self._end_x,
            self._end_y,
            self._default_timeout)
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
        This action cannot be undone.
        """
        self.get_monkey().drag(
            self._end_x,
            self._end_y,
            self._start_x,
            self._start_y,
            self._default_timeout)
        self._can_undo = False

    def can_redo(self):
        """ %
                                           target_script_path
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
