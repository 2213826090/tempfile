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
:since: 12 Sep 2012
:author: sfusilie
"""

from acs_test_scripts.Device.UECmd.Imp.Android.Common.Ui.UiUtilities.DragAction \
    import DragAction as CommonDragAction
from acs_test_scripts.Device.UECmd.Imp.Android.JB.Ui.UiUtilities.InputUtilities \
    import InputUtilities


class DragAction(CommonDragAction):

    """
    This class implements the I{drag} action.
    """

    def __init__(self, monkey, device, parameters, default_timeout):
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
        CommonDragAction.__init__(self, monkey, parameters, default_timeout)
        self._input = InputUtilities(device)

    def _do(self):
        """
        Runs this action.
        """
        if not self.parameters_processed():
            self.process_parameters()
        result = self._input.drag(self._start_x, self._start_y, self._end_x, self._end_y, self._default_timeout)
        self._set_result(result)
        return result
