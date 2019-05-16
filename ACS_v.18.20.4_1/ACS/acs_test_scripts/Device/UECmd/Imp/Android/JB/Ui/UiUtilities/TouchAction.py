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

from acs_test_scripts.Device.UECmd.Imp.Android.Common.Ui.UiUtilities.TouchAction \
    import TouchAction as CommonTouchAction
from acs_test_scripts.Device.UECmd.Imp.Android.JB.Ui.UiUtilities.InputUtilities \
    import InputUtilities


class TouchAction(CommonTouchAction):

    """
    This class implements the simple I{touch} action.
    """

    def __init__(self, monkey, device, parameters, default_timeout):
        """
        Constructor.
        Expected parameters:
            - C{parameters[0]}: x coordinate
            - C{parameters[1]}: y coordinate
            - C{parameters[2]}: timeout (optional)

        Each coordinate can be expressed as:
            - C{int} : an integer
            - C{str} : a integer value given as str.
            - random range: a str representing a range from wich
                a random value will be computed.
                E.g:
                - [RAND(1,7)]: a random value from 1 to 7
        :type default_timeout: int
        :param default_timeout: the default time out value to use when
            none is specified
        :type parameters: list
        :param parameters: the list of parameters as described above.
        """
        CommonTouchAction.__init__(self, monkey, device, parameters, default_timeout)
        self._input = InputUtilities(device)

    def _do(self):
        """
        Runs this action.
        """
        if not self.parameters_processed():
            self.process_parameters()
        result = self._input.touch(self._x, self._y, self._default_timeout)
        self._set_result(result)
        return result
