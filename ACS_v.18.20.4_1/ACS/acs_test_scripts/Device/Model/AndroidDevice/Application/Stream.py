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
:summary: This script implements the User mode image capture for power measurement
:since: 21/06/2013
:author: jbourgex
"""
import re
from acs_test_scripts.Device.Model.AndroidDevice.Application.IBinary import IBinary
import os
from ErrorHandling.AcsConfigException import AcsConfigException


class Stream(IBinary):
    """
    class for use case stream
    """

    def __init__(self, device):
        """
        Initializes this instance.

        :type device: Device
        :param device: The DUT
        """
        IBinary.__init__(self, device)
        self._path_ref = os.path.join("BENCHMARK", "STREAM")
        self.is_lower_better = False
        self._results = {"Copy": [], "Add": [], "Scale": [], "Triad": []}

    def _fetch_result(self):
        """
        get result of the test
        """
        output = self._fetch_file()

        if output is None:
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED,
                                     "Can't read output results")

        lines = output.splitlines()
        for line in lines:
            match = re.search(r"(?P<type>Add|Scale|Copy|Triad):\s*(?P<value>[\d\.]*).*", line)
            if match:
                self._results[match.group("type")].append(float(match.group("value")))
