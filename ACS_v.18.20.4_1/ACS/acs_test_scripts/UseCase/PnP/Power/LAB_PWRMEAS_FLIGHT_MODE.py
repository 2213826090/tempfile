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
:summary: Use Case base for Idle Power measurement CDK in flight mode
:since: 13/08/2010
:author: ssa
"""
from LAB_PWRMEAS_BASE import LabPwrMeasBase


class LabPwrMeasFlightMode(LabPwrMeasBase):

    """
    Class Lab Idle Power Measurement CDK in Flight Mode.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call power measurement base Init function
        LabPwrMeasBase.__init__(self, tc_name, global_config)

        # Read sleep mode of the board and brightness for s0 mode
        self._sleep_mode = self._tc_parameters.get_param_value("MODE")
        flight_mode = self._tc_parameters.get_param_value("FLIGHT_MODE", True, "str_to_bool")
        if not flight_mode:
            self._technology_power.append("cellular")
