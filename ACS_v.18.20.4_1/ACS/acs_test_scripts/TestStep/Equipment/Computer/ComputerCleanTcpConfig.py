"""
@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

@organization: UMG PSI Validation
@summary: This file implements the step to clean TCP stack on a computer
:since: 04/12/2014
:author: jfranchx
"""

from TestStep.Equipment.Computer.ComputerBase import ComputerBase


class ComputerCleanTcpConfig(ComputerBase):
    """
    Clean TCP stack on the computer
    """

    def run(self, context):
        """
        Run the test step
        """
        ComputerBase.run(self, context)

        self._computer.clean_tcp_records_computer()
