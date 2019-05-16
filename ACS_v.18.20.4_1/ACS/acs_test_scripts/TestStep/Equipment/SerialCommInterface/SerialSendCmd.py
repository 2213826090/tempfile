"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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

:organization: INTEL OTC Android SSG
:summary: This command is used to send strings to an embedded device attached through a serial interface(COM port/ttyUSB)
:since: 1/7/16
:author: mmaraci
"""

from acs_test_scripts.TestStep.Equipment.SerialCommInterface.SerialBase import SerialBase

class SerialSendCmd(SerialBase):
    """
    This class TestStep is used to send strings sent through a Serial interface with a device(COM port/ttyUSB device)
    """

    def run(self, context):
        SerialBase.run(self, context)

        self._api.send_command(self._pars.command)