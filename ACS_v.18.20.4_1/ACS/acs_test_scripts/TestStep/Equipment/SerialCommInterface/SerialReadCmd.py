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
:summary: This command is used to read strings to an embedded device attached
through a serial interface(COM port/ttyUSB)
:since: 1/7/16
:author: mmaraci
:modified: 3/30/16 mvminci
"""

from Core.TestStep.EquipmentTestStepBase import EquipmentTestStepBase
from acs_test_scripts.TestStep.Equipment.SerialCommInterface.SerialBase import SerialBase


class SerialReadCmd(SerialBase):
    """
    This class TestStep is used to read strings sent through a Serial interface with a device(COM port/ttyUSB device)
    """
    DEFAULT_READ_TIMEOUT = 10

    def run(self, context):
        EquipmentTestStepBase.run(self, context)
        self._api = self._equipment_manager.get_serial_com_port(self._pars.eqt)
        read_timeout = self._pars.readline_timeout
        if read_timeout or read_timeout == 0:
            self._api.set_timeout(read_timeout)
        else:
            # if no timeout specified set it to default value
            self._api.set_timeout(SerialReadCmd.DEFAULT_READ_TIMEOUT)
        timeout_lines_stop = -1
        parsed_timeout_lines = self._pars.timeout_lines_stop
        if parsed_timeout_lines:
            if parsed_timeout_lines <= 0:
                self._raise_config_exception("TIMEOUT_LINES_STOP must be > 0, but got:{0}".format(parsed_timeout_lines))
            else:
                timeout_lines_stop = parsed_timeout_lines

        results = self._api.read_output(int(self._pars.max_lines), timeout_lines_stop)
        self._logger.debug("SERIAL READ INPUT: %s", results)
        if self._pars.save_as:
            context.set_info(self._pars.save_as, results)
