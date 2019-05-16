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
:summary: This class implements an instantiation for reading serial output and for sending commands
:since: 07/01/16
:author: mmaraci
:modified: 3/30/16 mvminci
"""

import serial
from acs_test_scripts.Equipment.IEquipment import EquipmentBase


class SerialCom(serial.Serial, EquipmentBase):
    COM_PORT = "comPort"
    BAUD_RATE = "baudRate"
    DEFAULT_COM_PORT = "/dev/ttyUSB0"

    def __init__(self, name, model, eqt_params, bench_params):

        if bench_params.has_parameter(SerialCom.BAUD_RATE):
            self.baudRate = bench_params.get_param_value(SerialCom.BAUD_RATE)
        else:
            self.baudRate = 9600
        if bench_params.has_parameter(SerialCom.COM_PORT):
            self.port1 = bench_params.get_param_value(SerialCom.COM_PORT)
        else:
            self.port1 = SerialCom.DEFAULT_COM_PORT

        EquipmentBase.__init__(self, name, model, eqt_params)
        self.line = ""

    def init(self):
        super(SerialCom, self).__init__(self.port1, self.baudRate)

    def send_command(self, command):
        self.get_logger().info("SERIAL_COM send command: %s", command)
        super(SerialCom, self).write("\r\n")
        super(SerialCom, self).write(str(command))
        super(SerialCom, self).write("\r\n")

    def set_timeout(self, timeout):
        super(SerialCom, self).setTimeout(timeout)

    def read_output(self, maxlines, timeout_lines_stop):
        """
        Reads the specified maximum number of lines from serial equipment

        :param maxlines: Maximum amount of lines to be read from serial output
        :param timeout_lines_stop: the number of lines that give timeout after
        which to stop the read process
        """
        c = 0
        timeout_lines_count = 0
        ret_lines = []
        var = True
        while var:
            raw_output_line = super(SerialCom, self).readline()
            output_line = raw_output_line.strip('\n').strip('\r')
            if output_line and output_line.strip():
                ret_lines.append(output_line)
            else:
                if not raw_output_line:
                    timeout_lines_count += 1
                    # read timeout occurred, handle stop read here also
                    if timeout_lines_stop > 0:
                        if timeout_lines_count > timeout_lines_stop:
                            var = False
                    else:
                        var = False
            c += 1
            if c >= maxlines:
                var = False
        self.flushInput()
        self.flushOutput()

        return ret_lines
