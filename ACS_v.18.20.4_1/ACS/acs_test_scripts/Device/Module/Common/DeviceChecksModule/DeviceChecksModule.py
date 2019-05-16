"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

@organization: INTEL SVE DSV
@summary: Device Checks module for Register accessing
@since: 12/14/2014
@author: srdubbak
"""
from ErrorHandling.DeviceException import DeviceException


class DeviceChecksModule(IDeviceChecksModule, DeviceModuleBase):

    def __init__(self):
        super(DeviceChecksModule, self).__init__()
        self._system_info = AttributeDict()

    def read_mmio_reg_32(self, reg_address=None):

        """
        Reads the value in the register address specified.
        :rtype : string
        :return : value in the address
        """
        peeknpoke_path = "/system/bin/peeknpoke"

        cmd = "adb shell {0} r {1} 32".format(peeknpoke_path, reg_address)
        status, output = self.run_cmd(cmd, 2)
        temp_string = output.split(" ")
        if temp_string[0] != "Failed":
            temp_string = temp_string[(len(temp_string) - 1)].split("\r")[0]
            reg_val = int(temp_string, 0)
            return reg_val
        else:
            err_msg = cmd + " failed to access MMIO register"
            raise DeviceException(DeviceException.OPERATION_FAILED, err_msg)