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

:organization: INTEL TMT TTD AN
:summary: This file implements the Sandyshores platform DeviceChecks module
:since: 2/11/2016
:author: ahkhowaj
"""

from acs_test_scripts.Device.Module.Common.DeviceChecksModule.IDeviceChecksModule import IDeviceChecksModule
from ErrorHandling.DeviceException import DeviceException
from Device.Module.DeviceModuleBase import DeviceModuleBase


class SandyshoresDeviceChecksModule(IDeviceChecksModule, DeviceModuleBase):

    def __init__(self):
        super(SandyshoresDeviceChecksModule, self).__init__()

    def init(self):
        """
        Initialize nw module

        :rtype: UtilitiesFWK.Utilities.Global
        :return: Init status
        """
        verdict = Global.SUCCESS
        self._networking_properties = self.configuration
        return verdict

    def get_emmc_dma_mode(self):

        """
        Reads eMMC controller BAR at offset 0x10 from pci config space, adds offset 28h to get Host Control 1 Register (HC1). Bits [4:3] of HC1 indicate DMA mode.
        :rtype : int
        :return : DMA mode for eMMC controller
        """
        cmd = "adb shell peeknpoke b r {0} {1} {2} 10".format(self.configuration["bus"], self.configuration["dev"], self.configuration["func"])
        status, output = self.device.run_cmd(cmd, 2)
        temp_string = output.split(" ")
        if temp_string[0] == "PCI":
            temp_string = temp_string[(len(temp_string) - 1)].split("\r")[0]
            #clearing bits[7:1] fixed to 000 0000b then adding offset 28h to read Host Control 1 Register
            bar = hex(int("%x" %(int(temp_string, 0) & 0xffffff01)) + 0x28).split('x')[1]
            cmd = "adb shell peeknpoke r {0} 32".format(bar)
            status, output = self.device.run_cmd(cmd, 2)
            temp_string = output.split(" ")
            if temp_string[0] != "Failed":
                temp_string = temp_string[(len(temp_string) - 1)].split("\r")[0]
                #bits[4:3] specify DMA Mode
                dma_mode = (int(temp_string, 0) & 0x18) >> 3
                return dma_mode
            else:
                err_msg = cmd + " failed to access MMIO register"
                raise DeviceException(DeviceException.OPERATION_FAILED, err_msg)
        else:
            err_msg = cmd + " failed to access pci config space for eMMC controller"
            raise DeviceException(DeviceException.OPERATION_FAILED, err_msg)

    def verify_emmc_dma_mode(self):

        """
        Gets DMA mode for eMMC controller, should be 3 for 64-bit Address ADMA2 mode.
        :rtype : boolean
        :return : true if DMA mode is 64-bit Address ADMA2, false otherwise
        """
        dma_modes = {0: "SDMA", 1: "RSVD", 2: "32-bit Address ADMA2", 3: "64-bit Address ADMA2"}
        dma_mode = self.get_emmc_dma_mode()
        self.logger.debug("verify_dma_mod: emmc dma mode is {0}".format(dma_modes[dma_mode]))

        if dma_mode == 3:
            return True
        return False
