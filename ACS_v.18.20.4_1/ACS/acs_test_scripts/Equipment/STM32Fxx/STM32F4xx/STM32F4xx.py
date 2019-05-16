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
:summary: This class will instantiate an STM32F4 device in the flashing port
      This class does not reffer to any sort of USART/UART communication
:since: 3/10/16
:author: mmaraci
"""
import os
import stat
import subprocess
from acs_test_scripts.Equipment.IEquipment import EquipmentBase


class STM32F4xx(EquipmentBase):
    """
    This class will instantiate an STM32F4 board and perform some operations such as:
    - flash a new program on the board
    - set a global environment variable on your Unix host: STLINK_DEVICE. This variable
      tells the st-flash binary the USB bus and address of the device to interact with.
          - ! vital for multiple devices attached
    - reset the board. This 'emulates' the power loss behaviour that is being tested for
      some devices.
    """

    STM_FLASH_PATH = "STM_FLASH_PATH"
    STM_FLASH_BIN = "st-flash"
    STM_DEVICE_ENV = "STLINK_DEVICE"
    STM_USB_ADDRESS = "USBAddress"

    def __init__(self, name, model, eqt_params, bench_params):
        """
        This is the __init__ method of our class, default for all equipments
        """
        if bench_params.has_parameter(STM32F4xx.STM_USB_ADDRESS):
            self.stm_path = bench_params.get_param_value(STM32F4xx.STM_USB_ADDRESS)

        EquipmentBase.__init__(self, name, model, eqt_params)

    def init(self):
        """
        This is a mockup init for the equipment instatiation design pattern that is used
        :return:
        """

    def set_env_device(self):
        """
        This method is responsible for setting the STLINK_DEVICE global variable using the
        address passed in the bench config as USBAddress, in the format <USB_DEVICE>:<USB_ADDRESS>
        """
        if self.stm_path:
            os.environ[STM32F4xx.STM_DEVICE_ENV] = self.stm_path

    def chmod_x(self, file):
        """
        This method simply makes a binary be executable
        :param file: the binary file path for turn executable
        """
        st = os.stat(file)
        os.chmod(file, st.st_mode | stat.S_IEXEC)

    def flash_device(self, binaries_path, program_name):
        """
        This method will flash a given program, using a given binary path
        :param binaries_path: the path to the binary used for flashing
        :param program_name: the name(by path) of the program that will be flashed on the device
        :return: Output of the command that was executed
        """
        self.set_env_device()
        self.chmod_x(binaries_path)
        args = ["{0}".format(binaries_path),
                "write",
                "{0}".format(program_name),
                "0x8000000"]
        popen = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        output, err = popen.communicate()
        return output, err, popen.returncode

    def reset_board(self, reset_bin_path):
        """
        This method will reset an STM board
        :param reset_bin_path:the path to the binary used for flashing
        :return: Output of the command that was executed
        """
        self.set_env_device()
        self.chmod_x(reset_bin_path)
        args = ["{0}".format(reset_bin_path)]
        popen = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        output, err = popen.communicate()
        return output, err, popen.returncode
