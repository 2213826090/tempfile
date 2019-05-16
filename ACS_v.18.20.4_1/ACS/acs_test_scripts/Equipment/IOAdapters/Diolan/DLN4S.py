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

:organization: INTEL NDG SW
:summary: io adapter implementation for DLN-4S
:since: 07/08/2014
:author: dpierrex
"""

from acs_test_scripts.Equipment.IEquipment import DllLoader
from acs_test_scripts.Equipment.IOAdapters.Diolan.Wrapper.DiolanWrapper import DiolanWrapper
from acs_test_scripts.Equipment.IOAdapters.IIOAdapter import IIOAdapter


class DLN4S(DllLoader, IIOAdapter):
    """
    Class that implements Diolan DLN-4S IO Adapter equipment
    """

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        :type name: str
        :param name: the bench configuration name of the equipment
        :type model: str
        :param model: the model of the equipment
        :type eqt_params: dict
        :param eqt_params: the dictionary containing equipment parameters
        :type bench_params: dict
        :param bench_params: the dictionary containing equipment bench parameters
        """
        DllLoader.__init__(self, name, model, eqt_params)
        IIOAdapter.__init__(self)
        self.__bench_params = bench_params
        self.__diolan_wrapper = None
        self.__diolan_device_id = None
        self.__handle = None

    def init(self):
        """
        Initializes the equipment. The equipment is ready to use
        """
        self.get_logger().info("Initialization")
        self.load_driver()
        self.__diolan_wrapper = DiolanWrapper(self._logger, self.get_dll())

        # Get the ID of the adapter
        if ((self.get_bench_params().has_parameter("deviceId")) and
                (self.__bench_params.get_param_value("deviceId") != "")):
            self.__diolan_device_id = \
                int(self.__bench_params.get_param_value("deviceId"))

        # disconnect all boards and get a fresh connection
        self.reset()
        self.__diolan_wrapper.enable_notification_callback()

    def release(self):
        self.__diolan_wrapper.close_device()

    def get_bench_params(self):
        """
        Returns the bench parameters of the equipment
        """
        return self.__bench_params

    def reset(self):
        """
        Reset the IO Adapter, reopen device
        """
        self._logger.info("Reset Diolan board")
        self.__diolan_wrapper.reset_board()
        self.__handle = self.__diolan_wrapper.open_device(self.__diolan_device_id)
        self._logger.info("handle = {0}".format(self.__handle))
        return True if self.__handle is not None else False

    def get_version(self):
        self.__diolan_wrapper.get_version()

    def get_i2cslave_address(self):
        return self.__diolan_wrapper.get_i2cslave_address()

    def set_i2cslave_address(self, address):
        return self.__diolan_wrapper.set_i2cslave_address(address)

    def set_i2cslave_payload(self, payload):
        """
        Set payload for the I2C slave module
        """
        return self.__diolan_wrapper.set_i2cslave_payload(payload)

    def disable_i2cslave(self):
        """
        disable i2c slave module
        """
        return self.__diolan_wrapper.disable_i2cslave()

    def enable_i2cslave(self):
        """
        enable i2c slave module
        """
        return self.__diolan_wrapper.enable_i2cslave()

    def enable_spislave(self):
        """
        enable spi slave module
        """
        return self.__diolan_wrapper.enable_spislave()

    def disable_spislave(self):
        """
        disable Spi slave module
        """
        return self.__diolan_wrapper.disable_spislave()

    def set_spislave_payload(self, payload):
        """
        Set payload for the SPI slave module
        """
        return self.__diolan_wrapper.set_spislave_payload(payload)

    def set_spislave_mode(self, mode):
        """
        Set mode for the SPI slave module
        """
        return self.__diolan_wrapper.dln_spislave_set_mode(mode)
        # Issue with the Supported Modes
        res, listSupported = self.__diolan_wrapper.dln_spislave_get_supported_modes()
        if res:
            if mode in listSupported:
                return self.__diolan_wrapper.DlnSpiSlaveSetMode(mode)
            else:
                self._logger.error("Diolan only support the following modes : {0}".format(listSupported))
        return False

    def get_spislave_mode(self):
        """
        Get mode for the SPI slave module
        """
        return self.__diolan_wrapper.dln_spislave_get_mode()

    def set_spislave_wordsize(self, bits_per_word):
        """
        Set payload for the SPI slave module
        """
        res, listSupported = self.__diolan_wrapper.dln_spislave_get_supported_framesizes()
        if res:
            if bits_per_word in listSupported:
                return self.__diolan_wrapper.dln_spislave_set_framesize(bits_per_word)
            else:
                self._logger.error("Diolan only support the following frame sizes : {0}".format(listSupported))
        return False

    def get_spislave_wordsize(self):
        """
        Get payload for the SPI slave module
        """
        return self.__diolan_wrapper.dln_spislave_get_framesize()

    def load_specific_dut_config(self, dut_name):
        """
        .. warning:: not functional for this io adapter, defined only for compatibility reason
        """
        self._logger.debug("load_specific_dut_config is not supported by this relay card")
        pass
