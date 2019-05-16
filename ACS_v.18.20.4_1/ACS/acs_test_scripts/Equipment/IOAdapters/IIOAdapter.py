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
:summary: Interface definition for Diolan IO adapter
:since: 07/08/2014
:author: dpierrex
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException

# pylint: disable=W0613


class IIOAdapter(object):
    """
    Class IIOCard: virtual interface for input/output cards
    """

    def init(self):
        """
        Initializes the equipment. The equipment is ready to use
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def release(self):
        """
        Releases all resources allocated to equipment
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def reset(self):
        """
        Reset the IO adapter to default states
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_version(self):
        """
        return the version strings dictionary
        :rtype dictionary
        :return versions dictionary
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_i2cslave_address(self):
        """
        Get address for the I2C slave module
        :rtype String
        :return the I2C slave address in hex format
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_i2cslave_address(self, address):
        """
        Set address for the I2C slave module
        :type address: String
        :param address: address in hex format ( for 0x13 give "13" )
        :rtype boolean
        :return True if success, else False
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_i2cslave_payload(self, payload):
        """
        Set payload for the I2C slave module
        :type payload: String
        :param payload: the payload to put in the slave memory in hex format
        :rtype boolean
        :return True if success, else False
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def disable_i2cslave(self):
        """
        disable i2c slave module
        :rtype boolean
        :return True if success, else False
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def enable_i2cslave(self):
        """
        enable i2c slave module
        :rtype boolean
        :return True if success, else False
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def enable_spislave(self):
        """
        enable spi slave module
        :rtype boolean
        :return True if success, else False
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def disable_spislave(self):
        """
        disable Spi slave module
        :rtype boolean
        :return True if success, else False
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_spislave_payload(self, payload):
        """
        Set payload for the SPI slave module
        :type payload: String
        :param payload: The payload to load in the spi slave in hex format
        :rtype boolean
        :return True if success, else False
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_spislave_wordsize(self, bits_per_word):
        """
        Set payload for the SPI slave module
        :type bits_per_word: Integer
        :param bits_per_word: the number of bits per word
        :rtype boolean
        :return True if success, else False
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_spislave_wordsize(self):
        """
        Get payload for the SPI slave module
        :rtype tuple
        :return True if success, else False
        :return the actual bits per word
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_spislave_mode(self, mode):
        """
        Set mode for the SPI slave module
        :type mode: Integer
        :param mode: the SPI mode to use
        :rtype Boolean
        :return True if success, else False
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_spislave_mode(self):
        """
        Get mode for the SPI slave module
        :rtype tuple
        :return True if success, else False
        :return the actual mode
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def load_specific_dut_config(self, dut_name):
        """
        Configure different setting on your io card related to dut name,
        This is useful in multi device campaign.
        The setting will depending of your current dut name and what you declared on
        benchconfig.

        :type dut_name: str
        :param dut_name: phone name
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)
