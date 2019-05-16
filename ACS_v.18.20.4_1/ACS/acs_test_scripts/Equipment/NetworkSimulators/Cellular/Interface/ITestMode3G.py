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
:summary: virtual interface of 3G test mode functionalities for cellular network
simulators
:since: 06/04/2011
:author: ymorel
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException

# pylint: disable=W0613


class ITestMode3G(object):

    """
    ITestMode3G class: virtual interface of 3G test mode functionalities for cellular
    network simulators.
    """

    def set_ber_block_number(self, block_number):
        """
        Sets bit error rate block number for BER measurement
        :type block_number: integer
        :param block_number: the number of blocks to set: an integer from
            1 to 50000
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_ber(self):
        """
        Gets bit error rate (BER)
        :rtype: double
        :return: the measured BER
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def configure_rssi_measurement(self):
        """
        Configures received signal strength indicator (RSSI) measurement
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_rssi(self):
        """
        Gets received signal strength indicator (RSSI)
        :rtype: double
        :return: the measured RSSI
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def check_rssi_connectivity(self, target, margin):
        """
        Checks DUT to network simulator connectivity through RSSI measurement.
        The RSSI value retrieved is compared to target +/- margin.
        :type target: integer
        :param target: value of the RSSI expected (in dBm).
        :type margin: integer
        :param margin: acceptable margin.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_avg_max_pow_limits(self, peak_limit, rms_limit, peak_tolerance, rms_tolerance):
        """
        Sets maximum power limits for the average measurements.
        :type peak_limit: float
        :param peak_limit: the peak limit to set
        :type rms_limit: float
        :param rms_limit: the RMS limit to set
        :type peak_tolerance: tuple
        :param peak_tolerance:
            - float: peak lower tolerance
            - float: peak upper tolerance
        :type rms_tolerance: tuple
        :param rms_tolerance:
            - float: RMS lower tolerance
            - float: RMS upper tolerance
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_avg_min_pow_limits(self, peak_limit, rms_limit):
        """
        Sets minimum power limits for the average measurements.
        :type peak_limit: float
        :param peak_limit: the peak limlit to set
        :type rms_limit: float
        :param rms_limit: the RMS limit to set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_modulation_limits(self, peak_evm, rms_evm, offset, frequency_error, dom_error):
        """
        Set EVM (Error Vector Modulation) limits and frequency error limit.
        :type peak_evm: float
        :param peak_evm:
        :type rms_evm: float
        :param rms_evm:
        :type offset: float
        :param offset:
        :type frequency_error: float
        :param frequency_error:
        :type dom_error: float
        :param dom_error:
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_aclr_avg_limits(self, aclr_5_rel, aclr_10_rel, aclr_abs, obw_lim):
        """
        Set ACLR and OBW (Occupied bandwidth) limits.
        :type aclr_5_rel: float
        :param aclr_5_rel: relative limit for ACLR +5MHz and ACLR -5 MHz
        :type aclr_10_rel: float
        :param aclr_10_rel: relative limit for ACLR +10MHz and ACLR -10 MHz
        :type aclr_abs: float
        :param aclr_abs: absolute limit for ACLR
        :type obw_lim: float
        :param obw_lim: occupied bandwidth limit
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_sem_avg_limits(self, limits):
        """
        Sets SEM (Spectrum Emission Mask) limits.
        :type limits: list of doubles
        :param limits: list of 6 values defining 8 sections:
            - 1st & 2nd values: define section 12, 5-8, 5MHz and its symmetric
            - 2nd & 3rd values: define section 8,  5-7, 5MHz and its symmetric
            - 3rd & 4th values: define section 7,  5-3, 5MHz and its symmetric
            - 5th & 6th values: define section 3,  5-2, 5MHz and its symmetric
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)
