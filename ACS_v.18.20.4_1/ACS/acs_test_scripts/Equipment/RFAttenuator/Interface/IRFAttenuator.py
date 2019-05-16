"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related to the source code ("Material") are owned by
Intel Corporation or its suppliers or licensors. Title to the Material remains with Intel Corporation or its suppliers
and licensors. The Material contains trade secrets and proprietary and confidential information of Intel or its
suppliers and licensors.

The Material is protected by worldwide copyright and trade secret laws and treaty provisions. No part of the Material
may be used, copied, reproduced, modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual property right is granted to or conferred
upon you by disclosure or delivery of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express and approved by Intel in writing.

:organization: INTEL MCG
:summary: This file implements the interface for RF Attenuators
:since: 2014-08-12
:author: emarchan

"""
from ErrorHandling.TestEquipmentException import TestEquipmentException


class IRFAttenuator(object):

    def set_attenuation(self, attenuation):
        """
        Sets the attenuation. This method will round the value to the closest step supported by the attenuator.

        :type attenuation: float
        :param attenuation: Attenuation value (between MIN_ATTN_VALUE and MAX_ATTN_VALUE)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_attenuation(self):
        """
        Gets the current attenuation.

        :type attenuation: float
        :param attenuation: Attenuation value (between MIN_ATTN_VALUE and MAX_ATTN_VALUE)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_offset(self, offset):
        """
        Sets the offset to apply to attenuation.
        This method is used to calibrate several attenuators and set the value to have the same "0" basis.

        :type offset: float
        :param offset: Attenuation value
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_max_attn(self):
        """
        Parses the attenuator model number and computes the maximal attenuation from it.
        :rtype: float
        :return: The maximum attenuator for the current attenuator.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)
