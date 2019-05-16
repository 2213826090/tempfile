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
:summary: This script implements the interface of system uecmd.
:since: 08/24/2010
:author: wchen61
"""
from ErrorHandling.DeviceException import DeviceException


class IResidencies:
    """
    SleepMode interface
    """

    def __init__(self, device):
        """
        Constructor
        """
        pass

    def clear(self):
        """
        Clear sleep mode on the device
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def device_has_residencies(self):
        """
        To know if the device has a residency file

        :rtype: bool
        :return: true if the device has a residency file, else false
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_value(self, _name, _mode):
        """
        Get the residencies

        :rtype: list
        :return: a list of dictionary of residency values
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get(self):
        """
        Get the residencies

        :rtype: list
        :return: a list of dictionary of residency values
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def convert_criteria_list(self, target_criteria):
        """
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def parse_socwatch_nc_dstates_info(self, result_file_fullpath, target_block):
        """
        Parse D state info in the SOCWatch csv file

        :rtype: list
        :return: a list of D state residency values
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def parse_socwatch_sstates_info(self, result_file_fullpath):
        """
        Parse S state info in the SOCWatch csv file
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def disable_s3(self):
        """
        Blocks s3 sleep state
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
