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
:summary: This file implements the LG HDK product based on Moorefield platform
:since: 25/09/2014
:author: aderoche
"""

from Device.Model.AndroidDevice.AndroidDeviceBase import AndroidDeviceBase
from Device.Model.AndroidDevice.MoorefieldDevice import MoorefieldDevice
from UtilitiesFWK.Utilities import Global


class LgHdkDevice(MoorefieldDevice):

    """
        Moorefield platform implementation
    """

    # overrides boot modes from parent class
    BOOT_STATES = {"MOS": ["main", "normal"],
                   "ROS": "fota-recovery",
                   "COS": ["charger", "chargerlogo"]}
