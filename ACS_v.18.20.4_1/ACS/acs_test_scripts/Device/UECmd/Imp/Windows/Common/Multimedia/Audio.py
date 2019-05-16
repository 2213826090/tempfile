"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

:organization: SII on behalf of INTEL MCG PSI
:summary: This file implements audio media UECmds
:since: 11 Feb 2014
:author: vgomberx
"""
from acs_test_scripts.Device.UECmd.Interface.Multimedia.IAudio import IAudio
from acs_test_scripts.Device.UECmd.Imp.Windows.Common.Base import Base


class Audio(Base, IAudio):

    """
    Class that handles all multimedia audio related operations.
    """

    def __init__(self, device):
        """
        Initializes this instance.
        """
        Base.__init__(self, device)
        IAudio.__init__(self, device)
