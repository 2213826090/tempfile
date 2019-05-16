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
:summary: This file implements a Test Step that Cleans up OPP
:since:19/12/2013
:author: fbongiax
"""


class Constants(object):
    """
    Defines bluetooth constants
    """

    FILE_NAME_SEPARATOR = ","

    OPP_SPEED_FOR_TIMEOUT = (1024.0 * 1024.0 / (1.5 * 9.0))

    OPP_INFO_FILE_SIZE = "FileSize"
    OPP_INFO_FILE_CHKSUM = "FileChecksum"

    OPP_STATE_WAITING = "waiting"
    OPP_STATE_DOWNLOADED = "downloaded"
    OPP_STATE_DOWNLOADING = "downloading"
    OPP_STATE_CANCELLED = "cancelled"

    PROFILE_CONNECTED = "connected"
    PROFILE_DISCONNECTED = "disconnected"

    DISCOVERABLE_BOTH = "both"