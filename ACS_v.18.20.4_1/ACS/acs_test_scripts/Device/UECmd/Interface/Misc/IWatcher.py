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
:summary: This script implements the interface of Watcher UECmd for Android
phones
:since: 22/02/2013
:author: jduran4x
"""
from ErrorHandling.DeviceException import DeviceException

# pylint: disable=W0613


class IWatcher():

    """
    Abstract class that defines the interface to be implemented
    by device system operations handling sub classes.

    All method that shall be redefined in sub-classes raise a
    I{DeviceException} error.
    """
    # pylint: disable=W0613

    def __init__(self, phone):
        """
        Initializes this instance.

        Nothing to be done in abstract class.
        """
        pass

    def start_watch_pdp_contexts(self):
        """
        Starts the watcher of Pdp Context number

        :return: None
        :raise DeviceException: if the command failed
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def stop_watch_pdp_contexts(self):
        """
        Stops the watcher of Pdp Context number

        :return: None
        :raise DeviceException: if the command failed
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_pdp_contexts_number(self):
        """
        retreives the max number of pdp contexts
        activated simultaneously

        :rtype: dict
        :return: the output parameters in a dictionary.
        :raise DeviceException: if the command failed
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
