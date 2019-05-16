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
:summary: This script implements the interface to unitary actions for
Ui features.
:since: 14/12/2011
:author: ssavrimoutou
"""
from ErrorHandling.DeviceException import DeviceException

# pylint: disable=W0613


class IUi():

    """
    Abstract class that defines the interface to be implemented
    by device system operations handling sub classes.

    All method that shall be redefined in sub-classes raise a
    I{DeviceException} error.
    """

    def __init__(self, device):
        """
        Initializes this instance.

        Nothing to be done in abstract class.
        """
        pass

    def init(self, error_policy=None):
        """
        Used to establish connection to a port, and/or all other action to do to initialize the Ui uecmd instance

        :type error_policy: str
        :param error_policy: (Optional) Used for Android to manage error displaying
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def release(self):
        """
        Used to disconnect from a port, and/or all other action to do to release the Ui uecmd instance
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def run_action_from_script(self, script_name):
        """
        Execute UI action defined from xml scripts

        :type script_name: str
        :param script_name: The path to the script which contains all UI sequences to execute
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def run_operation_set(self, operation_set_name, parameters=None):
        """
        Execute operation set defined from Device OpDictionary

        :type operation_set_name: str
        :param operation_set_name: The name of the operation set, it MUST be defined in the OpDictionary

        :type parameters: list
        :param parameters: the list of parameters for this operation set, it MUST be defined in the OpDictionary

        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
