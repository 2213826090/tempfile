# pylint: disable = C0303
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

:organization: INTEL MCG PSI
:summary: This file define an Exception class handling http request problems
:since: 18/06/2014
:author: dgonza4x
"""
from exceptions import Exception


class CommandResultException(Exception):

    """
    Error occurring during bad manipulation of a CommandResult instance
    """

    def __init__(self, message, traceback=None):
        """
        Constructor

        :type message: str
        :param message: Exception explanations

        :type traceback: str
        :param traceback: String representation of exception traceback.
        """
        Exception.__init__(self, message)
        self.__traceback = traceback

    def has_traceback(self):
        """
        Define whether the instance contains a traceback or not

        :rtype: bool
        :return: True is the exception contains a traceback, False otherwise
        """
        return self.__traceback is not None and self.__traceback != ""

    def get_traceback(self):
        """
        Retrieve the traceback if any

        :rtype: str
        :return: String representing an exception traceback, or None if no traceback is set
        """
        return self.__traceback

    def get_message(self):
        """
        Retrieve the exception message

        :rtype: str
        :return: exception message
        """
        return str(self)
