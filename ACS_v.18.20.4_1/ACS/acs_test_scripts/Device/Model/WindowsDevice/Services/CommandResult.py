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
:since: 17/06/2014
:author: dgonza4x
"""

from CommandResultException import CommandResultException


class CommandResult:

    """
    Handle the result of each service method.
    """

    VERDICT_PASS = 0
    """
    Verdict defining a command result successful
    """

    VERDICT_FAILURE = 1
    """
    Verdict defining a command result failed
    """

    __DEFAULT_VALUE_NAME = "values"
    """
    Artifact from previous architecture, in which we had to retrieve
    "values" before retrieving our own specific data
    """

    def __init__(self, verdict, message, data=None, raised_excp=None, dev_logs=None):
        """
        Constructor

        :type verdict: int
        :param verdict: Verdict of the command result. See class constants.

        :type message: str
        :param message: Verdict message of the command result

        :type data: dict
        :param data: specific output data retrieved from a service command

        :type raised_excp: CommandResultException
        :param raised_excp: Optional parameter used when an error occurs inside a service command

        :type dev_logs: str
        :param dev_logs: retrieved developer logs
        """
        self.__verdict = verdict
        self.__verdict_msg = message
        self.__developer_logs = dev_logs
        if data is None or not isinstance(data, dict):
            self.__values = {}
        else:
            self.__values = data
        self.__raised_excp = raised_excp

    def __getitem__(self, item):
        """
        Retrieve a specific data from the instance like a dictionary

        :type item: str
        :param item: specific value name to retrieve

        :rtype: str
        :return: data associated to the specific value name
        """
        if str(item).lower() == self.__DEFAULT_VALUE_NAME:
            return self
        if item in self.__values.keys():
            return self.__values[item]
        else:
            raise CommandResultException("No specific value named '%s'" % str(item))

    def get(self, item):
        """
        Retrieve a specific data from the instance like a dictionary

        :type item: str
        :param item: specific value name to retrieve

        :rtype: str
        :return: data associated to the specific value name
        """
        return self.__getitem__(item)

    def keys(self):
        """
        Returns the set of all specific values name

        :rtype: list
        :return: list of all specific value names
        """
        return self.__values.keys()

    def has_key(self, key):
        """
        Check whether the value name is present or absent.

        :type key: str
        :param key: specific value name to find

        :rtype: bool
        :return: True is the key is present, False otherwise
        """
        return key in self.__values

    def __setitem__(self, key, value):
        """
        This method prevent developers to change any specific data.
        :attention: Unlike dictionary behavior, specific data cannot be modified.

        :type key: str
        :param key: specific value name to modify

        :type value: str
        :param value: data to set
        """
        raise CommandResultException(
            "Specific values of a %s instance cannot be modified (key : '%s')!"
            % (str(self.__class__.__name__), key))

    def get_verdict(self):
        """
        Verdict getter.

        :rtype: int
        :return: Verdict as integer. See class constants
        """
        return self.__verdict

    def get_verdict_message(self):
        """
        Message getter.

        :rtype: str
        :return: Verdict message.
        """
        return self.__verdict_msg

    def has_exception(self):
        """
        Define if the instance contain an exception

        :rtype: bool
        :return: True if the instance contains an exception, False otherwise
        """
        return self.__raised_excp is not None

    def get_raised_exception(self):
        """
        Raised exception getter

        :rtype: CommandResultException
        :return: Returns the exception occurred during the command processing, or None
        """
        return self.__raised_excp

    def has_developer_logs(self):
        """
        Define if the instance contain developer logs

        :rtype: bool
        :return: True if the instance contains developer logs, False otherwise
        """
        return self.__developer_logs is not None and self.__developer_logs != ""

    def get_developer_logs(self):
        """
        Developer logs getter

        :rtype: str
        :return: Returns the developer logs
        """
        return self.__developer_logs

    def __str__(self):
        """
        Human readable representation of the instance

        :rtype: str
        :return: Human readable representation of this instance
        """
        verdict = "FAILURE"
        if self.__verdict == 0:
            verdict = "SUCCESS"
        msg = "{Verdict: %s, Message: '%s', Data: %s" %\
              (verdict, self.__verdict_msg, str(self.__values))
        if self.has_exception():
            msg += ", Raised exception: %s" % str(self.__raised_excp)
        if self.has_developer_logs():
            msg += ", Developer logs: '%s'" % str(self.__developer_logs)
        msg += "}"
        return msg

if __name__ == "__main__":
    va = CommandResult(CommandResult.VERDICT_PASS, "All is fine",
                       {'abc': 12, 'def': "define_key def"}, dev_logs="DEV LOGS")
    print va
    print va['def']
    try:
        va['def'] = 12
    except CommandResultException as excp:
        print str(excp)
