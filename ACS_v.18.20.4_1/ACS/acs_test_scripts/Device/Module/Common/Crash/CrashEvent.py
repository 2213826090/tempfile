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
:summary: This file implements the CrashEvent class used by the CrashInfoModule class
:since: 07/07/2014
:author: kturban
"""
# pylint: disable=E1002
# pylint: disable=C0103
from UtilitiesFWK.AttributeDict import AttributeDict


class CrashEvent(AttributeDict):

    """
    Class to store crash event information
    """

    def __init__(self, eventId, type, date, crashdir, crashtoolUrl, *args, **kwargs):
        """
        Constructor
        """
        super(CrashEvent, self).__init__(*args, **kwargs)
        self["eventID"] = eventId
        self["eventType"] = type
        self["date"] = date
        self["crashdir"] = crashdir
        self["crashtoolUrl"] = crashtoolUrl

    def __str__(self):
        """
        Override str message in order to have human readable representation of the event
        """
        string = "CRASHLOG: {0} {1} {2} {3} {4}".format(self.eventType,
                                                        self.eventID,
                                                        self.date,
                                                        self.crashdir,
                                                        self.crashtoolUrl)
        return string
