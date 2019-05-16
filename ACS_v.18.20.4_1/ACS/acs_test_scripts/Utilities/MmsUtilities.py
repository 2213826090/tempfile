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
:summary: Utilities class for MMS implementation
:since: 02/07/2011
:author: lvacheyx
"""

import os

class MmsMessage:
    """
    Structure that represent MMS message
    """
    def __init__(self, sender_number, destination_number, direction, subject, text,
                 media, attachment_name, attachment_path):

        # MMS sender phone number (for MT MMS)
        self._sender_number = sender_number
        # MMS destination phone number
        self._destination_number = destination_number
        # MMS direction (MO or MT)
        self._direction = direction
        # MMS subject (text format)
        self._subject = subject
        # MMS text
        self._text = text
        # MMS media (picture, audio, video...)
        self._media = media
        # MMS attachment name
        self._attachment_name = attachment_name
        # MMS attachment path
        self._attachment_path = attachment_path
        # Retrieve the path of the MMS attachment by concatenating the
        # attachment path and name
        # Can be null if the mms_type is "text".
        if self._attachment_name is not None:
            self._attachment_file = os.path.join(
                self._attachment_path,
                self._attachment_name)
        else:
            self._attachment_file = ""


def compare(self, mms_sent, mms_received):
    """

    :param mms_sent:
    :type mms_sent:

    :param mms_received:
    :type mms_received:

    .. todo:: Compare MMS sent and received

    """
    return 1
