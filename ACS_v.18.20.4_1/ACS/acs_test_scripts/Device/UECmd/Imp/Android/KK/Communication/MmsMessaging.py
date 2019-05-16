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
:summary: This file implements the Messaging UEcmd for KitKat Android device
:since: 06/12/2013
:author: rbertolx
"""
import time

from acs_test_scripts.Device.UECmd.Imp.Android.ICS.Communication.\
MmsMessaging import MmsMessaging as MmsMessagingCommon
from acs_test_scripts.Device.UECmd.UECmdTypes import MSG_DB_PATH


class MmsMessaging(MmsMessagingCommon):
    """
    :summary: MmsMessaging UEcommands operations for 4.4 Android platforms
    using an C{Intent} based communication to the I{DUT}.
    """

    def __init__(self, device):
        """
        Constructor.
        """
        MmsMessagingCommon.__init__(self, device)

    def delete_all_messages(self):
        """
        Deletes all the SMS and MMS present on the phone.
        .. warning:: Need specific implementation for 4.4 android as since api 19
        No application but the default SMS-MMS application can modify the
        SMS-MMS database.
        """
        self._logger.info("Deleting all SMS and MMS.")
        # Get list of all tables in the database.
        table_list_str = self._exec("adb shell sqlite3 %s \".tables\""
                                    % MSG_DB_PATH)
        time.sleep(1)
        # Filters out all none empty str of the list.
        table_list = filter(lambda x: x, table_list_str.split(" "))
        # delete all content in those databases.
        for table in table_list:
            self._exec("adb shell sqlite3 %s \"DELETE FROM %s\""
                       % (MSG_DB_PATH, table))
            time.sleep(1)
