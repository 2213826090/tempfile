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
:summary: This file implements the Miscellaneous UEcmd for Android ICS device
:since: 02/12/2011
:author: ssavrimoutou
"""
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Misc.PhoneSystem import PhoneSystem as PhoneSystemCommon
from ErrorHandling.DeviceException import DeviceException


class PhoneSystem(PhoneSystemCommon):

    """
    :summary: PhoneSystem UEcommands operations for ICS Android platforms
    using an C{Intent} based communication to the I{DUT}.
    """

    def __init__(self, phone):
        """
        Constructor.

        """
        PhoneSystemCommon.__init__(self, phone)

        # ICS settings for disabling lockscreen
        self._locksettingdb = "/data/data/com.android.providers.settings/databases/settings.db"
        self._locksetting_table = "secure"
        self._required_records = {'lockscreen.disabled': "1",
                                  'lock_pattern_autolock': "0",
                                  'lockscreen.password_type': "65536",
                                  'lockscreen.password_type_alternate': "0"}
        self._phone = phone

    def generate_ap_crash(self):
        """
        Generate an AP crash onto the phone
        """
        self._exec("adb remount", 3, False)
        self._exec("adb shell mount -t debugfs none /d 2>/dev/null", 3)
        self._exec("adb shell echo 1 > /d/emmc_ipanic", 3)

    def disable_lockscreen(self, wait_settle_down_duration=False, reboot=True):
        """
        Disable the lockscreen. Need a reboot of the board to be applied

        :type wait_settle_down_duration: Integer
        :param wait_settle_down_duration: On reboot wait settle down duration if needed
                the duration is read from catalog in PhoneBase class.

        :type reboot: boolean
        :param reboot: do reboot if necessary
        """

        # The database must be accessible (root shell on the device is
        # mandatory)
        output = self._exec("adb shell ls %s" % self._locksettingdb)
        if output != self._locksettingdb:
            whoami = self._exec("adb shell id")
            raise DeviceException(DeviceException.OPERATION_FAILED, "Can not access %s file (return '%s') - "
                                                                    "user id is '%s'" % (self._locksettingdb,
                                                                                         output,
                                                                                         whoami))
        # If there is no settings in database, _exec will return error message
        # "No response from ADB" even command succeed
        # Add "echo 0" after record to bypass such event.
        sqlite3_connect = "adb shell sqlite3 " + self._locksettingdb + " \"%s\" && echo 0"
        sql_select = "select name, value from " + self._locksetting_table + " where name in ('" + \
            "', '".join(self._required_records.keys()) + "')"

        output = self._exec(sqlite3_connect % sql_select)

        recordstr = output.splitlines()
        recordset = {}
        for line in recordstr:
            if not line or line == "0":
                continue

            name, value = line.split('|')
            recordset[name] = value

        sql_request = ""
        for fieldname in self._required_records:
            if fieldname not in recordset:
                # Need to add record in database
                sql_request += "insert into " + self._locksetting_table + \
                               " (name, value) values ('%s', '%s');" % \
                               (fieldname, self._required_records[fieldname])
            elif recordset[fieldname] != self._required_records[fieldname]:
                # Need to update value in database
                sql_request += "update " + self._locksetting_table + \
                               " set value='%s' where name='%s';" % \
                               (self._required_records[fieldname], fieldname)

        if sql_request:
            self._exec(sqlite3_connect % sql_request)
            if reboot:
                self._phone.reboot(wait_settledown_duration=wait_settle_down_duration)
                return False
            else:
                return True

        return False
