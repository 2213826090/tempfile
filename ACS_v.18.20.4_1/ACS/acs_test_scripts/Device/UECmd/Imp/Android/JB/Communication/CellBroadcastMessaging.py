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
:summary: This file implements the CellBroadcastMessaging UEcmd for Android device
:since: 20/02/2013
:author: hbian
"""
from acs_test_scripts.Device.UECmd.Interface.Communication.ICellBroadcastMessaging import ICellBroadcastMessaging
from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
from acs_test_scripts.Device.UECmd.UECmdDecorator import need
from ErrorHandling.DeviceException import DeviceException


class CellBroadcastMessaging(BaseV2, ICellBroadcastMessaging):

    """
    :summary: CellBroadcastMessaging UEcommands operations for Android platforms
    using an C{Intent} based communication to the I{DUT}.
    """

    @need('modem')
    def __init__(self, device):
        """
        Constructor.
        """
        BaseV2.__init__(self, device)
        ICellBroadcastMessaging.__init__(self, device)
        self._logger = device.get_logger()
        self.component = "com.intel.acs.agent/.CellBroadcastMessaging"
        self.iaction = "intel.intents.action.CB_SMS"
        self._module = "acscmd.telephony.messaging.SmsCbModule"

    def wait_for_incoming_cell_broadcast_sms(self, timeout):
        """
        Wait for incoming cell_broadcast_sms, will return successfully
        :type timeout: int
        :param timeout: time in seconds to wait before cell_broadcast_sms received

        :rtype: dictionary
        :return: the dictionary of sms cb include sms cb text,sms cb category
                 sms cb serial number

        :raise DeviceException: if the command timeout is reached or the device
        doesn't receive the sms cb
        """
        cb_sms_dictionary = {}
        start_function = "waitForIncomingCellBroadcastSms"
        try:
            results = self._internal_exec_v2(self._module,
                                             start_function,
                                             timeout=timeout, is_system=True)
        except DeviceException:
            # The device doesn't receive the sms cb from ACS agent in time
            # Stop waiting the sms cb incoming.
            stop_function = "stopWaitingCellBroadcastSms"
            try:
                self._internal_exec_v2(self._module, stop_function, is_system=True)
            except DeviceException:
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                      "stopWaitingCellBroadcastSms on ACS agent failed")
            raise DeviceException(DeviceException.TIMEOUT_REACHED,
                                  "Did not receive the CB SMS after " + str(timeout) + " seconds")

        if "sms_cb_text" in results:

            cb_sms_dictionary.update({"sms_cb_text": str(results["sms_cb_text"])})
            cb_sms_dictionary.update({"sms_cb_category": results["sms_cb_category"]})
            cb_sms_dictionary.update({"sms_cb_serial_number": results["sms_cb_serial_number"]})

            self._logger.info("received cb sms :: %s" % str(cb_sms_dictionary))

            return cb_sms_dictionary
        else:
            raise DeviceException(DeviceException.SMS_EXCEPTION, "Can't get cell broadcast sms text")

    def clear_all_cell_broadcast_sms(self):
        """
        Clears all cell broadcast sms

        :return: None

        :raise DeviceException: if sms cb can't be removed
        """
        cb_sms_database_path = ("/data/data/com.android.cellbroadcastreceiver"
                                "/databases/cell_broadcasts.db")

        sql_delete_cmd = '''"delete from broadcasts;"'''
        sql_get_cb_sms_count = '''"select count(_id) from broadcasts;"'''

        delete_cmd = "adb shell sqlite3 " + cb_sms_database_path + " " + sql_delete_cmd

        get_cb_sms_count_nb = "adb shell sqlite3 " + cb_sms_database_path + " " + sql_get_cb_sms_count

        if "databases" in self._exec("adb shell ls /data/data/com.android.cellbroadcastreceiver"):

            # Delete all the sms cb from the database
            self._exec(delete_cmd)

            # check the sms cb count should be 0
            cb_sms_count = self._exec(get_cb_sms_count_nb)

            if int(cb_sms_count) == 0:
                self._logger.info("all the cb sms have been successfully cleared")
            else:
                raise DeviceException(DeviceException.OPERATION_FAILED, " %s cb sms are not removed" % cb_sms_count)
        else:
            self._logger.info("No cb sms to remove")
