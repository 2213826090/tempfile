"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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

:organization: INTEL NDG SW DEV
:summary: This file implements a Test Step to get gatt notifications (bluetooth LE)
:since:13/10/2014
:author: jreynaux
"""
from TestStep.Device.Wireless.BT.Base import BtBase
from acs_test_scripts.Device.UECmd.UECmdTypes import BtGattNotification


class BtGattGetNotification(BtBase):
    """
    Implements the BtGattSubscribeNotification test step for BT
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        BtBase.run(self, context)

        service_name = str(self._pars.service_name)
        char_name = str(self._pars.characteristic_name)
        notif_type = None
        if self._pars.notification_type is not None:
            notif_type = str(self._pars.notification_type)
        time_lap = self._pars.time_lap
        notif_found = False

        self._logger.info("Retrieve notification about '{0}' characteristic on '{1}' service, until {2} minutes ago".
                          format(char_name, service_name, time_lap))

        notifications_list = self._api.bt_gatt_get_notification(service_name=service_name, char_name=char_name,
                                                                time_lap=time_lap)

        if notifications_list is not None and len(notifications_list) > 0:
            index = 0
            for i, notification in enumerate(notifications_list):
                if notif_type is not None:
                    if notification.type != notif_type:
                        self._logger.debug("Notification type is not {0} ({1}). Notification filtered".
                                           format(notif_type, notification.type))
                        continue

                notif_found = True
                index += 1
                self._logger.debug("Notification saved as {0}:NOTIFICATION_{1}".format(self._pars.save_as, index))

                key = "{0}:NOTIFICATION_{1}:{2}".format(self._pars.save_as, index, "DATE")
                value = notification.date
                context.set_info(key, value)
                self._logger.debug("{0} = {1}".format(key, value))

                key = "{0}:NOTIFICATION_{1}:{2}".format(self._pars.save_as, index, "SERVICE")
                value = notification.service
                context.set_info(key, value)
                self._logger.debug("{0} = {1}".format(key, value))

                key = "{0}:NOTIFICATION_{1}:{2}".format(self._pars.save_as, index, "CHARACTERISTIC")
                value = notification.characteristic
                context.set_info(key, value)
                self._logger.debug("{0} = {1}".format(key, value))

                key = "{0}:NOTIFICATION_{1}:{2}".format(self._pars.save_as, index, "TYPE")
                value = notification.type
                context.set_info(key, value)
                self._logger.debug("{0} = {1}".format(key, value))

                for data_tmp in notification.data:
                    key = "{0}:NOTIFICATION_{1}:{2}".format(self._pars.save_as, index, data_tmp)
                    value = notification.data[data_tmp]
                    context.set_info(key, value)
                    self._logger.debug("{0} = {1}".format(key, value))

        if not notif_found:
            error_message = "No notification found on '{0}' characteristic".format(char_name)
            error_message += " in '{0}' service".format(service_name)
            if notif_type is not None:
                error_message += " with {0} type".format(notif_type)
            error_message += ", within {0} minutes. Result returned: ".format(time_lap)
            self._logger.debug(error_message)

            key = "{0}:NOTIFICATION_1:TYPE".format(self._pars.save_as)
            context.set_info(key, "not_found")
            self._logger.debug("{0} = {1}".format(key, "not_found"))
