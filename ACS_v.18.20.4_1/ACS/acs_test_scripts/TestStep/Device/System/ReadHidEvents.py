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
:summary: This file implements a Test Step to read HID events
:since:19/02/2014
:author: fbongiax
"""

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase


class ReadHidEvents(DeviceTestStepBase):
    """
    Reads HID events
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory=None):
        """
        Constructor
        """
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._api = self._device.get_uecmd("System")

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        events = self._pars.events.split(",")
        self._raise_error_if_empty_list(events)
        self._raise_error_if_hid_device_is_invalid()

        self._logger.info("Waiting for the following HID events: %s for %d seconds" % (str(events), self._pars.timeout))

        if not self._api.check_hid_events(self._pars.hid_device, events, self._pars.timeout):
            self._raise_device_exception("Read events don't match the expected sequence")

    def _raise_error_if_empty_list(self, events):
        """
        If the list is empty, it raises an error
        """
        if len(events) == 0 or not events[0]:
            self._raise_config_exception("Events list is empty")

    def _raise_error_if_hid_device_is_invalid(self):
        """
        HID_DEVICE must be valid
        """
        if not self._pars.hid_device:
            self._raise_config_exception("HID_DEVICE must be valid")
