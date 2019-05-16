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

:organization: INTEL NDG sw
:summary: This file implements the comparison between the reference and DUT time
:since: 2014-10-21
:author: dpierrex

"""
from datetime import datetime
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase


class CheckTime(DeviceTestStepBase):

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._system_api = self._device.get_uecmd("System")

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        ret_code, dut_time = self._system_api.get_date_and_time()

        device_ts = datetime.strptime(dut_time, "%Y-%m-%d %H:%M:%S")
        ref_ts = datetime.strptime("{0}-{1}".format(self._pars.date_ref, self._pars.time_ref), "%Y-%m-%d-%H:%M:%S")

        time_delta = abs(device_ts - ref_ts)

        # Workaround to compute total seconds in python2.6
        # https://bitbucket.org/wnielson/django-chronograph/issue/27/python26-datatimetimedelta-total_seconds
        if hasattr(time_delta, "total_seconds"):
            duration = time_delta.total_seconds()
        else:
            duration = (time_delta.microseconds + (time_delta.seconds + time_delta.days * 24 * 3600) * 10 ** 6) / 10 ** 6

        if duration > self._pars.gap_allowed:
            context.set_info(self._pars.save_as, "false")
        else:
            context.set_info(self._pars.save_as, "true")

