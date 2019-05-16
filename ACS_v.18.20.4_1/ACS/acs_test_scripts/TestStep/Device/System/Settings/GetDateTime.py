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
:summary: This file implements the read of the DUT's date and time
:since: 2014-10-21
:author: dpierrex

"""
from datetime import datetime
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
import UtilitiesFWK.Utilities as Util


class GetDateTime(DeviceTestStepBase):

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

        date_format = self._pars.date_format
        if date_format in ["", None]:
            date_format = "%Y-%m-%d"
        time_format = self._pars.time_format
        if time_format in ["", None]:
            time_format = "%H:%M:%S"

        return_code, device_date_and_time = self._system_api.get_date_and_time()
        if return_code == Util.Global.FAILURE:
            raise DeviceException(DeviceException.OPERATION_FAILED, "Unable to get the date from the DUT: {0}".format(device_date_and_time))

        device_ts = datetime.strptime(device_date_and_time, "%Y-%m-%d %H:%M:%S")
        try:
            date = device_ts.strftime(date_format)
            time = device_ts.strftime(time_format)
        except ValueError, e:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Bad date or time format : {0}".format(e.message))

        context.set_nested_info([self._pars.save_date_time_as, "DATE"], date)
        context.set_nested_info([self._pars.save_date_time_as, "TIME"], time)
