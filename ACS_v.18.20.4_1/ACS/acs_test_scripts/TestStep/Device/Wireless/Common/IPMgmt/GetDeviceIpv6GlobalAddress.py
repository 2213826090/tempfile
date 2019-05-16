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
otherwise. Any license under such intellectual property rights must be expressed
and approved by Intel in writing.

:summary: This file implements a Test Step to get the device IPV6 global address
:since: 01/08/2014
:author: Jerome Durand
:organization: INTEL QCTV
"""

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase


class GetDeviceIpv6GlobalAddress(DeviceTestStepBase):
    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        networking_api = self._device.get_uecmd("Networking")

        ip_addr = networking_api.get_interface_ipv6_global_address(self._pars.net_interface)
        context.set_info(self._pars.ip_addr, ip_addr)
        # Raising an error if function result is not PASS is already performed at UE CMD level.
        # no need to do it here
        self._logger.info("Global IPv6 address is %s" % ip_addr)
