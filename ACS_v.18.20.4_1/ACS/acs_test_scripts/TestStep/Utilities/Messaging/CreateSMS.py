"""
@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

@summary: Create a SMS object before sending it
@since 16 March 2015
@author: Martin Brisbarre
@organization: INTEL QCTV
"""
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from acs_test_scripts.Utilities.SmsUtilities import SmsMessage


class CreateSMS(DeviceTestStepBase):

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """

        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

        self._messaging_api = self._device.get_uecmd("SmsMessaging")
        self._equipment_manager = None

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        # Get access to the EquipmentManager instance
        self._equipment_manager = self._factory.create_equipment_manager()

        net_sim = self._equipment_manager.get_cellular_network_simulator(self._pars.eqt, visa=True)
        ns_messaging = net_sim.get_cell().get_messaging()

        sms = SmsMessage(self._pars.sms_text,
                         self._pars.phone_number,
                         self._pars.transportation,
                         ns_messaging,
                         self._messaging_api,
                         self._pars.coding_scheme,
                         self._pars.nb_bits_per_char,
                         self._pars.sms_transfer_timeout,
                         self._pars.content_type,
                         self._pars.sms_direction)

        context.set_info(self._pars.sms, sms)
