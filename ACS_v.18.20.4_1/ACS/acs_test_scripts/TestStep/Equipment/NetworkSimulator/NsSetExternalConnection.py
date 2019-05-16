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

:summary: This file implements a Test Step which enable/disable the external connection from one 8960 to another
:author 03/09/2015
:organization: INTEL QCTV
"""

from Core.TestStep.TestStepBase import TestStepBase


class NsSetExternalConnection(TestStepBase):
    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        TestStepBase.run(self, context)

        # Get access to the EquipmentManager instance
        self._equipment_manager = self._factory.create_equipment_manager()

        net_sim1 = self._equipment_manager.get_cellular_network_simulator(self._pars.eqt1, visa=True)
        net_sim2 = self._equipment_manager.get_cellular_network_simulator(self._pars.eqt2, visa=True)

        if self._pars.state == "ON":
            ip_eqt1 = net_sim1._bench_params.get_param_value("IP_Lan2")
            ip_eqt2 = net_sim2._bench_params.get_param_value("IP_Lan2")
            # Set external 8960 Ip Address with Agilent 8960 idle
            net_sim1.set_external_ip_address(ip_eqt2)

            # Set external 8960 Ip Address with Agilent 8960 active
            net_sim2.set_external_ip_address(ip_eqt1)

            # Connect to external 8960
            net_sim1.connect_to_external_device()

            # Verify ethernet connection before timeout
            # 5 seconds using the Agilent 8960 idle IP address
            net_sim1.check_external_device_connection(1, ip_eqt2, 5)

            # Verify ethernet connection before timeout
            # 5 seconds using the Agilent 8960 active IP address
            net_sim2.check_external_device_connection(1, ip_eqt1, 5)
        elif self._pars.state == "OFF":
            # Disconnect from external 8960
            net_sim1.disconnect_from_external_device()

            # Disconnect from external 8960
            net_sim2.disconnect_from_external_device()

            # Verify ethernet disconnection
            net_sim2.check_external_device_disconnection(1, 5)
