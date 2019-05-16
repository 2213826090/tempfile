# @PydevCodeAnalysisIgnore
# pylint: disable=E0602,W0212,C0103,C0111
"""
@summary: Unit test module for CHECK_BT_IPERF_RESULTS test step.

@since 24 June 2014
@author: Val Peterson
@organization: INTEL PEG-SVE-DSV

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
otherwise. Any license under such intellectual property rights must be expressed
and approved by Intel in writing.
"""

import mock
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from Core.TestStep.TestStepContext import TestStepContext
from acs_test_scripts.TestStep.Device.Comms.DataXfer.Ping import Ping
from acs_test_scripts.Device.UECmd.UECmdTypes import Measure


class Test(UTestTestStepBase):
    DESTINATION_IPV4 = "192.168.0.150"
    DESTINATION_IPV6 = "AAAA:BBBB:CCCC:DDDD:EEEE:FFFF"
    SOURCE_IP_NULL = "None"
    SOURCE_IPV4 = "192.168.0.42"
    SOURCE_IPV6 = "AAAA:BBBB:CCCC:DDDD:EEEE:0000"
    PACKET_SIZE = "16"
    PACKET_COUNT = "16"
    INTERVAL = "5.0"
    SAVE_AS = "PING_PACKET_LOSS_SAVE"
    PACKET_LOSS_SUCCESS = 0
    PACKET_LOSS_FAIL = 100

    def _ping6(self, ip_address, packet_size, packet_count, flood_mode, blocking,
               source_address):
        return self._ping(ip_address, packet_size, packet_count, interval=1, flood_mode=flood_mode,
                          blocking=blocking, source_address=source_address)

    def _ping(self, server_ip_address, packet_size, packet_count, interval, flood_mode, blocking,
              source_address):
        packet_loss = Measure()
        packet_loss.value = self._ping_return
        return packet_loss

#------------------------------------------------------------------------------
    def setUp(self):
        UTestTestStepBase.setUp(self)
        self._context = TestStepContext()
        self._sut = None
        self._ping_return = None

    def test_ping_ipv4_ok(self):
        self._create_sut(self.DESTINATION_IPV4, self.SOURCE_IP_NULL, self.PACKET_SIZE, self.PACKET_COUNT,
                         self.INTERVAL, False, False, self.SAVE_AS)
        self._ping_return = self.PACKET_LOSS_SUCCESS
        self._context = TestStepContext()
        self._assert_run_succeeded_with_msg(self._sut,
                                            "VERDICT: %s stored as %s" % (self.SAVE_AS, self.PACKET_LOSS_SUCCESS))
        self.assertEqual(self._context.get_info(self.SAVE_AS), str(self.PACKET_LOSS_SUCCESS))

    def test_ping_ipv6_ok(self):
        self._create_sut(self.DESTINATION_IPV6, self.SOURCE_IP_NULL, self.PACKET_SIZE, self.PACKET_COUNT,
                         self.INTERVAL, False, False, self.SAVE_AS)
        self._ping_return = self.PACKET_LOSS_SUCCESS
        self._context = TestStepContext()
        self._assert_run_succeeded_with_msg(self._sut,
                                            "VERDICT: %s stored as %s" % (self.SAVE_AS, self.PACKET_LOSS_SUCCESS))
        self.assertEqual(self._context.get_info(self.SAVE_AS), str(self.PACKET_LOSS_SUCCESS))

    def test_ping_ipv4_all_packet_lost(self):
        self._create_sut(self.DESTINATION_IPV4, self.SOURCE_IP_NULL, self.PACKET_SIZE, self.PACKET_COUNT,
                         self.INTERVAL, False, False, self.SAVE_AS)
        self._ping_return = self.PACKET_LOSS_FAIL
        self._context = TestStepContext()
        self._assert_run_succeeded_with_msg(self._sut,
                                            "VERDICT: %s stored as %s" % (self.SAVE_AS, self.PACKET_LOSS_FAIL))
        self.assertEqual(self._context.get_info(self.SAVE_AS), str(self.PACKET_LOSS_FAIL))

    def test_ping_ipv6_all_packet_lost(self):
        self._create_sut(self.DESTINATION_IPV6, self.SOURCE_IP_NULL, self.PACKET_SIZE, self.PACKET_COUNT,
                         self.INTERVAL, False, False, self.SAVE_AS)
        self._ping_return = self.PACKET_LOSS_FAIL
        self._context = TestStepContext()
        self._assert_run_succeeded_with_msg(self._sut,
                                            "VERDICT: %s stored as %s" % (self.SAVE_AS, self.PACKET_LOSS_FAIL))
        self.assertEqual(self._context.get_info(self.SAVE_AS), str(self.PACKET_LOSS_FAIL))

    def test_ping_ipv4_flood_blocking(self):
        self._create_sut(self.DESTINATION_IPV4, self.SOURCE_IP_NULL, self.PACKET_SIZE, self.PACKET_COUNT,
                         self.INTERVAL, True, True, self.SAVE_AS)
        self._ping_return = self.PACKET_LOSS_SUCCESS
        self._context = TestStepContext()
        self._assert_run_succeeded_with_msg(self._sut,
                                            "VERDICT: %s stored as %s" % (self.SAVE_AS, self.PACKET_LOSS_SUCCESS))
        self.assertEqual(self._context.get_info(self.SAVE_AS), str(self.PACKET_LOSS_SUCCESS))

    def test_ping_ipv4_source_address(self):
        self._create_sut(self.DESTINATION_IPV4, self.SOURCE_IPV4, self.PACKET_SIZE, self.PACKET_COUNT,
                         self.INTERVAL, True, True, self.SAVE_AS)
        self._ping_return = self.PACKET_LOSS_SUCCESS
        self._context = TestStepContext()
        self._assert_run_succeeded_with_msg(self._sut,
                                            "VERDICT: %s stored as %s" % (self.SAVE_AS, self.PACKET_LOSS_SUCCESS))
        self.assertEqual(self._context.get_info(self.SAVE_AS), str(self.PACKET_LOSS_SUCCESS))

    def test_ping_ipv6_source_address(self):
        self._create_sut(self.DESTINATION_IPV6, self.SOURCE_IPV6, self.PACKET_SIZE, self.PACKET_COUNT,
                         self.INTERVAL, True, True, self.SAVE_AS)
        self._ping_return = self.PACKET_LOSS_SUCCESS
        self._context = TestStepContext()
        self._assert_run_succeeded_with_msg(self._sut,
                                            "VERDICT: %s stored as %s" % (self.SAVE_AS, self.PACKET_LOSS_SUCCESS))
        self.assertEqual(self._context.get_info(self.SAVE_AS), str(self.PACKET_LOSS_SUCCESS))

    def _create_sut(self, destination_ip, source_ip, packet_size, packet_count, interval, flood_mode, blocking,
                    save_as):
        self._sut = Ping(None, None,
                         {"DESTINATION_IP": destination_ip, "SOURCE_IP": source_ip, "PACKET_SIZE": packet_size,
                          "PACKET_COUNT": packet_count, "INTERVAL": interval, "FLOOD_MODE": flood_mode,
                          "BLOCKING": blocking, "SAVE_AS": save_as}, mock.Mock())
        self._sut._networking_api.ping = self._ping
        self._sut._networking_api.ping6 = self._ping6
        return self._sut
