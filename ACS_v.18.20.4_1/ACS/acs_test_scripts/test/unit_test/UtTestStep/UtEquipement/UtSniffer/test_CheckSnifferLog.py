# @PydevCodeAnalysisIgnore
# pylint: disable=E0602,W0212,C0103,C0111
"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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
:summary: Unit test module
:since: 12/01/2015
:author: jfranchx
"""

import mock
from acs_test_scripts.test.unit_test.UtTestStep.UtEquipement.UtSniffer.test_SnifferCheckBase import SnifferCheckBaseTest
from acs_test_scripts.TestStep.Equipment.Sniffer.CheckSnifferLog import CheckSnifferLog


class CheckSnifferLogTest(SnifferCheckBaseTest):
    PARAM_FRAME_BEACON = "CHECK_FRAME_BEACON"
    PARAM_FRAME_ASSOCIATION_REQUEST = "CHECK_FRAME_ASSOCIATION_REQUEST"
    PARAM_CHECK_CAPABILITY = "CHECK_CAPABILITIES"
    PARAM_CHECK_BANDWIDTH = "CHECK_BANDWIDTH"
    PARAM_VALUE_HT_CAP = "HT_CAPABILITIES"
    PARAM_VALUE_VHT_CAP = "VHT_CAPABILITIES"
    PARAM_VALUE_20MHZ = "20MHZ"
    PARAM_VALUE_40MHZ = "40MHZ"
    PARAM_VALUE_80MHZ = "80MHZ"
    PARAM_MAC_SOURCE_OK = "11:22:33:44:55:66"
    PARAM_MAC_SOURCE_FAIL = "11:22:33:44:55:55"
    PARAM_MAC_RECEIVER_OK = "AA:BB:CC:DD:EE:FF"
    PARAM_MAC_RECEIVER_FAIL = "AA:BB:CC:DD:EE:EE"

    # --- MANDATORY PARAMS OK --- #
    def test_check_beacon_capabilities_ht_ok(self):
        try:
            self._create_sniffer_log_file_20mhz()
            sut = self._create_sut({"SNIFFER_LOG_FILE": self.FILE_NAME_20MHZ,
                                    "FRAME_TO_CHECK": self.PARAM_FRAME_BEACON,
                                    "PARAM_TO_CHECK": self.PARAM_CHECK_CAPABILITY,
                                    "PARAM_VALUE": self.PARAM_VALUE_HT_CAP})
            self._assert_run_succeeded(sut)
        finally:
            self._delete_sniffer_log_file(self.FILE_NAME_20MHZ)

    def test_check_beacon_capabilities_vht_ok(self):
        try:
            self._create_sniffer_log_file_80mhz()
            sut = self._create_sut({"SNIFFER_LOG_FILE": self.FILE_NAME_80MHZ,
                                    "FRAME_TO_CHECK": self.PARAM_FRAME_BEACON,
                                    "PARAM_TO_CHECK": self.PARAM_CHECK_CAPABILITY,
                                    "PARAM_VALUE": self.PARAM_VALUE_VHT_CAP})
            self._assert_run_succeeded(sut)
        finally:
            self._delete_sniffer_log_file(self.FILE_NAME_80MHZ)

    def test_check_beacon_bandwidth_20mhz_ok(self):
        try:
            self._create_sniffer_log_file_20mhz()
            sut = self._create_sut({"SNIFFER_LOG_FILE": self.FILE_NAME_20MHZ,
                                    "FRAME_TO_CHECK": self.PARAM_FRAME_BEACON,
                                    "PARAM_TO_CHECK": self.PARAM_CHECK_BANDWIDTH,
                                    "PARAM_VALUE": self.PARAM_VALUE_20MHZ})
            self._assert_run_succeeded(sut)
        finally:
            self._delete_sniffer_log_file(self.FILE_NAME_20MHZ)

    def test_check_beacon_bandwidth_40mhz_ok(self):
        try:
            self._create_sniffer_log_file_40mhz()
            sut = self._create_sut({"SNIFFER_LOG_FILE": self.FILE_NAME_40MHZ,
                                    "FRAME_TO_CHECK": self.PARAM_FRAME_BEACON,
                                    "PARAM_TO_CHECK": self.PARAM_CHECK_BANDWIDTH,
                                    "PARAM_VALUE": self.PARAM_VALUE_40MHZ})
            self._assert_run_succeeded(sut)
        finally:
            self._delete_sniffer_log_file(self.FILE_NAME_40MHZ)

    def test_check_beacon_bandwidth_80mhz_ok(self):
        try:
            self._create_sniffer_log_file_80mhz()
            sut = self._create_sut({"SNIFFER_LOG_FILE": self.FILE_NAME_80MHZ,
                                    "FRAME_TO_CHECK": self.PARAM_FRAME_BEACON,
                                    "PARAM_TO_CHECK": self.PARAM_CHECK_BANDWIDTH,
                                    "PARAM_VALUE": self.PARAM_VALUE_80MHZ})
            self._assert_run_succeeded(sut)
        finally:
            self._delete_sniffer_log_file(self.FILE_NAME_80MHZ)

    def test_check_association_request_capabilities_ht_ok(self):
        try:
            self._create_sniffer_log_file_20mhz()
            sut = self._create_sut({"SNIFFER_LOG_FILE": self.FILE_NAME_20MHZ,
                                    "FRAME_TO_CHECK": self.PARAM_FRAME_ASSOCIATION_REQUEST,
                                    "PARAM_TO_CHECK": self.PARAM_CHECK_CAPABILITY,
                                    "PARAM_VALUE": self.PARAM_VALUE_HT_CAP})
            self._assert_run_succeeded(sut)
        finally:
            self._delete_sniffer_log_file(self.FILE_NAME_20MHZ)

    def test_check_association_request_capabilities_vht_ok(self):
        try:
            self._create_sniffer_log_file_80mhz()
            sut = self._create_sut({"SNIFFER_LOG_FILE": self.FILE_NAME_80MHZ,
                                    "FRAME_TO_CHECK": self.PARAM_FRAME_ASSOCIATION_REQUEST,
                                    "PARAM_TO_CHECK": self.PARAM_CHECK_CAPABILITY,
                                    "PARAM_VALUE": self.PARAM_VALUE_VHT_CAP})
            self._assert_run_succeeded(sut)
        finally:
            self._delete_sniffer_log_file(self.FILE_NAME_80MHZ)

    def test_check_association_request_bandwidth_20mhz_ok(self):
        try:
            self._create_sniffer_log_file_20mhz()
            sut = self._create_sut({"SNIFFER_LOG_FILE": self.FILE_NAME_20MHZ,
                                    "FRAME_TO_CHECK": self.PARAM_FRAME_ASSOCIATION_REQUEST,
                                    "PARAM_TO_CHECK": self.PARAM_CHECK_BANDWIDTH,
                                    "PARAM_VALUE": self.PARAM_VALUE_20MHZ})
            self._assert_run_succeeded(sut)
        finally:
            self._delete_sniffer_log_file(self.FILE_NAME_20MHZ)

    def test_check_association_request_bandwidth_40mhz_ok(self):
        try:
            self._create_sniffer_log_file_40mhz()
            sut = self._create_sut({"SNIFFER_LOG_FILE": self.FILE_NAME_40MHZ,
                                    "FRAME_TO_CHECK": self.PARAM_FRAME_ASSOCIATION_REQUEST,
                                    "PARAM_TO_CHECK": self.PARAM_CHECK_BANDWIDTH,
                                    "PARAM_VALUE": self.PARAM_VALUE_40MHZ})
            self._assert_run_succeeded(sut)
        finally:
            self._delete_sniffer_log_file(self.FILE_NAME_40MHZ)

    def test_check_association_request_bandwidth_80mhz_ok(self):
        try:
            self._create_sniffer_log_file_80mhz()
            sut = self._create_sut({"SNIFFER_LOG_FILE": self.FILE_NAME_80MHZ,
                                    "FRAME_TO_CHECK": self.PARAM_FRAME_ASSOCIATION_REQUEST,
                                    "PARAM_TO_CHECK": self.PARAM_CHECK_BANDWIDTH,
                                    "PARAM_VALUE": self.PARAM_VALUE_80MHZ})
            self._assert_run_succeeded(sut)
        finally:
            self._delete_sniffer_log_file(self.FILE_NAME_80MHZ)

    # --- MANDATORY PARAMS FAILS --- #
    def test_check_beacon_capabilities_ht_fail(self):
        try:
            self._create_sniffer_log_file_null()
            sut = self._create_sut({"SNIFFER_LOG_FILE": self.FILE_NAME_NULL,
                                    "FRAME_TO_CHECK": self.PARAM_FRAME_BEACON,
                                    "PARAM_TO_CHECK": self.PARAM_CHECK_CAPABILITY,
                                    "PARAM_VALUE": self.PARAM_VALUE_HT_CAP})
            msg = "CHECK SNIFFER LOG FAIL - Unable to check parameters : %s %s %s" % (self.PARAM_FRAME_BEACON,
                                                                                      self.PARAM_CHECK_CAPABILITY,
                                                                                      self.PARAM_VALUE_HT_CAP)
            self._assert_run_throw_config_exception(sut, msg)
        finally:
            self._delete_sniffer_log_file(self.FILE_NAME_NULL)

    def test_check_beacon_capabilities_vht_fail(self):
        try:
            self._create_sniffer_log_file_20mhz()
            sut = self._create_sut({"SNIFFER_LOG_FILE": self.FILE_NAME_20MHZ,
                                    "FRAME_TO_CHECK": self.PARAM_FRAME_BEACON,
                                    "PARAM_TO_CHECK": self.PARAM_CHECK_CAPABILITY,
                                    "PARAM_VALUE": self.PARAM_VALUE_VHT_CAP})
            msg = "CHECK SNIFFER LOG FAIL - Unable to check parameters : %s %s %s" % (self.PARAM_FRAME_BEACON,
                                                                                      self.PARAM_CHECK_CAPABILITY,
                                                                                      self.PARAM_VALUE_VHT_CAP)
            self._assert_run_throw_config_exception(sut, msg)
        finally:
            self._delete_sniffer_log_file(self.FILE_NAME_20MHZ)

    def test_check_beacon_bandwidth_20mhz_fail(self):
        try:
            self._create_sniffer_log_file_40mhz()
            sut = self._create_sut({"SNIFFER_LOG_FILE": self.FILE_NAME_40MHZ,
                                    "FRAME_TO_CHECK": self.PARAM_FRAME_BEACON,
                                    "PARAM_TO_CHECK": self.PARAM_CHECK_BANDWIDTH,
                                    "PARAM_VALUE": self.PARAM_VALUE_20MHZ})
            msg = "CHECK SNIFFER LOG FAIL - Unable to check parameters : %s %s %s" % (self.PARAM_FRAME_BEACON,
                                                                                      self.PARAM_CHECK_BANDWIDTH,
                                                                                      self.PARAM_VALUE_20MHZ)
            self._assert_run_throw_config_exception(sut, msg)
        finally:
            self._delete_sniffer_log_file(self.FILE_NAME_40MHZ)

    def test_check_beacon_bandwidth_40mhz_fail(self):
        try:
            self._create_sniffer_log_file_20mhz()
            sut = self._create_sut({"SNIFFER_LOG_FILE": self.FILE_NAME_20MHZ,
                                    "FRAME_TO_CHECK": self.PARAM_FRAME_BEACON,
                                    "PARAM_TO_CHECK": self.PARAM_CHECK_BANDWIDTH,
                                    "PARAM_VALUE": self.PARAM_VALUE_40MHZ})
            msg = "CHECK SNIFFER LOG FAIL - Unable to check parameters : %s %s %s" % (self.PARAM_FRAME_BEACON,
                                                                                      self.PARAM_CHECK_BANDWIDTH,
                                                                                      self.PARAM_VALUE_40MHZ)
            self._assert_run_throw_config_exception(sut, msg)
        finally:
            self._delete_sniffer_log_file(self.FILE_NAME_20MHZ)

    def test_check_beacon_bandwidth_80mhz_fail(self):
        try:
            self._create_sniffer_log_file_20mhz()
            sut = self._create_sut({"SNIFFER_LOG_FILE": self.FILE_NAME_20MHZ,
                                    "FRAME_TO_CHECK": self.PARAM_FRAME_BEACON,
                                    "PARAM_TO_CHECK": self.PARAM_CHECK_BANDWIDTH,
                                    "PARAM_VALUE": self.PARAM_VALUE_80MHZ})
            msg = "CHECK SNIFFER LOG FAIL - Unable to check parameters : %s %s %s" % (self.PARAM_FRAME_BEACON,
                                                                                      self.PARAM_CHECK_BANDWIDTH,
                                                                                      self.PARAM_VALUE_80MHZ)
            self._assert_run_throw_config_exception(sut, msg)
        finally:
            self._delete_sniffer_log_file(self.FILE_NAME_20MHZ)

    def test_check_association_request_capabilities_ht_fail(self):
        try:
            self._create_sniffer_log_file_null()
            sut = self._create_sut({"SNIFFER_LOG_FILE": self.FILE_NAME_NULL,
                                    "FRAME_TO_CHECK": self.PARAM_FRAME_ASSOCIATION_REQUEST,
                                    "PARAM_TO_CHECK": self.PARAM_CHECK_CAPABILITY,
                                    "PARAM_VALUE": self.PARAM_VALUE_HT_CAP})
            msg = "CHECK SNIFFER LOG FAIL - Unable to check parameters : %s %s %s" % (
                self.PARAM_FRAME_ASSOCIATION_REQUEST,
                self.PARAM_CHECK_CAPABILITY,
                self.PARAM_VALUE_HT_CAP)
            self._assert_run_throw_config_exception(sut, msg)
        finally:
            self._delete_sniffer_log_file(self.FILE_NAME_NULL)

    def test_check_association_request_capabilities_vht_fail(self):
        try:
            self._create_sniffer_log_file_20mhz()
            sut = self._create_sut({"SNIFFER_LOG_FILE": self.FILE_NAME_20MHZ,
                                    "FRAME_TO_CHECK": self.PARAM_FRAME_ASSOCIATION_REQUEST,
                                    "PARAM_TO_CHECK": self.PARAM_CHECK_CAPABILITY,
                                    "PARAM_VALUE": self.PARAM_VALUE_VHT_CAP})
            msg = "CHECK SNIFFER LOG FAIL - Unable to check parameters : %s %s %s" % (
                self.PARAM_FRAME_ASSOCIATION_REQUEST,
                self.PARAM_CHECK_CAPABILITY,
                self.PARAM_VALUE_VHT_CAP)
            self._assert_run_throw_config_exception(sut, msg)
        finally:
            self._delete_sniffer_log_file(self.FILE_NAME_20MHZ)

    def test_check_association_request_bandwidth_20mhz_fail(self):
        try:
            self._create_sniffer_log_file_40mhz()
            sut = self._create_sut({"SNIFFER_LOG_FILE": self.FILE_NAME_40MHZ,
                                    "FRAME_TO_CHECK": self.PARAM_FRAME_ASSOCIATION_REQUEST,
                                    "PARAM_TO_CHECK": self.PARAM_CHECK_BANDWIDTH,
                                    "PARAM_VALUE": self.PARAM_VALUE_20MHZ})
            msg = "CHECK SNIFFER LOG FAIL - Unable to check parameters : %s %s %s" % (
                self.PARAM_FRAME_ASSOCIATION_REQUEST,
                self.PARAM_CHECK_BANDWIDTH,
                self.PARAM_VALUE_20MHZ)
            self._assert_run_throw_config_exception(sut, msg)
        finally:
            self._delete_sniffer_log_file(self.FILE_NAME_40MHZ)

    def test_check_association_request_bandwidth_40mhz_fail(self):
        try:
            self._create_sniffer_log_file_20mhz()
            sut = self._create_sut({"SNIFFER_LOG_FILE": self.FILE_NAME_20MHZ,
                                    "FRAME_TO_CHECK": self.PARAM_FRAME_ASSOCIATION_REQUEST,
                                    "PARAM_TO_CHECK": self.PARAM_CHECK_BANDWIDTH,
                                    "PARAM_VALUE": self.PARAM_VALUE_40MHZ})
            msg = "CHECK SNIFFER LOG FAIL - Unable to check parameters : %s %s %s" % (
                self.PARAM_FRAME_ASSOCIATION_REQUEST,
                self.PARAM_CHECK_BANDWIDTH,
                self.PARAM_VALUE_40MHZ)
            self._assert_run_throw_config_exception(sut, msg)
        finally:
            self._delete_sniffer_log_file(self.FILE_NAME_20MHZ)

    def test_check_association_request_bandwidth_80mhz_fail(self):
        try:
            self._create_sniffer_log_file_20mhz()
            sut = self._create_sut({"SNIFFER_LOG_FILE": self.FILE_NAME_20MHZ,
                                    "FRAME_TO_CHECK": self.PARAM_FRAME_ASSOCIATION_REQUEST,
                                    "PARAM_TO_CHECK": self.PARAM_CHECK_BANDWIDTH,
                                    "PARAM_VALUE": self.PARAM_VALUE_80MHZ})
            msg = "CHECK SNIFFER LOG FAIL - Unable to check parameters : %s %s %s" % (
                self.PARAM_FRAME_ASSOCIATION_REQUEST, self.PARAM_CHECK_BANDWIDTH, self.PARAM_VALUE_80MHZ)
            self._assert_run_throw_config_exception(sut, msg)
        finally:
            self._delete_sniffer_log_file(self.FILE_NAME_20MHZ)

    # --- OPTIONAL PARAMS --- #
    def test_check_mac_addr_source_and_receiver_ok(self):
        try:
            self._create_sniffer_log_file_80mhz()
            sut = self._create_sut({"SNIFFER_LOG_FILE": self.FILE_NAME_80MHZ,
                                    "FRAME_TO_CHECK": self.PARAM_FRAME_BEACON,
                                    "PARAM_TO_CHECK": self.PARAM_CHECK_CAPABILITY,
                                    "PARAM_VALUE": self.PARAM_VALUE_VHT_CAP,
                                    "MAC_ADDR_SOURCE": self.PARAM_MAC_SOURCE_OK,
                                    "MAC_ADDR_RECEIVER": self.PARAM_MAC_RECEIVER_OK})
            self._assert_run_succeeded(sut)
        finally:
            self._delete_sniffer_log_file(self.FILE_NAME_80MHZ)

    def test_check_mac_addr_source_and_receiver_fail(self):
        try:
            self._create_sniffer_log_file_80mhz()
            sut = self._create_sut({"SNIFFER_LOG_FILE": self.FILE_NAME_80MHZ,
                                    "FRAME_TO_CHECK": self.PARAM_FRAME_BEACON,
                                    "PARAM_TO_CHECK": self.PARAM_CHECK_CAPABILITY,
                                    "PARAM_VALUE": self.PARAM_VALUE_VHT_CAP,
                                    "MAC_ADDR_SOURCE": self.PARAM_MAC_SOURCE_FAIL,
                                    "MAC_ADDR_RECEIVER": self.PARAM_MAC_RECEIVER_OK})
            msg = "CHECK SNIFFER LOG FAIL - Unable to check parameters : "
            msg += "%s %s %s - with %s as source - with %s as receiver" % (
                self.PARAM_FRAME_BEACON, self.PARAM_CHECK_CAPABILITY, self.PARAM_VALUE_VHT_CAP,
                self.PARAM_MAC_SOURCE_FAIL, self.PARAM_MAC_RECEIVER_OK)
            self._assert_run_throw_config_exception(sut, msg)
        finally:
            self._delete_sniffer_log_file(self.FILE_NAME_80MHZ)

    def test_check_mac_addr_source_ok(self):
        try:
            self._create_sniffer_log_file_80mhz()
            sut = self._create_sut({"SNIFFER_LOG_FILE": self.FILE_NAME_80MHZ,
                                    "FRAME_TO_CHECK": self.PARAM_FRAME_BEACON,
                                    "PARAM_TO_CHECK": self.PARAM_CHECK_CAPABILITY,
                                    "PARAM_VALUE": self.PARAM_VALUE_VHT_CAP,
                                    "MAC_ADDR_SOURCE": self.PARAM_MAC_SOURCE_OK})
            self._assert_run_succeeded(sut)
        finally:
            self._delete_sniffer_log_file(self.FILE_NAME_80MHZ)

    def test_check_mac_addr_source_fail(self):
        try:
            self._create_sniffer_log_file_80mhz()
            sut = self._create_sut({"SNIFFER_LOG_FILE": self.FILE_NAME_80MHZ,
                                    "FRAME_TO_CHECK": self.PARAM_FRAME_BEACON,
                                    "PARAM_TO_CHECK": self.PARAM_CHECK_CAPABILITY,
                                    "PARAM_VALUE": self.PARAM_VALUE_VHT_CAP,
                                    "MAC_ADDR_SOURCE": self.PARAM_MAC_SOURCE_FAIL})
            msg = "CHECK SNIFFER LOG FAIL - Unable to check parameters : %s %s %s - with %s as source" % \
                  (self.PARAM_FRAME_BEACON, self.PARAM_CHECK_CAPABILITY, self.PARAM_VALUE_VHT_CAP,
                   self.PARAM_MAC_SOURCE_FAIL)
            self._assert_run_throw_config_exception(sut, msg)
        finally:
            self._delete_sniffer_log_file(self.FILE_NAME_80MHZ)

    def test_check_mac_addr_receiver_ok(self):
        try:
            self._create_sniffer_log_file_80mhz()
            sut = self._create_sut({"SNIFFER_LOG_FILE": self.FILE_NAME_80MHZ,
                                    "FRAME_TO_CHECK": self.PARAM_FRAME_BEACON,
                                    "PARAM_TO_CHECK": self.PARAM_CHECK_CAPABILITY,
                                    "PARAM_VALUE": self.PARAM_VALUE_VHT_CAP,
                                    "MAC_ADDR_RECEIVER": self.PARAM_MAC_RECEIVER_OK})
            self._assert_run_succeeded(sut)
        finally:
            self._delete_sniffer_log_file(self.FILE_NAME_80MHZ)

    def test_check_mac_addr_receiver_fail(self):
        try:
            self._create_sniffer_log_file_80mhz()
            sut = self._create_sut({"SNIFFER_LOG_FILE": self.FILE_NAME_80MHZ,
                                    "FRAME_TO_CHECK": self.PARAM_FRAME_BEACON,
                                    "PARAM_TO_CHECK": self.PARAM_CHECK_CAPABILITY,
                                    "PARAM_VALUE": self.PARAM_VALUE_VHT_CAP,
                                    "MAC_ADDR_RECEIVER": self.PARAM_MAC_RECEIVER_FAIL})
            msg = "CHECK SNIFFER LOG FAIL - Unable to check parameters : %s %s %s - with %s as receiver" % \
                  (self.PARAM_FRAME_BEACON, self.PARAM_CHECK_CAPABILITY, self.PARAM_VALUE_VHT_CAP,
                   self.PARAM_MAC_RECEIVER_FAIL)
            self._assert_run_throw_config_exception(sut, msg)
        finally:
            self._delete_sniffer_log_file(self.FILE_NAME_80MHZ)

    def _create_sut(self, args=None):
        self._sut = CheckSnifferLog(None, mock.Mock(), args, mock.Mock())
        return self._sut
