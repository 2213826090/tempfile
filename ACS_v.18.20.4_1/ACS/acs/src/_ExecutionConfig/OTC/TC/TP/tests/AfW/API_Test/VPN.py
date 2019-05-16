# -*- coding: utf-8 -*-

from testlib.AfW.api_impl import ApiImpl
from testlib.util.uiatestbase import UIATestBase
import time


class PPTP(UIATestBase):
    """
    @summary: VPN PPTP
    """

    def setUp(self):
        super(PPTP, self).setUp()
        self._test_name = __name__
        self.api = ApiImpl()
        self.api.unlock_screen()
        self.api.clean_tasks()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(PPTP, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testVPN_PPTP_Simple_testConnect(self):
        """
        verify pptp simple connection
        :return: None
        """
        self.pptp_server = self.config.read('tests.tablet.dut_init.conf', 'vpn').get("pptp_server")
        self.pptp_account = self.config.read('tests.tablet.dut_init.conf', 'vpn').get("pptp_account")
        self.pptp_password = self.config.read('tests.tablet.dut_init.conf', 'vpn').get("pptp_password")
        self.api.set_lock_pin()
        self.api.settings_sub_launch("More")
        self.api.click_with_timeout("text", "VPN")
        self.api.click_with_timeout("resourceId", "com.android.settings:id/vpn_create")
        assert not self.api.check_ui_exists("text", "Attention"), "lock screen doesn't set"
        self.api.click_with_timeout("resourceId", "com.android.settings:id/vpn_create")
        assert self.api.check_ui_exists("text", "Edit VPN profile"), "fail to detect vpn edit window"
        self.api.d(resourceId="com.android.settings:id/name").set_text("afwPPTP")
        self.api.d(resourceId="com.android.settings:id/server").set_text(self.pptp_server)
        self.api.click_with_timeout("resourceId", "com.android.settings:id/show_options")
        if not self.api.check_ui_exists("resourceId", "com.android.settings:id/dns_servers"):
            self.api.d(scrollable=True).scroll.vert.to(resourceId="com.android.settings:id/dns_servers")
        self.api.d(resourceId="com.android.settings:id/dns_servers").set_text("8.8.8.8")
        self.api.click_with_timeout("text", "Save")
        self.api.click_with_timeout("text", "afwPPTP")
        self.api.d(resourceId="com.android.settings:id/username").set_text(self.pptp_account)
        self.api.d(resourceId="com.android.settings:id/password").set_text(self.pptp_password)
        self.api.click_with_timeout("resourceId", "com.android.settings:id/save_login")
        self.api.click_with_timeout("text", "Connect")
        for _ in range(10):
            if self.api.check_ui_exists("text", "Disconnect"):
                self.api.d.press.back()
                break
            if self.api.check_ui_exists("text", "Connected"):
                break
            self.api.click_with_timeout("text", "afwPPTP")
            self.api.click_with_timeout("text", "Connect")
            time.sleep(5)
        assert self.api.check_ui_exists("text", "Connected"), "fail to connect VPN via PPTP"

    def testVPN_Simple_CreateVPN_PO(self):
        """
        verify pptp simple connection in PO
        :return: None
        """
        self.pptp_server = self.config.read('tests.tablet.dut_init.conf', 'vpn').get("pptp_server")
        self.pptp_account = self.config.read('tests.tablet.dut_init.conf', 'vpn').get("pptp_account")
        self.pptp_password = self.config.read('tests.tablet.dut_init.conf', 'vpn').get("pptp_password")
        self.api.set_lock_pin()
        self.api.settings_sub_launch("More")
        self.api.click_with_timeout("text", "VPN")
        self.api.click_with_timeout("resourceId", "com.android.settings:id/vpn_create")
        assert not self.api.check_ui_exists("text", "Attention"), "lock screen doesn't set in PO"
        self.api.click_with_timeout("resourceId", "com.android.settings:id/vpn_create")
        assert self.api.check_ui_exists("text", "Edit VPN profile"), "fail to detect vpn edit window in PO"
        self.api.d(resourceId="com.android.settings:id/name").set_text("afwPPTP")
        self.api.d(resourceId="com.android.settings:id/server").set_text(self.pptp_server)
        self.api.click_with_timeout("resourceId", "com.android.settings:id/show_options")
        if not self.api.check_ui_exists("resourceId", "com.android.settings:id/dns_servers"):
            self.api.d(scrollable=True).scroll.vert.to(resourceId="com.android.settings:id/dns_servers")
        self.api.d(resourceId="com.android.settings:id/dns_servers").set_text("8.8.8.8")
        self.api.click_with_timeout("text", "Save")
        self.api.click_with_timeout("text", "afwPPTP")
        self.api.d(resourceId="com.android.settings:id/username").set_text(self.pptp_account)
        self.api.d(resourceId="com.android.settings:id/password").set_text(self.pptp_password)
        self.api.click_with_timeout("resourceId", "com.android.settings:id/save_login")
        self.api.click_with_timeout("text", "Connect")
        for _ in range(10):
            if self.api.check_ui_exists("text", "Disconnect"):
                self.api.d.press.back()
                break
            if self.api.check_ui_exists("text", "Connected"):
                break
            self.api.click_with_timeout("text", "afwPPTP")
            self.api.click_with_timeout("text", "Connect")
            time.sleep(5)
        assert self.api.check_ui_exists("text", "Connected"), "fail to connect VPN via PPTP in PO"

    def testVPN_PPTP_Simple_testCheckConnection(self):
        self.api.settings_sub_launch("More")
        self.api.click_with_timeout("text", "VPN")
        for _ in range(5):
            if self.api.check_ui_exists("text", "Connected"):
                break
            self.api.click_with_timeout("text", "afwPPTP")
            self.api.click_with_timeout("text", "Connect")
            time.sleep(5)
        assert self.api.check_ui_exists("text", "afwPPTP"), "there isn't PPTP connection exist"
        assert self.api.check_ui_exists("text", "Connected"), "fail to connect PPTP VPN"

    def testVPN_PPTP_Simple_ConnectVPN_PO(self):
        self.api.settings_sub_launch("More")
        self.api.click_with_timeout("text", "VPN")
        for _ in range(5):
            if self.api.check_ui_exists("text", "Connected"):
                break
            self.api.click_with_timeout("text", "afwPPTP")
            self.api.click_with_timeout("text", "Connect")
            time.sleep(5)
        assert self.api.check_ui_exists("text", "afwPPTP"), "there isn't PPTP connection exist"
        assert self.api.check_ui_exists("text", "Connected"), "fail to connect PPTP VPN"

    def testVPN_PPTP_Simple_testDisconnect(self):
        try:
            self.api.settings_sub_launch("More")
            self.api.click_with_timeout("text", "VPN")
            assert self.api.check_ui_exists("text", "afwPPTP"), "there isn't PPTP connection exist"
            if not self.api.check_ui_exists("text", "Connected"):
                for _ in range(10):
                    if self.api.check_ui_exists("text", "Disconnect"):
                        self.api.d.press.back()
                        break
                    if self.api.check_ui_exists("text", "Connected"):
                        break
                    self.api.click_with_timeout("text", "afwPPTP")
                    self.api.click_with_timeout("text", "Connect")
                    time.sleep(5)
            assert self.api.check_ui_exists("text", "Connected"), "vpn wasn't connected"
            self.api.click_with_timeout("text", "afwPPTP")
            self.api.click_with_timeout("text", "Disconnect")
            assert not self.api.check_ui_exists("text", "Connected"), "fail to disconnect pptp"
        finally:
            self.api.clean_tasks()
            self.api.set_lock_swipe()

    def testVPN_PPTP_Simple_DisconnectVPN_PO(self):
        try:
            self.api.settings_sub_launch("More")
            self.api.click_with_timeout("text", "VPN")
            assert self.api.check_ui_exists("text", "afwPPTP"), "there isn't PPTP connection exist in PO"
            if not self.api.check_ui_exists("text", "Connected"):
                for _ in range(10):
                    if self.api.check_ui_exists("text", "Disconnect"):
                        self.api.d.press.back()
                        break
                    if self.api.check_ui_exists("text", "Connected"):
                        break
                    self.api.click_with_timeout("text", "afwPPTP")
                    self.api.click_with_timeout("text", "Connect")
                    time.sleep(5)
            assert self.api.check_ui_exists("text", "Connected"), "vpn wasn't connected in PO"
            self.api.click_with_timeout("text", "afwPPTP")
            self.api.click_with_timeout("text", "Disconnect")
            assert not self.api.check_ui_exists("text", "Connected"), "fail to disconnect pptp in PO"
        finally:
            self.api.clean_tasks()
            self.api.set_lock_swipe()


class L2TP(UIATestBase):
    """
    @summary: VPN L2TP
    """

    def setUp(self):
        super(L2TP, self).setUp()
        self._test_name = __name__
        self.api = ApiImpl()
        self.api.unlock_screen()
        self.api.clean_tasks()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(L2TP, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testVPN_L2TP_Simple_testConnect(self):
        """
        verify l2tp simple connection
        :return: None
        """
        self.l2tp_server = self.config.read('tests.tablet.dut_init.conf', 'vpn').get("l2tp_server")
        self.l2tp_key = self.config.read('tests.tablet.dut_init.conf', 'vpn').get("l2tp_key")
        self.l2tp_account = self.config.read('tests.tablet.dut_init.conf', 'vpn').get("l2tp_account")
        self.l2tp_password = self.config.read('tests.tablet.dut_init.conf', 'vpn').get("l2tp_password")
        self.api.set_lock_pin()
        self.api.settings_sub_launch("More")
        self.api.click_with_timeout("text", "VPN")
        self.api.click_with_timeout("resourceId", "com.android.settings:id/vpn_create")
        assert not self.api.check_ui_exists("text", "Attention"), "lock screen doesn't set"
        self.api.click_with_timeout("resourceId", "com.android.settings:id/vpn_create")
        assert self.api.check_ui_exists("text", "Edit VPN profile"), "fail to detect vpn edit window"
        self.api.d(resourceId="com.android.settings:id/name").set_text("afwL2TP")
        self.api.click_with_timeout("resourceId", "com.android.settings:id/type")
        self.api.click_with_timeout("text", "L2TP/IPSec PSK")
        self.api.d(resourceId="com.android.settings:id/server").set_text(self.l2tp_server)
        self.api.d(resourceId="com.android.settings:id/ipsec_secret").set_text(self.l2tp_key)
        self.api.click_with_timeout("resourceId", "com.android.settings:id/show_options")
        if not self.api.check_ui_exists("resourceId", "com.android.settings:id/dns_servers"):
            self.api.d(scrollable=True).scroll.vert.to(resourceId="com.android.settings:id/dns_servers")
        self.api.d(resourceId="com.android.settings:id/dns_servers").set_text("8.8.8.8")
        self.api.click_with_timeout("text", "Save")
        self.api.click_with_timeout("text", "afwL2TP")
        self.api.d(resourceId="com.android.settings:id/username").set_text(self.l2tp_account)
        self.api.d(resourceId="com.android.settings:id/password").set_text(self.l2tp_password)
        self.api.click_with_timeout("resourceId", "com.android.settings:id/save_login")
        self.api.click_with_timeout("text", "Connect")
        for _ in range(10):
            if self.api.check_ui_exists("text", "Disconnect"):
                self.api.d.press.back()
                break
            if self.api.check_ui_exists("text", "Connected"):
                break
            self.api.click_with_timeout("text", "afwL2TP")
            self.api.click_with_timeout("text", "Connect")
            time.sleep(5)
        assert self.api.check_ui_exists("text", "Connected"), "fail to connect VPN via L2TP"

    def testVPN_Simple_CreateVPN_PO(self):
        """
        verify l2tp simple connection
        :return: None
        """
        self.l2tp_server = self.config.read('tests.tablet.dut_init.conf', 'vpn').get("l2tp_server")
        self.l2tp_key = self.config.read('tests.tablet.dut_init.conf', 'vpn').get("l2tp_key")
        self.l2tp_account = self.config.read('tests.tablet.dut_init.conf', 'vpn').get("l2tp_account")
        self.l2tp_password = self.config.read('tests.tablet.dut_init.conf', 'vpn').get("l2tp_password")
        self.api.set_lock_pin()
        self.api.settings_sub_launch("More")
        self.api.click_with_timeout("text", "VPN")
        self.api.click_with_timeout("resourceId", "com.android.settings:id/vpn_create")
        assert not self.api.check_ui_exists("text", "Attention"), "lock screen doesn't set in PO"
        self.api.click_with_timeout("resourceId", "com.android.settings:id/vpn_create")
        assert self.api.check_ui_exists("text", "Edit VPN profile"), "fail to detect vpn edit window in PO"
        self.api.d(resourceId="com.android.settings:id/name").set_text("afwL2TP")
        self.api.click_with_timeout("resourceId", "com.android.settings:id/type")
        self.api.click_with_timeout("text", "L2TP/IPSec PSK")
        self.api.d(resourceId="com.android.settings:id/server").set_text(self.l2tp_server)
        self.api.d(resourceId="com.android.settings:id/ipsec_secret").set_text(self.l2tp_key)
        self.api.click_with_timeout("resourceId", "com.android.settings:id/show_options")
        if not self.api.check_ui_exists("resourceId", "com.android.settings:id/dns_servers"):
            self.api.d(scrollable=True).scroll.vert.to(resourceId="com.android.settings:id/dns_servers")
        self.api.d(resourceId="com.android.settings:id/dns_servers").set_text("8.8.8.8")
        self.api.click_with_timeout("text", "Save")
        self.api.click_with_timeout("text", "afwL2TP")
        self.api.d(resourceId="com.android.settings:id/username").set_text(self.l2tp_account)
        self.api.d(resourceId="com.android.settings:id/password").set_text(self.l2tp_password)
        self.api.click_with_timeout("resourceId", "com.android.settings:id/save_login")
        self.api.click_with_timeout("text", "Connect")
        for _ in range(10):
            if self.api.check_ui_exists("text", "Disconnect"):
                self.api.d.press.back()
                break
            if self.api.check_ui_exists("text", "Connected"):
                break
            self.api.click_with_timeout("text", "afwL2TP")
            self.api.click_with_timeout("text", "Connect")
            time.sleep(5)
        assert self.api.check_ui_exists("text", "Connected"), "fail to connect VPN via L2TP in PO"

    def testVPN_L2TP_Simple_testCheckConnection(self):
        self.api.settings_sub_launch("More")
        self.api.click_with_timeout("text", "VPN")
        for _ in range(5):
            if self.api.check_ui_exists("text", "Connected"):
                break
            self.api.click_with_timeout("text", "afwL2TP")
            self.api.click_with_timeout("text", "Connect")
            time.sleep(5)
        assert self.api.check_ui_exists("text", "afwL2TP"), "there isn't L2TP connection exist"
        assert self.api.check_ui_exists("text", "Connected"), "fail to connect L2TP VPN"

    def testVPN_L2TP_Simple_ConnectVPN_PO(self):
        self.api.settings_sub_launch("More")
        self.api.click_with_timeout("text", "VPN")
        for _ in range(5):
            if self.api.check_ui_exists("text", "Connected"):
                break
            self.api.click_with_timeout("text", "afwL2TP")
            self.api.click_with_timeout("text", "Connect")
            time.sleep(5)
        assert self.api.check_ui_exists("text", "afwL2TP"), "there isn't L2TP connection exist"
        assert self.api.check_ui_exists("text", "Connected"), "fail to connect L2TP VPN"

    def testVPN_L2TP_Simple_testDisconnect(self):
        try:
            self.api.settings_sub_launch("More")
            self.api.click_with_timeout("text", "VPN")
            assert self.api.check_ui_exists("text", "afwL2TP"), "there isn't L2TP connection exist"
            if not self.api.check_ui_exists("text", "Connected"):
                for _ in range(10):
                    if self.api.check_ui_exists("text", "Disconnect"):
                        self.api.d.press.back()
                        break
                    if self.api.check_ui_exists("text", "Connected"):
                        break
                    self.api.click_with_timeout("text", "afwL2TP")
                    self.api.click_with_timeout("text", "Connect")
                    time.sleep(5)
            assert self.api.check_ui_exists("text", "Connected"), "vpn wasn't connected"
            self.api.click_with_timeout("text", "afwL2TP")
            self.api.click_with_timeout("text", "Disconnect")
            assert not self.api.check_ui_exists("text", "Connected"), "fail to disconnect L2TP"
        finally:
            self.api.clean_tasks()
            self.api.set_lock_swipe()

    def testVPN_L2TP_Simple_DisconnectVPN_PO(self):
        try:
            self.api.settings_sub_launch("More")
            self.api.click_with_timeout("text", "VPN")
            assert self.api.check_ui_exists("text", "afwL2TP"), "there isn't L2TP connection exist in PO"
            if not self.api.check_ui_exists("text", "Connected"):
                for _ in range(10):
                    if self.api.check_ui_exists("text", "Disconnect"):
                        self.api.d.press.back()
                        break
                    if self.api.check_ui_exists("text", "Connected"):
                        break
                    self.api.click_with_timeout("text", "afwL2TP")
                    self.api.click_with_timeout("text", "Connect")
                    time.sleep(5)
            assert self.api.check_ui_exists("text", "Connected"), "vpn wasn't connected in PO"
            self.api.click_with_timeout("text", "afwL2TP")
            self.api.click_with_timeout("text", "Disconnect")
            assert not self.api.check_ui_exists("text", "Connected"), "fail to disconnect L2TP in PO"
        finally:
            self.api.clean_tasks()
            self.api.set_lock_swipe()
