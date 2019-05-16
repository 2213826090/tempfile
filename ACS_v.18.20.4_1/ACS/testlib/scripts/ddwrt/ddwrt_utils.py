#!/usr/bin/env python

#######################################################################
#
# @filename:    utils.py
# @description: Utils for DDWRT
# @author:      aurel.constantin@intel.com
#
#######################################################################
from testlib.utils.defaults import wifi_defaults
from testlib.scripts.connections.local import local_utils
from testlib.utils.connections.local import Local as connection_local

class APConfigException(Exception):
    """Error for ATF integration issues"""
    pass


def generate_ap_config(mode, security, encryption=None, wifi_password=None,
                       radius_ip=None, radius_secret=None, hidden_ssid="0",
                       new_ssid=None, channel_bw="20", channel_no=None, interface5ghz="0",
                       ipv6_enable=0, radvd_enable=0, dhcp_lease=None, txpwr=None, ipv4_class=None):


    """ description:
            sets a new configuration on the ddwrt router
                - <mode> = the operation mode of the AP
                - <security> = the security mode of the AP
                Valid values for "mode" = "b",
                                           "bg",
                                           "g",
                                           "mixed",
                                           "n",
                                           "ng"
                             for "security" = "none",
                                               "wpa_psk_mixed",
                                               "wpa_psk",
                                               "wpa2",
                                               "wpa_enterprise",
                                               "wpa2_enterprise",
                                               "wep64",
                                               "wep128"
        usage:
            steps.configure_ap(mode = "n",
                          security = "wpa2",
                          encryption = "tkip",
                          wifi_password = "PassPhrase",
                          ssh_host = "192.168.1.1",
                          ssh_user = "root")()
    """

    cmd_list = []
    if interface5ghz == None:
        interface5ghz = "0"
    if channel_bw == None:
        channel_bw = "20"
    elif channel_bw == "2040":
        channel_bw = "0"
    if channel_bw == "20":
        # default channels for 20MHz channel bandwidths -> for best compliance with different DUT regulatory domains
        if channel_no == None and interface5ghz == "0":
            channel_no = wifi_defaults.wifi['sta_channel']
        elif channel_no == None and interface5ghz == "1":
            channel_no = wifi_defaults.wifi['sta_channel_5ghz']

    # NETWORK MODE
    if interface5ghz == '0': # 2.4 GHz
        if mode == "b":
            cmd_list.extend(["nvram set wl"+interface5ghz+"_afterburner=off",
                             "nvram set wl"+interface5ghz+"_gmode=0",
                             "nvram set wl"+interface5ghz+"_bss_opmode_cap_reqd=0",
                             "nvram set wl"+interface5ghz+"_net_mode=b-only",
                             "nvram set wl"+interface5ghz+"_nmode=0",
                             "nvram set wl"+interface5ghz+"_nreqd=0",
                             "nvram set wl"+interface5ghz+"_phytype=n"])
            if channel_bw in ['5', '10', '20']:
                cmd_list.extend(["nvram set wl"+interface5ghz+"_nctrlsb=",
                                "nvram set wl"+interface5ghz+"_nbw="+channel_bw])
                if channel_bw == "20":
                    cmd_list.extend(["nvram set wl"+interface5ghz+"_channel="+channel_no])
            else:
                raise APConfigException("Invalid channel bandwidth value")

            if txpwr != None:
                if int(txpwr) in range(45, 101):
                    print "nvram set wl"+interface5ghz+"_txpwr="+txpwr
                    cmd_list.extend(["nvram set wl"+interface5ghz+"_txpwr="+txpwr])
                else:
                    raise APConfigException("Invalid tranmission power value")

            if ipv4_class != None:
                if str(ipv4_class):
                    cmd_list.extend(["nvram set lan_ipaddr="+ipv4_class])
                else:
                    raise APConfigException("Invalid ipv4 class address")


        elif mode == "g":
            cmd_list.extend(["nvram set wl"+interface5ghz+"_afterburner=off",
                             "nvram set wl"+interface5ghz+"_gmode=2",
                             "nvram set wl"+interface5ghz+"_bss_opmode_cap_reqd=1",
                             "nvram set wl"+interface5ghz+"_net_mode=g-only",
                             "nvram set wl"+interface5ghz+"_nmode=0",
                             "nvram set wl"+interface5ghz+"_nreqd=0",
                             "nvram set wl"+interface5ghz+"_phytype=n"])
            if channel_bw in ['5', '10', '20']:
                cmd_list.extend(["nvram set wl"+interface5ghz+"_nctrlsb=",
                                "nvram set wl"+interface5ghz+"_nbw="+channel_bw])
                if channel_bw == "20":
                    cmd_list.extend(["nvram set wl"+interface5ghz+"_channel="+channel_no])
            else:
                raise APConfigException("Invalid channel bandwidth value")

            if txpwr != None:
                if txpwr in range(45, 101):
                    print "nvram set wl" + interface5ghz + "_txpwr=" + txpwr
                    cmd_list.extend(["nvram set wl" + interface5ghz + "_txpwr=" + txpwr])
                else:
                    raise APConfigException("Invalid tranmission power value")

            if ipv4_class != None:
                if str(ipv4_class):
                    cmd_list.extend(["nvram set lan_ipaddr="+ipv4_class])
                else:
                    raise APConfigException("Invalid ipv4 class address")


        elif mode == "bg":
            cmd_list.extend(["nvram set wl"+interface5ghz+"_afterburner=auto",
                             "nvram set wl"+interface5ghz+"_gmode=1",
                             "nvram set wl"+interface5ghz+"_bss_opmode_cap_reqd=0",
                             "nvram set wl"+interface5ghz+"_net_mode=bg-mixed",
                             "nvram set wl"+interface5ghz+"_nmode=0",
                             "nvram set wl"+interface5ghz+"_nreqd=0",
                             "nvram set wl"+interface5ghz+"_phytype=n"])
            if channel_bw in ['5', '10', '20']:
                cmd_list.extend(["nvram set wl"+interface5ghz+"_nctrlsb=",
                                "nvram set wl"+interface5ghz+"_nbw="+channel_bw])
                if channel_bw == "20":
                    cmd_list.extend(["nvram set wl"+interface5ghz+"_channel="+channel_no])
            else:
                raise APConfigException("Invalid channel bandwidth value")

            if txpwr != None:
                if int(txpwr) in range(45, 101):
                    print "nvram set wl"+interface5ghz+"_txpwr="+txpwr
                    cmd_list.extend(["nvram set wl"+interface5ghz+"_txpwr="+txpwr])
                else:
                    raise APConfigException("Invalid tranmission power value")

            if ipv4_class != None:
                if str(ipv4_class):
                    cmd_list.extend(["nvram set lan_ipaddr="+ipv4_class])
                else:
                    raise APConfigException("Invalid ipv4 class address")

        elif mode == "n":
            cmd_list.extend(["nvram set wl"+interface5ghz+"_afterburner=off",
                             "nvram set wl"+interface5ghz+"_gmode=1",
                             "nvram set wl"+interface5ghz+"_bss_opmode_cap_reqd=2",
                             "nvram set wl"+interface5ghz+"_net_mode=n-only",
                             "nvram set wl"+interface5ghz+"_nmode=2",
                             "nvram set wl"+interface5ghz+"_nreqd=1",
                             "nvram set wl"+interface5ghz+"_phytype=n"])
            if channel_bw in ['5', '10', '20']:
                cmd_list.extend(["nvram set wl"+interface5ghz+"_nctrlsb=none",
                                "nvram set wl"+interface5ghz+"_nbw="+channel_bw])
                if channel_bw == "20":
                    cmd_list.extend(["nvram set wl"+interface5ghz+"_channel="+channel_no])
            elif channel_bw in ['0', '40']:
                cmd_list.extend(["nvram set wl"+interface5ghz+"_nctrlsb=lower",
                                "nvram set wl"+interface5ghz+"_nbw="+channel_bw])
            else:
                raise APConfigException("Invalid channel bandwidth value")
            if txpwr != None:
                if int(txpwr) in range(45, 101):
                    print "nvram set wl"+interface5ghz+"_txpwr="+txpwr
                    cmd_list.extend(["nvram set wl"+interface5ghz+"_txpwr="+txpwr])
                else:
                    raise APConfigException("Invalid tranmission power value")

            if ipv4_class != None:
                if str(ipv4_class):
                    cmd_list.extend(["nvram set lan_ipaddr="+ipv4_class])
                else:
                    raise APConfigException("Invalid ipv4 class address")


        elif mode == "ng":
            cmd_list.extend(["nvram set wl"+interface5ghz+"_afterburner=off",
                             "nvram set wl"+interface5ghz+"_gmode=2",
                             "nvram set wl"+interface5ghz+"_bss_opmode_cap_reqd=1",
                             "nvram set wl"+interface5ghz+"_net_mode=ng-only",
                             "nvram set wl"+interface5ghz+"_nmode=2",
                             "nvram set wl"+interface5ghz+"_nreqd=0",
                             "nvram set wl"+interface5ghz+"_phytype=n"])
            if channel_bw in ['5', '10', '20']:
                cmd_list.extend(["nvram set wl"+interface5ghz+"_nctrlsb=none",
                                "nvram set wl"+interface5ghz+"_nbw="+channel_bw])
                if channel_bw == "20":
                    cmd_list.extend(["nvram set wl"+interface5ghz+"_channel="+channel_no])
            elif channel_bw in ['0', '40']:
                cmd_list.extend(["nvram set wl"+interface5ghz+"_nctrlsb=lower",
                                "nvram set wl"+interface5ghz+"_nbw="+channel_bw])
            else:
                raise APConfigException("Invalid channel bandwidth value")
            if txpwr != None:
                if int(txpwr) in range(45, 101):
                    print "nvram set wl"+interface5ghz+"_txpwr="+txpwr
                    cmd_list.extend(["nvram set wl"+interface5ghz+"_txpwr="+txpwr])
                else:
                    raise APConfigException("Invalid tranmission power value")

            if ipv4_class != None:
                if str(ipv4_class):
                    cmd_list.extend(["nvram set lan_ipaddr="+ipv4_class])
                else:
                    raise APConfigException("Invalid ipv4 class address")

        elif mode == "mixed":
            cmd_list.extend(["nvram set wl"+interface5ghz+"_afterburner=auto",
                             "nvram set wl"+interface5ghz+"_gmode=1",
                             "nvram set wl"+interface5ghz+"_bss_opmode_cap_reqd=0",
                             "nvram set wl"+interface5ghz+"_net_mode=mixed",
                             "nvram set wl"+interface5ghz+"_nmode=-1",
                             "nvram set wl"+interface5ghz+"_nreqd=0",
                             "nvram set wl"+interface5ghz+"_phytype=n"])
            if channel_bw in ['5', '10', '20']:
                cmd_list.extend(["nvram set wl"+interface5ghz+"_nctrlsb=none",
                                "nvram set wl"+interface5ghz+"_nbw="+channel_bw])
                if channel_bw == "20":
                    cmd_list.extend(["nvram set wl"+interface5ghz+"_channel="+channel_no])
            elif channel_bw in ['0', '40']:
                cmd_list.extend(["nvram set wl"+interface5ghz+"_nctrlsb=lower",
                                "nvram set wl"+interface5ghz+"_nbw="+channel_bw])
            else:
                raise APConfigException("Invalid channel bandwidth value")

            if txpwr != None:
                if int(txpwr) in range(45, 101):
                    print "nvram set wl"+interface5ghz+"_txpwr="+txpwr
                    cmd_list.extend(["nvram set wl"+interface5ghz+"_txpwr="+txpwr])
                else:
                    raise APConfigException("Invalid tranmission power value")

            if ipv4_class != None:
                if str(ipv4_class):
                    cmd_list.extend(["nvram set lan_ipaddr="+ipv4_class])
                else:
                    raise APConfigException("Invalid ipv4 class address")

        elif mode == "disabled":
            cmd_list.extend(["nvram set wl"+interface5ghz+"_afterburner=auto",
                             "nvram set wl"+interface5ghz+"_gmode=-1",
                             "nvram set wl"+interface5ghz+"_bss_opmode_cap_reqd=0",
                             "nvram set wl"+interface5ghz+"_net_mode=disabled",
                             "nvram set wl"+interface5ghz+"_nmode=-1",
                             "nvram set wl"+interface5ghz+"_nreqd=0",
                             "nvram set wl"+interface5ghz+"_phytype=n"])

        else:
            raise APConfigException("Invalid mode value")


    elif interface5ghz == '1': # 5 GHz
        #TODO: test on routers without AC

        if mode == "a":
            cmd_list.extend(["nvram set wl"+interface5ghz+"_afterburner=auto",
                             "nvram set wl"+interface5ghz+"_bss_opmode_cap_reqd=1",
                             "nvram set wl"+interface5ghz+"_net_mode=a-only",
                             "nvram set wl"+interface5ghz+"_nmode=0",
                             "nvram set wl"+interface5ghz+"_nreqd=0",
                             "nvram set wl"+interface5ghz+"_gmode=1",
                             "nvram set wl"+interface5ghz+"_phytype=v"])
            if channel_bw in ['5', '10', '20']:
                cmd_list.extend(["nvram set wl"+interface5ghz+"_nctrlsb=none",
                                "nvram set wl"+interface5ghz+"_nbw="+channel_bw])
                if channel_bw == "20":
                    cmd_list.extend(["nvram set wl"+interface5ghz+"_channel="+channel_no])
            else:
                raise APConfigException("Invalid channel bandwidth value")

            if txpwr != None:
                if int(txpwr) in range(45, 101):
                    print "nvram set wl"+interface5ghz+"_txpwr="+txpwr
                    cmd_list.extend(["nvram set wl"+interface5ghz+"_txpwr="+txpwr])
                else:
                    raise APConfigException("Invalid tranmission power value")

            if ipv4_class != None:
                if str(ipv4_class):
                    cmd_list.extend(["nvram set lan_ipaddr="+ipv4_class])
                else:
                    raise APConfigException("Invalid ipv4 class address")

        elif mode == "n":
            cmd_list.extend(["nvram set wl"+interface5ghz+"_afterburner=off",
                             "nvram set wl"+interface5ghz+"_bss_opmode_cap_reqd=2",
                             "nvram set wl"+interface5ghz+"_net_mode=n5-only",
                             "nvram set wl"+interface5ghz+"_nmode=2",
                             "nvram set wl"+interface5ghz+"_nreqd=1",
                             "nvram set wl"+interface5ghz+"_gmode=1",
                             "nvram set wl"+interface5ghz+"_phytype=v"])
            if channel_bw in ['5', '10', '20']:
                cmd_list.extend(["nvram set wl"+interface5ghz+"_nctrlsb=none",
                                "nvram set wl"+interface5ghz+"_nbw="+channel_bw])
                if channel_bw == "20":
                    cmd_list.extend(["nvram set wl"+interface5ghz+"_channel="+channel_no])
            elif channel_bw in ['0', '40']:
                cmd_list.extend(["nvram set wl"+interface5ghz+"_nctrlsb=upper",
                                "nvram set wl"+interface5ghz+"_nbw="+channel_bw])
            else:
                raise APConfigException("Invalid channel bandwidth value")

            if txpwr != None:
                if int(txpwr) in range(45, 101):
                    print "nvram set wl"+interface5ghz+"_txpwr="+txpwr
                    cmd_list.extend(["nvram set wl"+interface5ghz+"_txpwr="+txpwr])
                else:
                    raise APConfigException("Invalid tranmission power value")

            if ipv4_class != None:
                if str(ipv4_class):
                    cmd_list.extend(["nvram set lan_ipaddr="+ipv4_class])
                else:
                    raise APConfigException("Invalid ipv4 class address")

        elif mode == "na":
            cmd_list.extend(["nvram set wl"+interface5ghz+"_afterburner=off",
                             "nvram set wl"+interface5ghz+"_bss_opmode_cap_reqd=1",
                             "nvram set wl"+interface5ghz+"_net_mode=na-only",
                             "nvram set wl"+interface5ghz+"_nmode=2",
                             "nvram set wl"+interface5ghz+"_nreqd=0",
                             "nvram set wl"+interface5ghz+"_gmode=1",
                             "nvram set wl"+interface5ghz+"_phytype=v"])
            if channel_bw in ['5', '10', '20']:
                cmd_list.extend(["nvram set wl"+interface5ghz+"_nctrlsb=none",
                                "nvram set wl"+interface5ghz+"_nbw="+channel_bw])
                if channel_bw == "20":
                    cmd_list.extend(["nvram set wl"+interface5ghz+"_channel="+channel_no])
            elif channel_bw in ['0', '40']:
                cmd_list.extend(["nvram set wl"+interface5ghz+"_nctrlsb=upper",
                                "nvram set wl"+interface5ghz+"_nbw="+channel_bw])
            else:
                raise APConfigException("Invalid channel bandwidth value")

            if txpwr != None:
                if int(txpwr) in range(45, 101):
                    print "nvram set wl"+interface5ghz+"_txpwr="+txpwr
                    cmd_list.extend(["nvram set wl"+interface5ghz+"_txpwr="+txpwr])
                else:
                    raise APConfigException("Invalid tranmission power value")

            if ipv4_class != None:
                if str(ipv4_class):
                    cmd_list.extend(["nvram set lan_ipaddr="+ipv4_class])
                else:
                    raise APConfigException("Invalid ipv4 class address")

        elif mode == "mixed":
            cmd_list.extend(["nvram set wl"+interface5ghz+"_afterburner=auto",
                             "nvram set wl"+interface5ghz+"_bss_opmode_cap_reqd=0",
                             "nvram set wl"+interface5ghz+"_net_mode=mixed",
                             "nvram set wl"+interface5ghz+"_nmode=-1",
                             "nvram set wl"+interface5ghz+"_nreqd=0",
                             "nvram set wl"+interface5ghz+"_gmode=1",
                             "nvram set wl"+interface5ghz+"_phytype=v"])
            if channel_bw in ['5', '10', '20']:
                cmd_list.extend(["nvram set wl"+interface5ghz+"_nctrlsb=none",
                                "nvram set wl"+interface5ghz+"_nbw="+channel_bw])
                if channel_bw == "20":
                    cmd_list.extend(["nvram set wl"+interface5ghz+"_channel="+channel_no])
            elif channel_bw in ['0', '40']:
                cmd_list.extend(["nvram set wl"+interface5ghz+"_nctrlsb=upper",
                                "nvram set wl"+interface5ghz+"_nbw="+channel_bw])
            elif channel_bw == '80':
                cmd_list.extend(["nvram set wl"+interface5ghz+"_nctrlsb=uu",
                                "nvram set wl"+interface5ghz+"_nbw="+channel_bw])
            else:
                raise APConfigException("Invalid channel bandwidth value")

            if txpwr != None:
                if int(txpwr) in range(45, 101):
                    print "nvram set wl"+interface5ghz+"_txpwr="+txpwr
                    cmd_list.extend(["nvram set wl"+interface5ghz+"_txpwr="+txpwr])
                else:
                    raise APConfigException("Invalid tranmission power value")

            if ipv4_class != None:
                if str(ipv4_class):
                    cmd_list.extend(["nvram set lan_ipaddr="+ipv4_class])
                else:
                    raise APConfigException("Invalid ipv4 class address")

        elif mode == "ac":
            cmd_list.extend(["nvram set wl"+interface5ghz+"_afterburner=auto",
                             "nvram set wl"+interface5ghz+"_bss_opmode_cap_reqd=3",
                             "nvram set wl"+interface5ghz+"_net_mode=ac-only",
                             "nvram set wl"+interface5ghz+"_nmode=2",
                             "nvram set wl"+interface5ghz+"_nreqd=1",
                             "nvram set wl"+interface5ghz+"_gmode=1",
                             "nvram set wl"+interface5ghz+"_phytype=v"])
            if channel_bw in ['5', '10', '20']:
                cmd_list.extend(["nvram set wl"+interface5ghz+"_nctrlsb=none",
                                "nvram set wl"+interface5ghz+"_nbw="+channel_bw])
                if channel_bw == "20":
                    cmd_list.extend(["nvram set wl"+interface5ghz+"_channel="+channel_no])
            elif channel_bw in ['0', '40']:
                cmd_list.extend(["nvram set wl"+interface5ghz+"_nctrlsb=upper",
                                "nvram set wl"+interface5ghz+"_nbw="+channel_bw])
            elif channel_bw == '80':
                cmd_list.extend(["nvram set wl"+interface5ghz+"_nctrlsb=uu",
                                "nvram set wl"+interface5ghz+"_nbw="+channel_bw])
            else:
                raise APConfigException("Invalid channel bandwidth value")

            if txpwr != None:
                if int(txpwr) in range(45, 101):
                    print "nvram set wl"+interface5ghz+"_txpwr="+txpwr
                    cmd_list.extend(["nvram set wl"+interface5ghz+"_txpwr="+txpwr])
                else:
                    raise APConfigException("Invalid tranmission power value")

            if ipv4_class != None:
                if str(ipv4_class):
                    cmd_list.extend(["nvram set lan_ipaddr="+ipv4_class])
                else:
                    raise APConfigException("Invalid ipv4 class address")

        elif mode == "acn":
            cmd_list.extend(["nvram set wl"+interface5ghz+"_afterburner=off",
                             "nvram set wl"+interface5ghz+"_bss_opmode_cap_reqd=2",
                             "nvram set wl"+interface5ghz+"_net_mode=acn-mixed",
                             "nvram set wl"+interface5ghz+"_nmode=2",
                             "nvram set wl"+interface5ghz+"_nreqd=1",
                             "nvram set wl"+interface5ghz+"_gmode=1",
                             "nvram set wl"+interface5ghz+"_phytype=v"])
            if channel_bw in ['5', '10', '20']:
                cmd_list.extend(["nvram set wl"+interface5ghz+"_nctrlsb=none",
                                "nvram set wl"+interface5ghz+"_nbw="+channel_bw])
                if channel_bw == "20":
                    cmd_list.extend(["nvram set wl"+interface5ghz+"_channel="+channel_no])
            elif channel_bw in ['0', '40']:
                cmd_list.extend(["nvram set wl"+interface5ghz+"_nctrlsb=upper",
                                "nvram set wl"+interface5ghz+"_nbw="+channel_bw])
            elif channel_bw == '80':
                cmd_list.extend(["nvram set wl"+interface5ghz+"_nctrlsb=uu",
                                "nvram set wl"+interface5ghz+"_nbw="+channel_bw])
            else:
                raise APConfigException("Invalid channel bandwidth value")

            if txpwr != None:
                if int(txpwr) in range(45, 101):
                    print "nvram set wl"+interface5ghz+"_txpwr="+txpwr
                    cmd_list.extend(["nvram set wl"+interface5ghz+"_txpwr="+txpwr])
                else:
                    raise APConfigException("Invalid tranmission power value")

            if ipv4_class != None:
                if str(ipv4_class):
                    cmd_list.extend(["nvram set lan_ipaddr="+ipv4_class])
                else:
                    raise APConfigException("Invalid ipv4 class address")

        elif mode == "disabled":
            cmd_list.extend(["nvram set wl"+interface5ghz+"_afterburner=off",
                             "nvram set wl"+interface5ghz+"_bss_opmode_cap_reqd=1",
                             "nvram set wl"+interface5ghz+"_net_mode=disabled",
                             "nvram set wl"+interface5ghz+"_nmode=-1",
                             "nvram set wl"+interface5ghz+"_nreqd=0",
                             "nvram set wl"+interface5ghz+"_gmode=-1",
                             "nvram set wl"+interface5ghz+"_phytype=v"])
        else:
            raise APConfigException("Invalid network mode value")
    else:
        raise APConfigException("Invalid option for inteface")


    # security, hidden, change SSID
    cmd_list.extend(generate_min_config(security, encryption=encryption, wifi_password=wifi_password,
                                        radius_ip=radius_ip, radius_secret=radius_secret,
                                        hidden_ssid=hidden_ssid, new_ssid=new_ssid,
                                        interface5ghz=interface5ghz, ipv6_enable=ipv6_enable,
                                        radvd_enable=radvd_enable, dhcp_lease=dhcp_lease))

    return cmd_list


def generate_min_config(security, encryption=None, wifi_password=None, radius_ip=None,
                        radius_secret=None, hidden_ssid="0", new_ssid=None, interface5ghz="0",
                        ipv6_enable=0, radvd_enable=0, dhcp_lease=None):

    cmd_list = []

    if security == "wpa_psk_mixed":
        cmd_list.extend(["nvram set wl"+interface5ghz+"_akm=\"psk psk2\"",
                         "nvram set wl"+interface5ghz+"_security_mode=\"psk psk2\"",
                         "nvram set wl"+interface5ghz+"_wpa_psk=\"" + wifi_password + "\"",
                         "nvram set wl"+interface5ghz+"_wep=disabled",
                         "nvram set wl"+interface5ghz+"_crypto=\"" + encryption + "\""])
    elif security == "wpa_psk":
        cmd_list.extend(["nvram set wl"+interface5ghz+"_akm=\"psk\"",
                         "nvram set wl"+interface5ghz+"_security_mode=\"psk\"",
                         "nvram set wl"+interface5ghz+"_wpa_psk=\""+ wifi_password + "\"",
                         "nvram set wl"+interface5ghz+"_wep=disabled",
                         "nvram set wl"+interface5ghz+"_crypto=\"" + encryption + "\""])
    elif security == "wpa2":
        cmd_list.extend(["nvram set wl"+interface5ghz+"_akm=\"psk2\"",
                         "nvram set wl"+interface5ghz+"_security_mode=\"psk2\"",
                         "nvram set wl"+interface5ghz+"_wpa_psk=\"" + wifi_password + "\"",
                         "nvram set wl"+interface5ghz+"_wep=disabled",
                         "nvram set wl"+interface5ghz+"_crypto=\"" + encryption + "\""])
    elif security == "wpa_enterprise":
        cmd_list.extend(["nvram set wl"+interface5ghz+"_akm=\"wpa\"",
                         "nvram set wl"+interface5ghz+"_security_mode=\"wpa\"",
                         "nvram set wl"+interface5ghz+"_radius_port=1812",
                         "nvram set wl"+interface5ghz+"_radius_ipaddr=" + radius_ip,
                         "nvram set wl"+interface5ghz+"_radius_key=" + radius_secret,
                         "nvram set wl"+interface5ghz+"_wep=disabled",
                         "nvram set wl"+interface5ghz+"_crypto=\"" + encryption + "\""])
    elif security == "wpa2_enterprise":
        cmd_list.extend(["nvram set wl"+interface5ghz+"_akm=\"wpa2\"",
                         "nvram set wl"+interface5ghz+"_security_mode=\"wpa2\"",
                         "nvram set wl"+interface5ghz+"_radius_port=1812",
                         "nvram set wl"+interface5ghz+"_radius_ipaddr=" + radius_ip,
                         "nvram set wl"+interface5ghz+"_radius_key=" + radius_secret,
                         "nvram set wl"+interface5ghz+"_wep=disabled",
                         "nvram set wl"+interface5ghz+"_crypto=\"" + encryption + "\""])
    elif security == "wpa_enterprise_mixed":
        cmd_list.extend(["nvram set wl"+interface5ghz+"_akm=\"wpa wpa2\"",
                         "nvram set wl"+interface5ghz+"_security_mode=\"wpa wpa2\"",
                         "nvram set wl"+interface5ghz+"_radius_port=1812",
                         "nvram set wl"+interface5ghz+"_radius_ipaddr=" + radius_ip,
                         "nvram set wl"+interface5ghz+"_radius_key=" + radius_secret,
                         "nvram set wl"+interface5ghz+"_wep=disabled",
                         "nvram set wl"+interface5ghz+"_crypto=\"" + encryption + "\""])

    elif security == "wep64":

        raise APConfigException("Security configuration not supported on broadcom based routers - WEP")
        """try:
            int(wifi_password, 16)
        except ValueError:
            raise APConfigException("Invalid WEP password - only hexa characters allowed")
        if len(wifi_password) <> 10:
            raise APConfigException("Invalid WEP 64b password - should have 10 characters")

        key1 = wifi_password
        key2 = "B703120019"
        key3 = "938969B47C"
        key4 = "DA9515EE03"
        cmd_list.extend(["nvram set wl"+interface5ghz+"_akm=wep",
                         "nvram set wl"+interface5ghz+"_security_mode=wep",
                         "nvram set wl"+interface5ghz+"_key1=" + key1,
                         "nvram set wl"+interface5ghz+"_key2=" + key2,
                         "nvram set wl"+interface5ghz+"_key3=" + key3,
                         "nvram set wl"+interface5ghz+"_key4=" + key4,
                         "nvram set wl"+interface5ghz+"_wep=enabled",
                         "nvram set wl"+interface5ghz+"_wep_bit=64",
                         "nvram set wl"+interface5ghz+"_passphrase=test",
                         "nvram set wl_wep=enabled",
                         "nvram set wl_wep_bit=64",
                         "nvram set wl"+interface5ghz+"_wep_buf=test" + ":" + key1 + ":" +
                         key2 + ":" + key3 + ":" + key4 + ":1",
                         "nvram set wl"+interface5ghz+"_wep_gen=test" + ":" + key1 + ":" +
                         key2 + ":" + key3 + ":" + key4 + ":1"])
        if encryption == "open" or str(encryption).lower() == "none":
            cmd_list.extend(["nvram set wl"+interface5ghz+"_authmode=open"])
        elif encryption == "shared":
            cmd_list.extend(["nvram set wl"+interface5ghz+"_authmode=shared"])
        else:
            raise APConfigException("Invalid WEP mode value")"""

    elif security == "wep128":

        raise APConfigException("Security configuration not supported on broadcom based routers - WEP")
        """
        try:
            int(wifi_password, 16)
        except ValueError:
            raise APConfigException("Invalid WEP password - only hexa characters allowed")
        if len(wifi_password) <> 26:
            raise APConfigException("Invalid WEP 128b password - should have 26 characters")

        key1 = wifi_password
        key2 = "0079966B0F8AF9ADB2F03DCCF0"
        key3 = "FC00D6272A95700B2BB90F1FFC"
        key4 = "1BF40000E754864AA708294298"
        cmd_list.extend(
            ["nvram set wl"+interface5ghz+"_akm=wep",
             "nvram set wl"+interface5ghz+"_security_mode=wep",
             "nvram set wl"+interface5ghz+"_key1=" + key1,
             "nvram set wl"+interface5ghz+"_key2=" + key2,
             "nvram set wl"+interface5ghz+"_key3=" + key3,
             "nvram set wl"+interface5ghz+"_key4=" + key4,
             "nvram set wl"+interface5ghz+"_wep=enabled",
             "nvram set wl"+interface5ghz+"_wep_bit=128",
             "nvram set wl"+interface5ghz+"_passphrase=test",
             "nvram set wl_wep=enabled",
             "nvram set wl_wep_bit=128",
             "nvram set wl"+interface5ghz+"_wep_buf=test" + ":" + key1 + ":" +
             key2 + ":" + key3 + ":" + key4 + ":1",
             "nvram set wl"+interface5ghz+"_wep_gen=test" + ":" + key1 + ":"
             + key2 + ":" + key3 + ":" + key4 + ":1"])
        if encryption == "open" or str(encryption).lower() == "none":
            cmd_list.extend(["nvram set wl"+interface5ghz+"_authmode=open"])
        elif encryption == "shared":
            cmd_list.extend(["nvram set wl"+interface5ghz+"_authmode=shared"])
        else:
            raise APConfigException("Invalid WEP mode value")"""

    elif security == "none":
        cmd_list.extend(["nvram set wl"+interface5ghz+"_akm=disabled",
                         "nvram set wl"+interface5ghz+"_security_mode=disabled",
                         "nvram set wl"+interface5ghz+"_wep=disabled"])
    else:
        raise APConfigException("Invalid security value")

    # hidden SSID
    if hidden_ssid == "1":
        cmd_list.extend(["nvram set wl"+interface5ghz+"_closed=1"])
    elif hidden_ssid == "0":
        cmd_list.extend(["nvram set wl"+interface5ghz+"_closed=0"])
    else:
        raise APConfigException("Invalid value for hidden setting")

    # change SSID
    if not str(new_ssid).lower() == 'none':
        cmd_list.extend(["nvram set wl"+interface5ghz+"_ssid=" + new_ssid])

    # Enable/ disable IPv6
    cmd_list.extend(["nvram set ipv6_enable={0}".format(ipv6_enable)])
    cmd_list.extend(["nvram set ipv6_enable0={0}".format(ipv6_enable)])

    # Enable/ disable radvd
    cmd_list.extend(["nvram set radvd_enable={0}".format(radvd_enable)])

    # set the dhcp_lease
    if not str(dhcp_lease).lower() == 'none':
        cmd_list.extend(["nvram set dhcp_lease=" + dhcp_lease])

    cmd_list.append("nvram commit")

    return cmd_list


def generate_virtap_config(enable, security = None, encryption = None, wifi_password = None,
                           radius_ip = None, radius_secret = None, hidden_ssid = "0",
                           new_ssid = None, interface5ghz = "0", virt_if_idx = "1",
                           reload_ap=True):

    if interface5ghz == None:
        interface5ghz = "0"

    if str(enable) == "True":
        cmd_list = []
        cmd_list.append("nvram set wl" + interface5ghz + "_vifs=\"" +\
                        update_virt_ifs_string(virt_if_idx, interface5ghz, action="add") + "\"")
        cmd_list.append("nvram commit")
        cmd_list.extend(generate_min_config(security,
                                            encryption = encryption,
                                            wifi_password = wifi_password,
                                            radius_ip = radius_ip,
                                            radius_secret = radius_secret,
                                            hidden_ssid = hidden_ssid,
                                            new_ssid = new_ssid,
                                            interface5ghz = interface5ghz+"." + virt_if_idx))
        if not reload_ap:
            cmd_list.append("cat /tmp/nas.wl{}.{}lan.pid|xargs kill -9".format(interface5ghz, virt_if_idx))
            cmd_list.append("nas -P /tmp/nas.wl{0}.{1}lan.pid -H 34954 -i wl{0}.{1} -A -m 128 -k {2} -s {3} -w 4 -g 3600"\
                             .format(interface5ghz, virt_if_idx, wifi_password, reload_ap))

        return cmd_list
    elif str(enable) == "False":
        if virt_if_idx == "0":
            virtual_ifs = ""
        else:
            virtual_ifs = update_virt_ifs_string(virt_if_idx, interface5ghz, action="remove")

        cmd_list = []
        cmd_list.append("nvram set wl" + interface5ghz + "_vifs='" + virtual_ifs + "'")
        cmd_list.append("nvram commit")
        if not reload_ap:
            cmd_list.append("cat /tmp/nas.wl{}.{}lan.pid|xargs kill -9".format(interface5ghz, virt_if_idx))
            cmd_list.append("nas -P /tmp/nas.wl{0}.{1}lan.pid -H 34954 -i wl{0}.{1} -A -m 128 -k dummypassword -s dummyssid -w 4 -g 3600".format(interface5ghz, virt_if_idx))
        return cmd_list
    else:
        raise APConfigException("Invalid virtual ap operation mode <=> True / False")

# returns the strig for the virtual interafaces to be configured after the
# elimination/addition of the interface index 'virt_if_idx'
def update_virt_ifs_string(virt_if_idx, interface5ghz, action, ssh_host = "192.168.1.1", ssh_user = "root"):
    local_connection = connection_local()
    command = "ssh {0}@{1} '{2}'".format(ssh_user, ssh_host, "nvram get wl" + interface5ghz + "_vifs")
    out, err = local_connection.run_cmd(command = command)

    if action == "remove":
        virt_if_string = ""
        for virt_if in out.strip().split():
            if virt_if.split(".")[-1] != virt_if_idx:
                if virt_if_string != "":
                    virt_if = " " + virt_if
                virt_if_string += virt_if
        return virt_if_string
    elif action == "add":
        #add the virtual if
        virt_if = "wl" + interface5ghz + "." + virt_if_idx
        if out.strip() == "":
            return virt_if
        if virt_if not in out.strip():
            return out.strip() + " " + virt_if
        else:
            return out.strip()
    else:
        raise APConfigException("Invalid virtual interface action: 'remove' or 'add'")
def generate_ap_config_atheros(mode, security, encryption=None, wifi_password=None,
                               radius_ip=None, radius_secret=None, hidden_ssid="0", new_ssid=None,
                               channel_bw="20", channel_no=None, interface5ghz="0",
                               ipv6_enable=0, radvd_enable=0, dhcp_lease=None):

    cmd_list = []
    if channel_bw == None:
        channel_bw = "20"
    if interface5ghz == None:
        interface5ghz = "0"
    if channel_bw == "20":
        # default channels for 20MHz channel bandwidths -> for best compliance with different DUT regulatory domains
        # broadcom routers use channel number, while atheros routers use the channel frequency in their setup
        if channel_no == None and interface5ghz == "0":
            channel_no = wifi_defaults.wifi['sta_channel']
        elif channel_no == None and interface5ghz == "1":
            channel_no = wifi_defaults.wifi['sta_channel_5ghz']

    # NETWORK MODE
    if interface5ghz == '0': # 2.4 GHz
        if mode == "b":
            cmd_list.extend(["nvram set ath"+interface5ghz+"_afterburner=off",
                             "nvram set ath"+interface5ghz+"_gmode=0",
                             "nvram set ath"+interface5ghz+"_bss_opmode_cap_reqd=0",
                             "nvram set ath"+interface5ghz+"_net_mode=b-only",
                             "nvram set ath"+interface5ghz+"_nmode=0",
                             "nvram set ath"+interface5ghz+"_nreqd=0",
                             "nvram set ath"+interface5ghz+"_phytype=g"])
            if channel_bw in ['5', '10', '20']:
                cmd_list.extend(["nvram set ath"+interface5ghz+"_nctrlsb=none",
                                "nvram set ath"+interface5ghz+"_channelbw="+channel_bw])
                if channel_bw == "20":
                    cmd_list.extend(["nvram set ath"+interface5ghz+"_channel=" + wifi_defaults.channel_no_to_freq[channel_no]])
            else:
                raise APConfigException("Invalid channel bandwidth value")

        elif mode == "g":
            cmd_list.extend(["nvram set ath"+interface5ghz+"_afterburner=off",
                             "nvram set ath"+interface5ghz+"_gmode=2",
                             "nvram set ath"+interface5ghz+"_bss_opmode_cap_reqd=1",
                             "nvram set ath"+interface5ghz+"_net_mode=g-only",
                             "nvram set ath"+interface5ghz+"_nmode=0",
                             "nvram set ath"+interface5ghz+"_nreqd=0",
                             "nvram set ath"+interface5ghz+"_phytype=g"])
            if channel_bw in ['5', '10', '20']:
                cmd_list.extend(["nvram set ath"+interface5ghz+"_nctrlsb=none",
                                "nvram set ath"+interface5ghz+"_channelbw="+channel_bw])
                if channel_bw == "20":
                    cmd_list.extend(["nvram set ath"+interface5ghz+"_channel=" + wifi_defaults.channel_no_to_freq[channel_no]])
            else:
                raise APConfigException("Invalid channel bandwidth value")

        elif mode == "bg":
            cmd_list.extend(["nvram set ath"+interface5ghz+"_afterburner=auto3",
                             "nvram set ath"+interface5ghz+"_gmode=1",
                             "nvram set ath"+interface5ghz+"_bss_opmode_cap_reqd=0",
                             "nvram set ath"+interface5ghz+"_net_mode=bg-mixed",
                             "nvram set ath"+interface5ghz+"_nmode=0",
                             "nvram set ath"+interface5ghz+"_nreqd=0",
                             "nvram set ath"+interface5ghz+"_phytype=g"])
            if channel_bw in ['5', '10', '20']:
                cmd_list.extend(["nvram set ath"+interface5ghz+"_nctrlsb=none",
                                "nvram set ath"+interface5ghz+"_channelbw="+channel_bw])
                if channel_bw == "20":
                    cmd_list.extend(["nvram set ath"+interface5ghz+"_channel=" + wifi_defaults.channel_no_to_freq[channel_no]])
            else:
                raise APConfigException("Invalid channel bandwidth value")

        elif mode == "n":
            cmd_list.extend(["nvram set ath"+interface5ghz+"_afterburner=off",
                             "nvram set ath"+interface5ghz+"_gmode=1",
                             "nvram set ath"+interface5ghz+"_bss_opmode_cap_reqd=2",
                             "nvram set ath"+interface5ghz+"_net_mode=n2-only",
                             "nvram set ath"+interface5ghz+"_nmode=2",
                             "nvram set ath"+interface5ghz+"_nreqd=1",
                             "nvram set ath"+interface5ghz+"_phytype=n"])
            if channel_bw in ['5', '10', '20']:
                cmd_list.extend(["nvram set ath"+interface5ghz+"_nctrlsb=none",
                                "nvram set ath"+interface5ghz+"_channelbw="+channel_bw])
                if channel_bw == "20":
                    cmd_list.extend(["nvram set ath"+interface5ghz+"_channel=" + wifi_defaults.channel_no_to_freq[channel_no]])
            elif channel_bw in ['2040', '40']:
                cmd_list.extend(["nvram set ath"+interface5ghz+"_nctrlsb=lower",
                                "nvram set ath"+interface5ghz+"_channelbw="+channel_bw])
            else:
                raise APConfigException("Invalid channel bandwidth value")

        elif mode == "ng":
            cmd_list.extend(["nvram set ath"+interface5ghz+"_afterburner=auto",
                             "nvram set ath"+interface5ghz+"_gmode=2",
                             "nvram set ath"+interface5ghz+"_bss_opmode_cap_reqd=1",
                             "nvram set ath"+interface5ghz+"_net_mode=ng-only",
                             "nvram set ath"+interface5ghz+"_nmode=2",
                             "nvram set ath"+interface5ghz+"_nreqd=0",
                             "nvram set ath"+interface5ghz+"_phytype=g"])
            if channel_bw in ['5', '10', '20']:
                cmd_list.extend(["nvram set ath"+interface5ghz+"_nctrlsb=none",
                                "nvram set ath"+interface5ghz+"_channelbw="+channel_bw])
                if channel_bw == "20":
                    cmd_list.extend(["nvram set ath"+interface5ghz+"_channel=" + wifi_defaults.channel_no_to_freq[channel_no]])
            elif channel_bw in ['2040', '40']:
                cmd_list.extend(["nvram set ath"+interface5ghz+"_nctrlsb=lower",
                                "nvram set ath"+interface5ghz+"_channelbw="+channel_bw])
            else:
                raise APConfigException("Invalid channel bandwidth value")

        elif mode == "mixed":
            cmd_list.extend(["nvram set ath"+interface5ghz+"_afterburner=auto",
                             "nvram set ath"+interface5ghz+"_gmode=1",
                             "nvram set ath"+interface5ghz+"_bss_opmode_cap_reqd=0",
                             "nvram set ath"+interface5ghz+"_net_mode=mixed",
                             "nvram set ath"+interface5ghz+"_nmode=-1",
                             "nvram set ath"+interface5ghz+"_nreqd=0",
                             "nvram set ath"+interface5ghz+"_phytype=g"])
            if channel_bw in ['5', '10', '20']:
                cmd_list.extend(["nvram set ath"+interface5ghz+"_nctrlsb=none",
                                "nvram set ath"+interface5ghz+"_channelbw="+channel_bw])
                if channel_bw == "20":
                    cmd_list.extend(["nvram set ath"+interface5ghz+"_channel=" + wifi_defaults.channel_no_to_freq[channel_no]])
            elif channel_bw in ['2040', '40']:
                cmd_list.extend(["nvram set ath"+interface5ghz+"_nctrlsb=lower",
                                "nvram set ath"+interface5ghz+"_channelbw="+channel_bw])
            else:
                raise APConfigException("Invalid channel bandwidth value")

        elif mode == "disabled":
            cmd_list.extend(["nvram set ath"+interface5ghz+"_afterburner=off",
                             "nvram set ath"+interface5ghz+"_gmode=-1",
                             "nvram set ath"+interface5ghz+"_bss_opmode_cap_reqd=1",
                             "nvram set ath"+interface5ghz+"_net_mode=disabled",
                             "nvram set ath"+interface5ghz+"_nmode=-1",
                             "nvram set ath"+interface5ghz+"_nreqd=0",
                             "nvram set ath"+interface5ghz+"_phytype=g"])
        else:
            raise APConfigException("Invalid network mode value")

    elif interface5ghz == '1': # 5 GHz

        if mode == "a":
            cmd_list.extend(["nvram set ath"+interface5ghz+"_afterburner=auto",
                             "nvram set ath"+interface5ghz+"_bss_opmode_cap_reqd=1",
                             "nvram set ath"+interface5ghz+"_net_mode=a-only",
                             "nvram set ath"+interface5ghz+"_nmode=0",
                             "nvram set ath"+interface5ghz+"_nreqd=0",
                             "nvram set ath"+interface5ghz+"_phytype=a"])
            if channel_bw in ['5', '10', '20']:
                cmd_list.extend(["nvram set ath"+interface5ghz+"_nctrlsb=none",
                                "nvram set ath"+interface5ghz+"_channelbw="+channel_bw])
                if channel_bw == "20":
                    cmd_list.extend(["nvram set ath"+interface5ghz+"_channel=" + wifi_defaults.channel_no_to_freq[channel_no]])
            else:
                raise APConfigException("Invalid channel bandwidth value")

        elif mode == "n":
            cmd_list.extend(["nvram set ath"+interface5ghz+"_afterburner=off",
                             "nvram set ath"+interface5ghz+"_bss_opmode_cap_reqd=2",
                             "nvram set ath"+interface5ghz+"_net_mode=n5-only",
                             "nvram set ath"+interface5ghz+"_nmode=2",
                             "nvram set ath"+interface5ghz+"_nreqd=1",
                             "nvram set ath"+interface5ghz+"_phytype=n"])
            if channel_bw in ['5', '10', '20']:
                cmd_list.extend(["nvram set ath"+interface5ghz+"_nctrlsb=none",
                                "nvram set ath"+interface5ghz+"_channelbw="+channel_bw])
                if channel_bw == "20":
                    cmd_list.extend(["nvram set ath"+interface5ghz+"_channel=" + wifi_defaults.channel_no_to_freq[channel_no]])
            elif channel_bw in ['2040', '40']:
                cmd_list.extend(["nvram set ath"+interface5ghz+"_nctrlsb=upper",
                                "nvram set ath"+interface5ghz+"_channelbw="+channel_bw])
            else:
                raise APConfigException("Invalid channel bandwidth value")

        elif mode == "na":
            cmd_list.extend(["nvram set ath"+interface5ghz+"_afterburner=off",
                             "nvram set ath"+interface5ghz+"_bss_opmode_cap_reqd=1",
                             "nvram set ath"+interface5ghz+"_net_mode=na-only",
                             "nvram set ath"+interface5ghz+"_nmode=2",
                             "nvram set ath"+interface5ghz+"_nreqd=0",
                             "nvram set ath"+interface5ghz+"_phytype=n"])
            if channel_bw in ['5', '10', '20']:
                cmd_list.extend(["nvram set ath"+interface5ghz+"_nctrlsb=none",
                                "nvram set ath"+interface5ghz+"_channelbw="+channel_bw])
                if channel_bw == "20":
                    cmd_list.extend(["nvram set ath"+interface5ghz+"_channel=" + wifi_defaults.channel_no_to_freq[channel_no]])
            elif channel_bw in ['2040', '40']:
                cmd_list.extend(["nvram set ath"+interface5ghz+"_nctrlsb=upper",
                                "nvram set ath"+interface5ghz+"_channelbw="+channel_bw])
            else:
                raise APConfigException("Invalid channel bandwidth value")

        elif mode == "mixed":
            cmd_list.extend(["nvram set ath"+interface5ghz+"_afterburner=auto",
                             "nvram set ath"+interface5ghz+"_bss_opmode_cap_reqd=0",
                             "nvram set ath"+interface5ghz+"_net_mode=mixed",
                             "nvram set ath"+interface5ghz+"_nmode=-1",
                             "nvram set ath"+interface5ghz+"_nreqd=0",
                             "nvram set ath"+interface5ghz+"_phytype=g"])
            if channel_bw in ['5', '10', '20']:
                cmd_list.extend(["nvram set ath"+interface5ghz+"_nctrlsb=none",
                                "nvram set ath"+interface5ghz+"_channelbw="+channel_bw])
                if channel_bw == "20":
                    cmd_list.extend(["nvram set ath"+interface5ghz+"_channel=" + wifi_defaults.channel_no_to_freq[channel_no]])
            elif channel_bw in ['2040', '40']:
                cmd_list.extend(["nvram set ath"+interface5ghz+"_nctrlsb=upper",
                                "nvram set ath"+interface5ghz+"_channelbw="+channel_bw])
            else:
                raise APConfigException("Invalid channel bandwidth value")

        elif mode == "disabled":
            cmd_list.extend(["nvram set ath"+interface5ghz+"_afterburner=off",
                             "nvram set ath"+interface5ghz+"_bss_opmode_cap_reqd=2",
                             "nvram set ath"+interface5ghz+"_net_mode=disabled",
                             "nvram set ath"+interface5ghz+"_nmode=-1",
                             "nvram set ath"+interface5ghz+"_nreqd=0",
                             "nvram set ath"+interface5ghz+"_phytype=n"])
        else:
            raise APConfigException("Invalid network mode value")
    else:
        raise APConfigException("Invalid option for inteface")

    # security, hidden, change SSID
    cmd_list.extend(generate_min_config_atheros(security, encryption=encryption, wifi_password=wifi_password,
                                                radius_ip=radius_ip, radius_secret=radius_secret,
                                                hidden_ssid=hidden_ssid, new_ssid=new_ssid,
                                                interface5ghz=interface5ghz, ipv6_enable=ipv6_enable,
                                                radvd_enable=radvd_enable, dhcp_lease=dhcp_lease))

    return cmd_list


def generate_min_config_atheros(security, encryption=None, wifi_password=None, radius_ip=None,
                                radius_secret=None, hidden_ssid="0", new_ssid=None, interface5ghz="0",
                                ipv6_enable=0, radvd_enable=0, dhcp_lease=None):

    cmd_list = []

    # SECURITY
    if security == "wpa_psk_mixed":
        cmd_list.extend(["nvram set ath"+interface5ghz+"_akm=\"psk psk2\"",
                         "nvram set ath"+interface5ghz+"_security_mode=\"psk psk2\"",
                         "nvram set ath"+interface5ghz+"_wpa_psk=\"" + wifi_password + "\"",
                         "nvram set ath"+interface5ghz+"_wep=disabled",
                         "nvram set ath"+interface5ghz+"_crypto=\"" + encryption + "\""])
    elif security == "wpa_psk":
        cmd_list.extend(["nvram set ath"+interface5ghz+"_akm=\"psk\"",
                         "nvram set ath"+interface5ghz+"_security_mode=\"psk\"",
                         "nvram set ath"+interface5ghz+"_wpa_psk=\""+ wifi_password + "\"",
                         "nvram set ath"+interface5ghz+"_wep=disabled",
                         "nvram set ath"+interface5ghz+"_crypto=\"" + encryption + "\""])
    elif security == "wpa2":
        cmd_list.extend(["nvram set ath"+interface5ghz+"_akm=\"psk2\"",
                         "nvram set ath"+interface5ghz+"_security_mode=\"psk2\"",
                         "nvram set ath"+interface5ghz+"_wpa_psk=\"" + wifi_password + "\"",
                         "nvram set ath"+interface5ghz+"_wep=disabled",
                         "nvram set ath"+interface5ghz+"_crypto=\"" + encryption + "\""])
    elif security == "wpa_enterprise":
        cmd_list.extend(["nvram set ath"+interface5ghz+"_akm=\"wpa\"",
                         "nvram set ath"+interface5ghz+"_security_mode=\"wpa\"",
                         "nvram set ath"+interface5ghz+"_radius_port=1812",
                         "nvram set ath"+interface5ghz+"_radius_ipaddr=" + radius_ip,
                         "nvram set ath"+interface5ghz+"_radius_key=" + radius_secret,
                         "nvram set ath"+interface5ghz+"_wep=disabled",
                         "nvram set ath"+interface5ghz+"_crypto=\"" + encryption + "\""])
    elif security == "wpa2_enterprise":
        cmd_list.extend(["nvram set ath"+interface5ghz+"_akm=\"wpa2\"",
                         "nvram set ath"+interface5ghz+"_security_mode=\"wpa2\"",
                         "nvram set ath"+interface5ghz+"_radius_port=1812",
                         "nvram set ath"+interface5ghz+"_radius_ipaddr=" + radius_ip,
                         "nvram set ath"+interface5ghz+"_radius_key=" + radius_secret,
                         "nvram set ath"+interface5ghz+"_wep=disabled",
                         "nvram set ath"+interface5ghz+"_crypto=\"" + encryption + "\""])
    elif security == "wpa_enterprise_mixed":
        cmd_list.extend(["nvram set ath"+interface5ghz+"_akm=\"wpa wpa2\"",
                         "nvram set ath"+interface5ghz+"_security_mode=\"wpa wpa2\"",
                         "nvram set ath"+interface5ghz+"_radius_port=1812",
                         "nvram set ath"+interface5ghz+"_radius_ipaddr=" + radius_ip,
                         "nvram set ath"+interface5ghz+"_radius_key=" + radius_secret,
                         "nvram set ath"+interface5ghz+"_wep=disabled",
                         "nvram set ath"+interface5ghz+"_crypto=\"" + encryption + "\""])

    elif security == "wep64":

        try:
            int(wifi_password, 16)
        except ValueError:
            raise APConfigException("Invalid WEP password - only hexa characters allowed")
        if len(wifi_password) <> 10:
            raise APConfigException("Invalid WEP 64b password - should have 10 characters")

        key1 = wifi_password
        key2 = "941E4148D0"
        key3 = "91B3CDF063"
        key4 = "E38D0086C9"
        cmd_list.extend(["nvram set ath"+interface5ghz+"_akm=wep",
                         "nvram set ath"+interface5ghz+"_security_mode=wep",
                         "nvram set ath"+interface5ghz+"_key1=" + key1,
                         "nvram set ath"+interface5ghz+"_key2=" + key2,
                         "nvram set ath"+interface5ghz+"_key3=" + key3,
                         "nvram set ath"+interface5ghz+"_key4=" + key4,
                         "nvram set ath"+interface5ghz+"_wep=enabled",
                         "nvram set ath"+interface5ghz+"_wep_bit=64",
                         "nvram set ath"+interface5ghz+"_passphrase=test",
                         "nvram set wl_wep=enabled",
                         "nvram set wl_wep_bit=64",
                         "nvram set ath"+interface5ghz+"_wep_buf=test" + ":" + key1 + ":" +
                         key2 + ":" + key3 + ":" + key4 + ":1",
                         "nvram set ath"+interface5ghz+"_wep_gen=test" + ":" + key1 + ":" +
                         key2 + ":" + key3 + ":" + key4 + ":1"])
        if encryption == "open" or str(encryption).lower() == "none":
            cmd_list.extend(["nvram set ath"+interface5ghz+"_authmode=open"])
        elif encryption == "shared":
            cmd_list.extend(["nvram set ath"+interface5ghz+"_authmode=shared"])
        else:
            raise APConfigException("Invalid WEP mode value")

    elif security == "wep128":

        try:
            int(wifi_password, 16)
        except ValueError:
            raise APConfigException("Invalid WEP password - only hexa characters allowed")
        if len(wifi_password) <> 26:
            raise APConfigException("Invalid WEP 128b password - should have 26 characters")

        key1 = wifi_password
        key2 = "0079966B0F8AF9ADB2F03DCCF0"
        key3 = "FC00D6272A95700B2BB90F1FFC"
        key4 = "1BF40000E754864AA708294298"
        cmd_list.extend(
            ["nvram set ath"+interface5ghz+"_akm=wep",
             "nvram set ath"+interface5ghz+"_security_mode=wep",
             "nvram set ath"+interface5ghz+"_key1=" + key1,
             "nvram set ath"+interface5ghz+"_key2=" + key2,
             "nvram set ath"+interface5ghz+"_key3=" + key3,
             "nvram set ath"+interface5ghz+"_key4=" + key4,
             "nvram set ath"+interface5ghz+"_wep=enabled",
             "nvram set ath"+interface5ghz+"_wep_bit=128",
             "nvram set ath"+interface5ghz+"_passphrase=test",
             "nvram set wl_wep=enabled",
             "nvram set wl_wep_bit=128",
             "nvram set ath"+interface5ghz+"_wep_buf=test" + ":" + key1 + ":" +
             key2 + ":" + key3 + ":" + key4 + ":1",
             "nvram set ath"+interface5ghz+"_wep_gen=test" + ":" + key1 + ":"
             + key2 + ":" + key3 + ":" + key4 + ":1"])
        if encryption == "open" or str(encryption).lower() == "none":
            cmd_list.extend(["nvram set ath"+interface5ghz+"_authmode=open"])
        elif encryption == "shared":
            cmd_list.extend(["nvram set ath"+interface5ghz+"_authmode=shared"])
        else:
            raise APConfigException("Invalid WEP mode value")

    elif security == "none":
        cmd_list.extend(["nvram set ath"+interface5ghz+"_akm=disabled",
                         "nvram set ath"+interface5ghz+"_security_mode=disabled",
                         "nvram set ath"+interface5ghz+"_wep=disabled"])
    else:
        raise APConfigException("Invalid security value")

    # hidden SSID
    if hidden_ssid == "1":
        cmd_list.extend(["nvram set ath"+interface5ghz+"_closed=1"])
    elif hidden_ssid == "0":
        cmd_list.extend(["nvram set ath"+interface5ghz+"_closed=0"])
    else:
        raise APConfigException("Invalid value for hidden setting")

    # change SSID
    if not str(new_ssid).lower() == 'none':
        cmd_list.extend(["nvram set ath"+interface5ghz+"_ssid=" + new_ssid])

    # Enable/ disable IPv6
    cmd_list.extend(["nvram set ipv6_enable={0}".format(ipv6_enable)])
    cmd_list.extend(["nvram set ipv6_enable0={0}".format(ipv6_enable)])

    # Enable/ disable radvd
    cmd_list.extend(["nvram set radvd_enable={0}".format(radvd_enable)])

    # set the dhcp_lease
    if not str(dhcp_lease).lower() == 'none':
        cmd_list.extend(["nvram set dhcp_lease=" + dhcp_lease])

    cmd_list.append("nvram commit")

    return cmd_list


def generate_virtap_config_atheros(enable, security = None, encryption = None, wifi_password = None, radius_ip = None,
                        radius_secret = None, hidden_ssid = "0", new_ssid = None, interface5ghz = "0"):

    if interface5ghz == None:
        interface5ghz = "0"

    if str(enable) == "True":
        cmd_list = []
        cmd_list.append("nvram set ath"+interface5ghz+"_vifs=ath"+interface5ghz+".1")
        cmd_list.append("nvram commit")
        cmd_list.extend(generate_min_config_atheros(security, encryption = encryption, wifi_password = wifi_password, radius_ip = radius_ip,
                            radius_secret = radius_secret, hidden_ssid = hidden_ssid, new_ssid = new_ssid, interface5ghz = interface5ghz+".1"))
        return cmd_list
    elif str(enable) == "False":
        cmd_list = []
        cmd_list.append("nvram set ath"+interface5ghz+"_vifs=")
        cmd_list.append("nvram commit")
        return cmd_list
    else:
        raise APConfigException("Invalid virtual ap operation mode <=> True / False")

def split_config(config):
    cmd_list = []
    cmd_string = ""
    for command in config:
        if len(cmd_string) + len(command) < 1000:
            if cmd_string == "":
                cmd_string = command
            else:
                cmd_string = ";".join([cmd_string,command])
        else:
            cmd_list.append(cmd_string)
            cmd_string = command
    cmd_list.append(cmd_string)
    return cmd_list
