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

:organization: INTEL OTC ANDROID CORE QA
:summary: Function that return a command list to configure the router
:since: 04/05/2014
:author: mcchilax
"""

from ErrorHandling.AcsConfigException import AcsConfigException

def get_config_cmds(operation_mode, security_mode, algorithm, password, radius_ip=None,
                    radius_key=None, ap_type = "ddwrt_broadcom"):

    if ap_type == "ddwrt_broadcom":
        cmd_list = get_config_cmds_broadcom(operation_mode=operation_mode,
                    security_mode=security_mode, algorithm=algorithm,
                    password=password, radius_ip=radius_ip, radius_key=radius_key)
    elif ap_type == "ddwrt_atheros":
        cmd_list = get_config_cmds_atheros(operation_mode=operation_mode,
                    security_mode=security_mode, algorithm=algorithm,
                    password=password, radius_ip=radius_ip, radius_key=radius_key)
    else:
        raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                "Invalid router type. Please verify the test case")

    return cmd_list

def get_config_cmds_broadcom(operation_mode, security_mode, algorithm, password,
                             radius_ip=None, radius_key=None):
    """
    This function return a command list to configure the router based on
    Operation mode, Security mode and algorithm for wpa and wpa2
    The command list contains at the end the commands to apply the
    configuration with out restarting the router
    """
    cmd_list = []
    if security_mode == "wpa_psk_mixed":
        cmd_list.extend(["nvram set wl0_akm=\"psk psk2\"",
                         "nvram set wl0_security_mode=\"psk psk2\"",
                         "nvram set wl0_wpa_psk=\"" + password + "\"",
                         "nvram set wl0_wep=disabled",
                         "nvram set wl0_crypto=\"" + algorithm + "\""])
    elif security_mode == "wpa_psk":
        cmd_list.extend(["nvram set wl0_akm=\"psk\"",
                         "nvram set wl0_security_mode=\"psk\"",
                         "nvram set wl0_wpa_psk=\""+ password + "\"",
                         "nvram set wl0_wep=disabled",
                         "nvram set wl0_crypto=\"" + algorithm + "\""])
    elif security_mode == "wpa2":
        cmd_list.extend(["nvram set wl0_akm=\"psk2\"",
                         "nvram set wl0_security_mode=\"psk2\"",
                         "nvram set wl0_wpa_psk=\"" + password + "\"",
                         "nvram set wl0_wep=disabled",
                         "nvram set wl0_crypto=\"" + algorithm + "\""])
    elif security_mode == "wpa_enterprise":
        cmd_list.extend(["nvram set wl0_akm=\"wpa\"",
                         "nvram set wl0_security_mode=\"wpa\"",
                         "nvram set wl0_radius_port=1812",
                         "nvram set wl0_radius_ipaddr=" + radius_ip,
                         "nvram set wl0_radius_key=" + radius_key,
                         "nvram set wl0_wep=disabled",
                         "nvram set wl0_crypto=\"" + algorithm + "\""])
    elif security_mode == "wpa2_enterprise":
        cmd_list.extend(["nvram set wl0_akm=\"wpa2\"",
                         "nvram set wl0_security_mode=\"wpa2\"",
                         "nvram set wl0_radius_port=1812",
                         "nvram set wl0_radius_ipaddr=" + radius_ip,
                         "nvram set wl0_radius_key=" + radius_key,
                         "nvram set wl0_wep=disabled",
                         "nvram set wl0_crypto=\"" + algorithm + "\""])
    elif security_mode == "wep64":
        key1 = "49774B692D"
        key2 = "B703120019"
        key3 = "938969B47C"
        key4 = "DA9515EE03"
        cmd_list.extend(["nvram set wl0_wep_buf=" + password + ":" + key1 +
                         ":" + key2 + ":" + key3 + ":" + key4 + ":1",
                         "nvram set wl0_akm=wep",
                         "nvram set wl0_security_mode=wep",
                         "nvram set wl0_key1=" + key1,
                         "nvram set wl0_key2=" + key2,
                         "nvram set wl0_key3=" + key3,
                         "nvram set wl0_key4=" + key4,
                         "nvram set wl0_wep=enabled",
                         "nvram set wl0_wep_bit=64",
                         "nvram set wl0_passphrase=" + password,
                         "nvram set wl0_wep_gen=" + password + ":" + key1 + ":" +
                         key2 + ":" + key3 + ":" + key4 + ":1",
                         "nvram set wl_wep=enabled",
                         "nvram set wl_wep_bit=64",
                         "nvram set wl0_wep_buf=" + password + ":" + key1 + ":" +
                         key2 + ":" + key3 + ":" + key4 + ":1",
                         "nvram set wl0_wep_gen=" + password + ":" + key1 + ":" +
                         key2 + ":" + key3 + ":" + key4 + ":1"])
    elif security_mode == "wep128":
        key1 = "9417DCC8901724A74C0E65C7E0"
        key2 = "003625BE269601E2F59B947355"
        key3 = "9400F6712DF00B0648D3A97565"
        key4 = "E476004F367BFD457362CC5491"
        cmd_list.extend(
            ["nvram set wl0_wep_buf=" + password + ":" + key1 + ":" +
             key2 + ":" + key3 + ":" + key4 + ":1",
             "nvram set wl0_akm=wep",
             "nvram set wl0_security_mode=wep",
             "nvram set wl0_key1=" + key1,
             "nvram set wl0_key2=" + key2,
             "nvram set wl0_key3=" + key3,
             "nvram set wl0_key4=" + key4,
             "nvram set wl0_wep=enabled",
             "nvram set wl0_wep_bit=128",
             "nvram set wl0_passphrase=" + password,
             "nvram set wl0_wep_gen=" + password + ":" + key1 + ":" +
             key2 + ":" + key3 + ":" + key4 + ":1",
             "nvram set wl_wep=enabled",
             "nvram set wl_wep_bit=128",
             "nvram set wl0_wep_buf=" + password + ":" + key1 + ":" +
             key2 + ":" + key3 + ":" + key4 + ":1",
             "nvram set wl0_wep_gen=" + password + ":" + key1 + ":"
             + key2 + ":" + key3 + ":" + key4 + ":1"])
    elif security_mode == "open":
        cmd_list.extend(["nvram set wl0_akm=disabled",
                         "nvram set wl0_security_mode=disabled",
                         "nvram set wl0_wep=disabled"])
    else:
        raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "There is a problem with the security mode parameter")
    if operation_mode == "b":
        cmd_list.extend(["nvram set wl0_afterburner=off",
                         "nvram set wl0_frameburst=on",
                         "nvram set wl0_gmode=0",
                         "nvram set wl0_nctrlsb=",
                         "nvram set wl0_net_mode=b-only",
                         "nvram set wl0_nmode=0",
                         "nvram set wl0_nreqd=0",
                         "nvram set wl0_phytype=n",
                         "nvram set wl_afterburner=off",
                         "nvram set wl_bss_opmode_cap_reqd=0",
                         "nvram set wl_channel=6",
                         "nvram set wl_frameburst=on",
                         "nvram set wl_gmode=0",
                         "nvram set wl_nband=2",
                         "nvram set wl_net_mode=b-only",
                         "nvram set wl_nmode=0",
                         "nvram set wl_nreqd=0",
                         "nvram set wl_phytype=g"])
    elif operation_mode == "g":
        cmd_list.extend(["nvram set wl0_afterburner=off",
                         "nvram set wl0_frameburst=on",
                         "nvram set wl0_gmode=2",
                         "nvram set wl0_nctrlsb=",
                         "nvram set wl0_net_mode=g-only",
                         "nvram set wl0_nmode=0",
                         "nvram set wl0_nreqd=0",
                         "nvram set wl0_phytype=n",
                         "nvram set wl_afterburner=off",
                         "nvram set wl_bss_opmode_cap_reqd=1",
                         "nvram set wl_channel=6",
                         "nvram set wl_frameburst=on",
                         "nvram set wl_gmode=2",
                         "nvram set wl_nband=2",
                         "nvram set wl_net_mode=g-only",
                         "nvram set wl_nmode=0",
                         "nvram set wl_nreqd=0",
                         "nvram set wl_phytype=g"])
    elif operation_mode == "bg":
        cmd_list.extend(["nvram set wl0_afterburner=auto",
                         "nvram set wl0_frameburst=on",
                         "nvram set wl0_gmode=1",
                         "nvram set wl0_nctrlsb=none",
                         "nvram set wl0_net_mode=bg-mixed",
                         "nvram set wl0_nmode=0",
                         "nvram set wl0_nreqd=0",
                         "nvram set wl0_phytype=n",
                         "nvram set wl_afterburner=auto",
                         "nvram set wl_bss_opmode_cap_reqd=0",
                         "nvram set wl_channel=6",
                         "nvram set wl_frameburst=on",
                         "nvram set wl_gmode=1",
                         "nvram set wl_nband=2",
                         "nvram set wl_net_mode=bg-mixed",
                         "nvram set wl_nmode=0",
                         "nvram set wl_nreqd=0",
                         "nvram set wl_phytype=g"])
    elif operation_mode == "n":
        cmd_list.extend(["nvram set wl0_afterburner=off",
                         "nvram set wl0_frameburst=on",
                         "nvram set wl0_gmode=1",
                         "nvram set wl0_nctrlsb=none",
                         "nvram set wl0_net_mode=n-only",
                         "nvram set wl0_nmode=2",
                         "nvram set wl0_nreqd=1",
                         "nvram set wl0_phytype=n",
                         "nvram set wl_afterburner=off",
                         "nvram set wl_bss_opmode_cap_reqd=2",
                         "nvram set wl_channel=6",
                         "nvram set wl_frameburst=on",
                         "nvram set wl_gmode=1",
                         "nvram set wl_nband=2",
                         "nvram set wl_net_mode=n-only",
                         "nvram set wl_nmode=2",
                         "nvram set wl_nreqd=1",
                         "nvram set wl_phytype=n"])
    elif operation_mode == "ng":
        cmd_list.extend(["nvram set wl0_afterburner=off",
                         "nvram set wl0_frameburst=on",
                         "nvram set wl0_gmode=2",
                         "nvram set wl0_nctrlsb=none",
                         "nvram set wl0_net_mode=ng-only",
                         "nvram set wl0_nmode=2",
                         "nvram set wl0_nreqd=0",
                         "nvram set wl0_phytype=n",
                         "nvram set wl_afterburner=off",
                         "nvram set wl_bss_opmode_cap_reqd=1",
                         "nvram set wl_channel=6",
                         "nvram set wl_frameburst=on",
                         "nvram set wl_gmode=2",
                         "nvram set wl_nband=2",
                         "nvram set wl_net_mode=ng-only",
                         "nvram set wl_nmode=2",
                         "nvram set wl_nreqd=0",
                         "nvram set wl_phytype=g"])
    elif operation_mode == "mixed":
        cmd_list.extend(["nvram set wl0_afterburner=auto",
                         "nvram set wl0_frameburst=on",
                         "nvram set wl0_gmode=1",
                         "nvram set wl0_nctrlsb=none",
                         "nvram set wl0_net_mode=mixed",
                         "nvram set wl0_nmode=-1",
                         "nvram set wl0_nreqd=0",
                         "nvram set wl0_phytype=n",
                         "nvram set wl_afterburner=auto",
                         "nvram set wl_bss_opmode_cap_reqd=0",
                         "nvram set wl_channel=6",
                         "nvram set wl_frameburst=on",
                         "nvram set wl_gmode=1",
                         "nvram set wl_nband=2",
                         "nvram set wl_net_mode=mixed",
                         "nvram set wl_nmode=-1",
                         "nvram set wl_nreqd=0",
                         "nvram set wl_phytype=g"])
    else:
        raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "There is a problem with the operation mode parameter")
    cmd_list.append("nvram commit")
    #cmd_list.extend(["sleep 5",
                     #"wl down",
                     #"wlconf wl0 down",
                     #"stopservice wland",
                     #"stopservice nas",
                     #"sleep 5",
                     #"startservice nas",
                     #"startservice wland",
                     #"wlconf eth1 up",
                     #"wl -i eth1 up"])
    return cmd_list



def get_config_cmds_atheros(operation_mode, security_mode, algorithm, password, radius_ip=None, radius_key=None):
    """
    This function return a command list to configure the router based on
    Operation mode, Security mode and algorithm for wpa and wpa2
    The command list contains at the end the commands to apply the
    configuration with out restarting the router
    """
    cmd_list = []
    if security_mode == "wpa_psk_mixed":
        cmd_list.extend(["nvram set ath0_akm=\"psk psk2\"",
                         "nvram set ath0_security_mode=\"psk psk2\"",
                         "nvram set ath0_wpa_psk=\"" + password + "\"",
                         "nvram set ath0_wep=disabled",
                         "nvram set ath0_crypto=\"" + algorithm + "\""])
    elif security_mode == "wpa_psk":
        cmd_list.extend(["nvram set ath0_akm=\"psk\"",
                         "nvram set ath0_security_mode=\"psk\"",
                         "nvram set ath0_wpa_psk=\""+ password + "\"",
                         "nvram set ath0_wep=disabled",
                         "nvram set ath0_crypto=\"" + algorithm + "\""])
    elif security_mode == "wpa2":
        cmd_list.extend(["nvram set ath0_akm=\"psk2\"",
                         "nvram set ath0_security_mode=\"psk2\"",
                         "nvram set ath0_wpa_psk=\"" + password + "\"",
                         "nvram set ath0_wep=disabled",
                         "nvram set ath0_crypto=\"" + algorithm + "\""])
    elif security_mode == "wpa_enterprise":
        cmd_list.extend(["nvram set ath0_akm=\"wpa\"",
                         "nvram set ath0_security_mode=\"wpa\"",
                         "nvram set ath0_radius_port=1812",
                         "nvram set ath0_radius_ipaddr=" + radius_ip,
                         "nvram set ath0_radius_key=" + radius_key,
                         "nvram set ath0_wep=disabled",
                         "nvram set ath0_crypto=\"" + algorithm + "\""])
    elif security_mode == "wpa2_enterprise":
        cmd_list.extend(["nvram set ath0_akm=\"wpa2\"",
                         "nvram set ath0_security_mode=\"wpa2\"",
                         "nvram set ath0_radius_port=1812",
                         "nvram set ath0_radius_ipaddr=" + radius_ip,
                         "nvram set ath0_radius_key=" + radius_key,
                         "nvram set ath0_wep=disabled",
                         "nvram set ath0_crypto=\"" + algorithm + "\""])
    elif security_mode == "wep64":
        key1 = "49774B692D"
        key2 = "B703120019"
        key3 = "938969B47C"
        key4 = "DA9515EE03"
        cmd_list.extend(["nvram set ath0_akm=wep",
                         "nvram set ath0_security_mode=wep",
                         "nvram set ath0_key1=" + key1,
                         "nvram set ath0_key2=" + key2,
                         "nvram set ath0_key3=" + key3,
                         "nvram set ath0_key4=" + key4,
                         "nvram set ath0_wep=enabled",
                         "nvram set ath0_wep_bit=64",
                         "nvram set ath0_passphrase=" + password,
                         "nvram set wl_wep=enabled",
                         "nvram set wl_wep_bit=64",
                         "nvram set ath0_wep_buf=" + password + ":" + key1 + ":" +
                         key2 + ":" + key3 + ":" + key4 + ":1",
                         "nvram set ath0_wep_gen=" + password + ":" + key1 + ":" +
                         key2 + ":" + key3 + ":" + key4 + ":1"])
    elif security_mode == "wep128":
        key1 = "9417DCC8901724A74C0E65C7E0"
        key2 = "003625BE269601E2F59B947355"
        key3 = "9400F6712DF00B0648D3A97565"
        key4 = "E476004F367BFD457362CC5491"
        cmd_list.extend(
            ["nvram set ath0_akm=wep",
             "nvram set ath0_security_mode=wep",
             "nvram set ath0_key1=" + key1,
             "nvram set ath0_key2=" + key2,
             "nvram set ath0_key3=" + key3,
             "nvram set ath0_key4=" + key4,
             "nvram set ath0_wep=enabled",
             "nvram set ath0_wep_bit=128",
             "nvram set ath0_passphrase=" + password,
             "nvram set wl_wep=enabled",
             "nvram set wl_wep_bit=128",
             "nvram set ath0_wep_buf=" + password + ":" + key1 + ":" +
             key2 + ":" + key3 + ":" + key4 + ":1",
             "nvram set ath0_wep_gen=" + password + ":" + key1 + ":"
             + key2 + ":" + key3 + ":" + key4 + ":1"])
    elif security_mode == "open":
        cmd_list.extend(["nvram set ath0_akm=disabled",
                         "nvram set ath0_security_mode=disabled",
                         "nvram set ath0_wep=disabled"])
    else:
        raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "There is a problem with the security mode parameter")
    if operation_mode == "b":
        cmd_list.extend(["nvram set ath0_afterburner=off",
                         "nvram set ath0_gmode=0",
                         "nvram set ath0_bss_opmode_cap_reqd=0",
                         "nvram set ath0_net_mode=b-only",
                         "nvram set ath0_nmode=0",
                         "nvram set ath0_nreqd=0",
                         "nvram set ath0_phytype=g"])
    elif operation_mode == "g":
        cmd_list.extend(["nvram set ath0_afterburner=off",
                         "nvram set ath0_gmode=2",
                         "nvram set ath0_bss_opmode_cap_reqd=1",
                         "nvram set ath0_net_mode=g-only",
                         "nvram set ath0_nmode=0",
                         "nvram set ath0_nreqd=0",
                         "nvram set ath0_phytype=g"])
    elif operation_mode == "bg":
        cmd_list.extend(["nvram set ath0_afterburner=auto3",
                         "nvram set ath0_gmode=1",
                         "nvram set ath0_bss_opmode_cap_reqd=0",
                         "nvram set ath0_net_mode=bg-mixed",
                         "nvram set ath0_nmode=0",
                         "nvram set ath0_nreqd=0",
                         "nvram set ath0_phytype=g"])
    elif operation_mode == "n":
        cmd_list.extend(["nvram set ath0_afterburner=off",
                         "nvram set ath0_gmode=1",
                         "nvram set ath0_bss_opmode_cap_reqd=2",
                         "nvram set ath0_net_mode=n2-only",
                         "nvram set ath0_nmode=2",
                         "nvram set ath0_nreqd=1",
                         "nvram set ath0_phytype=n"])
    elif operation_mode == "ng":
        cmd_list.extend(["nvram set ath0_afterburner=auto",
                         "nvram set ath0_gmode=2",
                         "nvram set ath0_bss_opmode_cap_reqd=1",
                         "nvram set ath0_net_mode=ng-only",
                         "nvram set ath0_nmode=2",
                         "nvram set ath0_nreqd=0",
                         "nvram set ath0_phytype=g"])
    elif operation_mode == "mixed":
        cmd_list.extend(["nvram set ath0_afterburner=auto",
                         "nvram set ath0_gmode=1",
                         "nvram set ath0_bss_opmode_cap_reqd=0",
                         "nvram set ath0_net_mode=mixed",
                         "nvram set ath0_nmode=-1",
                         "nvram set ath0_nreqd=0",
                         "nvram set ath0_phytype=g"])
    else:
        raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "There is a problem with the operation mode parameter")
    cmd_list.append("nvram commit")
    return cmd_list


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
