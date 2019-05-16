#!/usr/bin/env python

#######################################################################
#
# @filename:    wifi_utils.py
# @description: WiFi utils
# @author:      ion-horia.petrisor@intel.com
#
#######################################################################

from testlib.utils.ui.uiandroid import UIDevice as ui_device
from testlib.utils.connections.adb import Adb as connection_adb
from testlib.scripts.android.adb.adb_step import step as adb_step
from testlib.scripts.android.adb import adb_utils
from testlib.utils.statics.android import statics
from testlib.external.daemon import createDaemon
from subprocess import Popen, PIPE
import time
import re
import os
import filecmp
import hashlib
import SimpleHTTPServer
import multiprocessing
import os
import signal
import commands
import socket, subprocess, re
import fnmatch
import sys



"""
ddwrt_ip, ddwrt_user, ddwrt_pwd, ddwrt_ap_name, ddwrt_ap_pass, ddwrt_script_path =\
    open("/home/sys_spjenkins/work/testlib/scripts/wireless/wifi/resources.in",
         "rt").readlines()


def get_ddwrt_ip():
    return ddwrt_ip.strip()

def get_ddwrt_user():
    return ddwrt_user.strip()

def get_ddwrt_pwd():
    return ddwrt_pwd.strip()

def get_ddwrt_ap_name():
    return ddwrt_ap_name.strip()

def get_ddwrt_ap_pass():
    return ddwrt_ap_pass.strip()

def get_ddwrt_script_path():
    return ddwrt_script_path.strip()
"""

def is_known_AP(ap_name, serial = None, device = None):

    """ description:
            Checks if the given <ap_name> is known on the <serial> device

        usage:
            is_known_AP(ap_name = "some_AP_name")

        tags:
            ui, android, wifi, AP, known
    """

    if serial:
        uidevice = ui_device(serial = serial)
    else:
        uidevice = ui_device()

    exclude_ap_names = ["Wi?Fi", "On"]
    if ap_name in exclude_ap_names:
            print "WARNING: You are using bad AP names for testing: " + ",".join(exclude_ap_names)

    if uidevice(className = "android.widget.ScrollView", scrollable = True).wait.exists(timeout=1000):
        uidevice(scrollable = True).scroll.to({"text":ap_name})

    uidevice(resourceId = "android:id/title", text = ap_name).wait.exists(timeout=1000)
    time.sleep(0.5)
    uidevice(text = ap_name).click.wait()
    count = 0
    while not uidevice(resourceId = "android:id/title", text = ap_name).wait.gone(timeout=1000) and count < 5:
        count += 1
        uidevice(text = ap_name).click.wait()
    uidevice(resourceId = "android:id/alertTitle", text = ap_name).wait.exists(timeout=1000)
    exists = uidevice(**device.wifi_saved_network_forget_btn_id).wait.exists(timeout=1000)
    print device.wifi_saved_network_cancel_btn_id
    if uidevice(**device.wifi_saved_network_cancel_btn_id).wait.exists(timeout=1000):
        uidevice(**device.wifi_saved_network_cancel_btn_id).click.wait()
    elif uidevice(**device.wifi_saved_network_done_btn_id).wait.exists(timeout=1000):
        uidevice(**device.wifi_saved_network_done_btn_id).click.wait()



    return exists


def wifi_check_AP(ap_name, serial = None, device = None):

    """ description:
            Checks if the given <ap_name> is known on the <serial> device

        usage:
            is_known_AP(ap_name = "some_AP_name")

        tags:
            ui, android, wifi, AP, known
    """

    if serial:
        uidevice = ui_device(serial = serial)
    else:
        uidevice = ui_device()

    exclude_ap_names = ["Wi?Fi", "On"]
    if ap_name in exclude_ap_names:
            print "WARNING: You are using bad AP names for testing: " + ",".join(exclude_ap_names)

    if uidevice(className = "android.widget.ScrollView", scrollable = True).wait.exists(timeout=1000):
        uidevice(scrollable = True).scroll.to({"text":ap_name})
    
    if uidevice(resourceId = "android:id/title", text = ap_name).wait.exists(timeout=1000):
       return(True)
    else: 
       return(False)

def get_mDhcpResults(ret_val, line):
    m = re.search(r'DHCP server \/*([\.\d]+)', line)
    if m != None:
        ret_val['DHCP_server'] = m.group(1)

    m = re.search(r'Gateway \/*([\.\d]+)', line)
    if m != None and ret_val['Gateway'] == None:
        ret_val['Gateway'] = m.group(1)
    return ret_val


def get_mLinkProperties(ret_val, line):
    #m = re.search(r'LinkAddresses: \[[\w\:\/]*?,?([\.\d]+)\/.*\]', line)
    m = re.search(r'LinkAddresses: \[(.*?)\]', line)
    m = re.search(r'(\d+\.\d+\.\d+\.\d+)\/\d+', m.group(1))
    if m != None:
        ret_val['ip_address'] = m.group(1)

    m = re.search(r'LinkAddresses: \[(.*?)\]', line)
    mask = None
    if m != None:
        segment = m.group(1)
        ips = segment.split(",")
        for element in ips:
            m = re.search(r'\d+\.\d+\.\d+\.\d+\/(\d+)', element)
            if m != None:
                mask = m.group(1)
                ret_val['net_mask'] = m.group(1)
                break



    m = re.search(r'DnsAddresses:\s+\[([\.\d,]+?),?\]', line)
    if m != None:
        ret_val['DNS_addresses'] = m.group(1)
    return ret_val


def get_key_mgmt(ret_val, line):
    line = line.strip()
    if 'key_mgmt' in line:
        ret_val['Security'] = line.split('=')[1]
    elif "KeyMgmt" in line:
        ret_val['Security'] = line.split(' Protocols:')[0].split(': ')[1]
    elif 'p2p_device_address' in line:
        ret_val['p2p_device_address'] = line.split('=')[1]
    elif 'pairwise_cipher' in line:
        ret_val['pairwise_cipher'] = line.split('=')[1]
    elif 'PairwiseCiphers' in line:
        ret_val['pairwise_cipher'] = line.split(': ')[1]
    elif 'group_cipher' in line:
        ret_val['group_cipher'] = line.split('=')[1]
    return ret_val


def get_mWifiInfo(ret_val, line):
    for element in line:
        if 'mWifiInfo SSID' in element:
            ret_val['SSID'] = element.split(': ')[1]
        elif 'Frequency' in element:
            ret_val['Frequency'] = element.split(': ')[1]
        elif 'Link speed' in element:
            ret_val['Link_speed'] = element.split(': ')[1]
        elif 'MAC' in element:
            ret_val['MAC'] = element.split(': ')[1]
        elif 'state' in element and 'Supplicant' not in element:
            ret_val['state'] = element.split(': ')[1]

    return ret_val


def get_connection_info(content):
    # imitialize the values
    ret_val = {'DHCP_server': None,
               'DNS_addresses': None,
               'Frequency': None,
               'Gateway': None,
               'Link_speed': None,
               'MAC': None,
               'SSID': None,
               'Security': None,
               'ip_address': None,
               'p2p_device_address': None,
               'state': None,
               'pairwise_cipher': None,
               'group_cipher': None,
               'net_mask':None}
    state_section_found = False
    for line in content.split("\n"):
        if not state_section_found:
            if "WifiStateMachine:" in line:
                state_section_found = True
            else:
                continue
        else:
            if "mWifiInfo" in line:
                get_mWifiInfo(ret_val, line.split(", "))
            elif "mNetworkInfo" in line:
                get_mWifiInfo(ret_val, line.split(", "))
            elif "mDhcpResults" in line:
                get_mDhcpResults(ret_val, line)
            elif "mLinkProperties" in line:
                get_mLinkProperties(ret_val, line)
            elif "key_mgmt=" in line:
                get_key_mgmt(ret_val, line)
            elif "KeyMgmt:" in line and "WifiService:" not in line:
                get_key_mgmt(ret_val, line)
            elif "p2p_device_address=" in line:
                get_key_mgmt(ret_val, line)
            elif "pairwise_cipher=" in line:
                get_key_mgmt(ret_val, line)
            elif "PairwiseCiphers:" in line:
                get_key_mgmt(ret_val, line)
            elif "group_cipher=" in line:
                get_key_mgmt(ret_val, line)
            elif "WifiConfigStore - Log Begin" in line:
                break
            else:
                continue
    print ret_val
    for key in ret_val.keys():
        if ret_val[key] == "":
            ret_val[key] = None

    return ret_val

def get_connection_content(serial = None, service="wifi"):
    # services supported: "wifi", "wifip2p"
    if serial:
        adb_connection = connection_adb(serial = serial)
    else:
        adb_connection = connection_adb()
    wifi_conf = adb_connection.parse_cmd_output("dumpsys {}".format(service))
    return wifi_conf


from testlib.external.pyftpdlib.authorizers import DummyAuthorizer
from testlib.external.pyftpdlib.handlers import FTPHandler
from testlib.external.pyftpdlib.servers import FTPServer
import logging


class FTPRequestHandler(FTPHandler):

    def log_message(self, format_str, *args):
        pass

class FTP_Server(FTPServer):

    def server_start(self):
        # start ftp server
        #logging.basicConfig(filename='pyftpd.log', level=logging.CRITICAL)
        logging.disable(logging.CRITICAL)
        self.serve_forever()
        # there is no stop method for the ftp server.
        # for simplicity, in order to avoid using singletons and be consistent with http server stopping:
        # it should be stopped by shutting down the PID using the socket
        # with kill_socket(port_number, process_name) from local_steps

class FTPService(object):
    def __init__(self, ip_address, port_number):
        self.ip_address = ip_address
        self.port_number = port_number

    def start(self):

        createDaemon()

        # Instantiate a dummy authorizer for managing 'virtual' users
        self.authorizer = DummyAuthorizer()
        # Define a new user having full r/w permissions and a read-only
        # anonymous user
        os.chdir(".")
        self.authorizer.add_user('user', '12345', os.getcwd(), perm='elradfmwM')
        self.authorizer.add_anonymous(os.getcwd())
        os.getcwd()

        # Instantiate FTP handler class
        self.handler = FTPRequestHandler
        self.handler.authorizer = self.authorizer

        # Define a customized banner (string returned when client connects)
        self.handler.banner = "pyftpdlib based ftpd ready."

        # Instantiate FTP server class and listen on 0.0.0.0:2121
        self.address = (self.ip_address, int(self.port_number))
        self.server = FTP_Server((self.address), self.handler)

        # set a limit for connections
        self.server.max_cons = 256
        self.server.max_cons_per_ip = 5

        #self.server = FTP_Server((self.ip_address, self.port_number), FTPRequestHandler)
        self.process = multiprocessing.Process(target=self.server.server_start, args=[])
        self.process.start()
        sys.exit(0)

def ping_ipv6(ip, trycount = 2, target_percent = 50, timeout = 15, serial = None):

    """ Descriptions:
            Pings an ipv6 ip for a <trycount> times

        Returns True if the packet loss is less then or equal to the target_percent

        Usage:
            ping(serial = serial,
                   ip = "ipv6.google.com"
                   trycount = 30,
                   target_percent= 15)
    """
    if serial:
        adb_connection = connection_adb(serial = serial)
    else:
        adb_connection = connection_adb()

    ping_command = "ping6"
    command = ping_command + " -c " + str(trycount) + " " + str(ip)

    ping_data = adb_connection.run_cmd(command=command,
                                       timeout=timeout,
                                       mode="sync")
    ping_output = ping_data.stdout.read().strip()
    matcher = re.compile(r'(\d+)% packet loss')
    match = matcher.search(ping_output)
    if not match:
        print "Ping output : ", ping_output
        return False, None, ping_output

    percent = match.groups()[0]
    verdict = float(percent) <= target_percent

    return verdict, percent, ping_output

def ping(ip, trycount = 2, target_percent = 50, timeout = 15, serial = None):

    """ Descriptions:
            Pings the ip for a <trycount> times

        Returns True if the packet loss is less then or equal to the target_percent

        Usage:
            ping(serial = serial,
                   ip = "192.168.1.131" or "2001:1234:5678:9abc::1",
                   trycount = 30,
                   target_percent= 15)
    """
    if serial:
        adb_connection = connection_adb(serial = serial)
    else:
        adb_connection = connection_adb()

    if ":" in ip:
        ping_command = "ping6"
        command = ping_command + " -c " + str(trycount) + " " + str(ip) + "%wlan0"
    else:
        ping_command = "ping"
        command = ping_command + " -c " + str(trycount) + " " + str(ip)
    ping_data = adb_connection.run_cmd(command=command,
                                       timeout=timeout,
                                       mode="sync")
    ping_output = ping_data.stdout.read().strip()
    matcher = re.compile(r'(\d+)% packet loss')
    match = matcher.search(ping_output)
    if not match:
        print "Ping output : ", ping_output
        return False, None, ping_output

    percent = match.groups()[0]
    verdict = float(percent) <= target_percent

    return verdict, percent, ping_output

def start_ping(ip, trycount = 2, target_percent = 50, timeout = 15, serial = None):

    """ Descriptions:
            Pings the ip for a <trycount> times

        Returns True if the packet loss is less then or equal to the target_percent

        Usage:
            ping(serial = serial,
                   ip = "192.168.1.131" or "2001:1234:5678:9abc::1",
                   trycount = 30,
                   target_percent= 15)
    """
    if serial:
        adb_connection = connection_adb(serial = serial)
    else:
        adb_connection = connection_adb()

    if ":" in ip:
        ping_command = "ping6"
    else:
        ping_command = "ping"
    command = ping_command + " -c " + str(trycount) + " " + str(ip)
    ping_data = adb_connection.run_cmd(command=command,
                                       timeout=timeout,
                                       mode="sync")
    ping_output = ping_data.stdout.read().strip()
    matcher = re.compile(r'(\d+)% packet loss')
    match = matcher.search(ping_output)
    if not match:
        print "Ping output : ", ping_output
        return False, None, ping_output

    percent = match.groups()[0]
    verdict = float(percent) <= target_percent

    return verdict, percent, ping_output
def ip_to_int(ip):
    #conversion from IP string to integer
    val = 0
    i =0
    for s in ip.split('.'):
        val += int(s) * 256 ** (3 - i)
        i += 1
    return val

def int_to_ip(val):
    #conversion from integer to IP string
    octets = []
    for i in range(4):
        octets.append(str(val % 256))
        val = val >> 8
    return '.'.join(reversed(octets))

def get_ip_range(serial = None):
    content = get_connection_content(serial = serial)
    connection_info = get_connection_info(content)
    mask = connection_info["net_mask"]
    ip_address = connection_info["ip_address"]
    if ip_address:
        ip = ip_to_int(ip_address)
        netmask = (0xffffffff << (32-int(mask))) & 0xffffffff
        net_address = int_to_ip(ip & netmask)
        start = int_to_ip(ip_to_int(net_address) + 1)
        end = int_to_ip(ip_to_int(net_address) + pow(2,(32-int(mask))) - 1)
        return start + "_" + end
    else:
        print "The device does not have an IP Address. Check connection to AP!"
        return False


def gen_random_binary_file(name, size):
    """
    helper function used to generate a random file with a given name and size (in Bytes)
    """
    with open(name, 'wb') as fout:
        fout.write(os.urandom(size))


def create_file(file_name = "generated.bin" , file_size = "1024", file_path = "."):
    """
    more user friendly version of the above function
    creates a file with the provided name and given size, in KB
    """
    # first some clean-up... deleting files named like the test file, if any
    for file in os.listdir('.'):
        if fnmatch.fnmatch(file, file_name + "*"):
            os.remove(file)
    # file size is in Bytes = 1000 * size in KB, as used in download_file step
    byte_size = file_size * 1024
    gen_random_binary_file(file_path + "/" + file_name, byte_size)

def check_file_cmp(local_file, remote_file):
    """
    performs bitwise comparison between two files contents
    """
    if not filecmp.cmp(local_file, remote_file):
        return False
    else:
        return True

def check_file_md5(local_file, remote_file):
    """
    performs md5sum comparison between two files contents
    """
    local_md5 = hashlib.md5(open(local_file).read()).hexdigest()
    remote_md5 = hashlib.md5(open(remote_file).read()).hexdigest()
    if not local_md5 == remote_md5:
        return False
    else:
        return True

def get_ipv4_address():
    """
    Returns IP address(es) of current machine.
    """
    p = subprocess.Popen(["ifconfig"], stdout=subprocess.PIPE)
    ifc_resp = p.communicate()
    patt = re.compile(r'inet\s*\w*\S*:\s*(\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})')
    resp = patt.findall(ifc_resp[0])
    return resp

def select_ip(prefix, ip_list):
    """
    Out of a list of addresses, it returns the one the starts with a given prefix
    """
    ip_docker = None
    for address in ip_list:
        if str(prefix) in address:
            return address
        # in case of docker env
        if address != "127.0.0.1" and not ip_docker:
            ip_docker = address
    return ip_docker

def get_prefix(ip, mask):
    """
    Given an IP address and a mask, it returns the subnetwork part of the address
    """
    octets = ip.split(".")
    if mask == "8":
        return octets[0]
    if mask == "16":
        return ".".join(octets[:2])
    if mask == "24":
        return ".".join(octets[:3])
    else:
        return None

def get_host_ip(serial = None):
    """
    Determine de IP address of the current machine that is in the same subnet with the DUT
    """
    if serial:
        adb_connection = connection_adb(serial = serial)
    else:
        adb_connection = connection_adb()

    wifi_conf = get_connection_content(serial=serial)
    device_ip_mlink = get_connection_info(wifi_conf)['ip_address'] # ip address from mLink properties in dumpsys wifi output
    if device_ip_mlink:
        subnet_mask = get_connection_info(wifi_conf)['net_mask']
        host_addresses = get_ipv4_address()
        prefix = get_prefix(device_ip_mlink, subnet_mask)
        host_ip = select_ip(prefix, host_addresses)
        if host_ip != None:
            return host_ip
        else:
            print "Host IP - undetermined due to device connectivity issue: ", host_ip
            return None
    else:
        print "Host IP - undetermined due to device connectivity issue"
        net_cfg = get_netcfg_content(serial = serial)
        print "net cfg info : \n", net_cfg
        device_ip_netcfg = get_device_wlan0_ip(serial = serial).split('/')[0].split(':')[-1]
        print "Device ip netcfg way : ", device_ip_netcfg
        print "Device ip mlink way : ", device_ip_mlink
        return None


def get_connection_parameter(parameter, serial = None):
    """
    Returns the requested connection parameter.
    Possible parameter values:
               DHCP_server
               DNS_addresses
               Frequency
               Gateway
               Link_speed
               MAC
               SSID
               Security
               ip_address
               p2p_device_address
               state
               pairwise_cipher
               group_cipher
               net_mask
    """
    if serial:
        adb_connection = connection_adb(serial = serial)
    else:
        adb_connection = connection_adb()

    wifi_conf = get_connection_content(serial = serial)
    connection_params = get_connection_info(wifi_conf)
    if parameter in connection_params.keys():
        return connection_params[parameter]
    else:
        return None



def check_mac_format(mac):
    m = re.search(r'^[0-9a-f\:]+$', mac)
    if m != None:
        return True
    else:
        return False


def wait_p2p_state(uidevice, device_name, state="Connected", timeout = 5):
    """
    Waits for the given state to happen with timeout (at UI level).
    Possible parameter values:
               Connected
               Available
               Invited
    """

    wait_time = timeout
    while wait_time > 0:
        uidevice(text=state).wait.exists(timeout=1000)
        if uidevice(text=device_name).sibling(text=state):
            return True
        else:
            wait_time -= 1
            time.sleep(1)
    return False

def get_mWifiP2pInfo(ret_val, line):
    m = re.search(r'groupOwnerAddress: /([\.\d]+)', line)

    if m != None:
        ret_val['go_ip'] = m.group(1)
    return ret_val


def get_mNetworkInfo(ret_val, line):
    m = re.search(r'state: (\w+/\w+),', line)

    if m != None:
        ret_val['state'] = m.group(1)
    return ret_val

def get_netcfg_content(serial = None, platform = None):
    if serial:
        adb_connection = connection_adb(serial = serial)
    else:
        adb_connection = connection_adb()

    if platform:
        platform = platform
    else:
        platform = statics.Device(serial=serial)

    # netcfg of ifconfig output
    net_conf = adb_connection.parse_cmd_output(platform.get_interfaces_tool)
    return net_conf

def is_go_device(serial = None):
    if serial:
        adb_connection = connection_adb(serial = serial)
    else:
        adb_connection = connection_adb()

    if 'isGroupOwner: true' in adb_connection.parse_cmd_output("dumpsys {}".format('wifip2p')):
        return True
    else:
        return False


def p2p_get_connection_info(go_serial = None, slave_serial = None, platform = None):
    # imitialize the values
    ret_val = {'go_ip': None,
               'slave_ip': None,
               'group_name': None,
               'go_mac': None,
               'slave_mac': None,
               'go_name': None,
               'slave_name': None,
               'state': None,
               'group_state': None,
               'slave_interface_name': None,
               'go_interface_name': None,
               'go_serial': None,
               'slave_serial': None}

    if is_go_device(serial = go_serial):
        go_ser = go_serial
        slave_ser = slave_serial
    else:
        go_ser = slave_serial
        slave_ser = go_serial

    ret_val['go_serial'] = go_ser
    ret_val['slave_serial'] = slave_ser

    content1 = get_connection_content(serial = go_ser, service = "wifip2p")
    content2 = get_connection_content(serial = slave_ser, service = "wifip2p")

    # get values from GO device
    slave_device_found = False
    # first get connection state
    for line in content1.split("\n"):
        if "mNetworkInfo" in line:
            get_mNetworkInfo(ret_val, line)

    for line in content1.split("\n"):
        if "curState=" in line:
            ret_val['group_state'] = line.strip().split('=')[1]

        if ret_val['state'] == "CONNECTED/CONNECTED":
            if "mWifiP2pInfo" in line:
                get_mWifiP2pInfo(ret_val, line)
            elif "mGroup network:" in line:
                ret_val['group_name'] = line.split(':')[-1].strip()
            elif "deviceAddress:" in line:
                if not slave_device_found:
                    ret_val['go_mac'] = line.split(': ')[-1].strip()
                else:
                    ret_val['slave_mac'] = line.split(': ')[-1].strip()
            elif "Client: Device:" in line:
                slave_device_found = True
                ret_val['slave_name'] = line.split(':')[-1].strip()
            elif "interface:" in line:
                ret_val['go_interface_name'] = line.split(':')[-1].strip()


    # get values from slave device
    if ret_val['state'] == "CONNECTED/CONNECTED":
        for line in content2.split("\n"):
            if "GO: Device:" in line:
                ret_val['go_name'] = line.split(':')[-1].strip()
            elif "interface:" in line:
                ret_val['slave_interface_name'] = line.split(':')[-1].strip()

        # get slave IP
        if platform:
            platform = platform
        else:
            platform = statics.Device(serial=slave_ser)

        get_interfaces_tool_ip = platform.get_interfaces_tool

        content = get_netcfg_content(serial = slave_ser).split("\n")
        if get_interfaces_tool_ip == "ifconfig":
            for line in range(len(content)):
                if ret_val['slave_interface_name'] and ret_val['slave_interface_name'] in content[line]:
                    try:
                        ret_val['slave_ip'] = content[line + 1].split()[1].split(':')[1]
                    except IndexError:
                        time.sleep(10)
                        content = get_netcfg_content(serial=slave_ser).split("\n")
                        ret_val['slave_ip'] = content[line + 1].split()[1].split(':')[1]
        else:
            for line in content:
                if ret_val['slave_interface_name'] and ret_val['slave_interface_name'] in line:
                    ret_val['slave_ip'] = line.split()[2].split('/')[0]

    return ret_val


def p2p_get_connection_parameter(parameter, go_serial = None, slave_serial = None):
    """
    Returns the requested connection parameter.
    Possible parameter values:
               go_ip
               slave_ip
               group_name
               go_mac
               slave_mac
               go_name
               slave_name
               state
               group_state
               slave_interface_name
               go_interface_name
    """

    connection_params = p2p_get_connection_info(go_serial = go_serial, slave_serial = slave_serial)
    if parameter in connection_params.keys():
        return connection_params[parameter]
    else:
        return None

def p2p_get_interface_mac(get_if_tool, serial = None):
    """
    returns the MAC Address of the p2p0 interface
    """
    if serial:
        adb_connection = connection_adb(serial = serial)
    else:
        adb_connection = connection_adb()
    p2p0_line = adb_connection.parse_cmd_output(cmd = get_if_tool,
                                                 grep_for = "p2p").split()
    if len(p2p0_line) < 2:
        wlan0_line = adb_connection.parse_cmd_output(cmd = get_if_tool,
                                                 grep_for = "wlan0").split()
        global_mac_address = wlan0_line[-1]
        local_mac_address = get_local_mac_address(global_mac_address, serial)
        return local_mac_address
    return p2p0_line[-1]

def get_local_mac_address(global_address, serial):
    """
    given a global mac address as input,
    this function returns the locally administered value of a MAC address
    """
    if statics.Device(serial = serial).p2p_mac_mode == "locally administered bit":
        global_address_as_list = global_address.split(":") # splits the global mac address in a list of elements
        first_octet_hex_global = global_address_as_list[0] # takes the first octet
        first_octet_decimal_global = int(first_octet_hex_global, 16) # converts the value of the 1st octet from hex to decimal
        first_octet_decimal_local = first_octet_decimal_global + 2 # adds 2 to make set the locally administered bit
        first_octet_hex_local = hex(first_octet_decimal_local) # converts the 1st octet value to hex
        octet_local = first_octet_hex_local[2:] # removes "0x" prefix to prepare for local address construction
        local_address_as_list = global_address_as_list # initially, make the local address in list format identical to the global one
        local_address_as_list[0] = octet_local # replace the 1st octet of the address with the locally administered value
        local_address = ':'.join(local_address_as_list) # join the list elements with ":", obtaining the local adddress

    elif statics.Device(serial = serial).p2p_mac_mode == "increment":
        hex_global_address = global_address.replace(":", "")
        decimal_global_address = int(hex_global_address, 16)
        decimal_p2p_address = decimal_global_address + 1
        hex_p2p_address = hex(decimal_p2p_address)[2:]
        local_address = hex_p2p_address[:2] + ":" + hex_p2p_address[2:4] + ":" + hex_p2p_address[4:6] + ":" + hex_p2p_address[6:8] + ":" + hex_p2p_address[8:10] + ":" + hex_p2p_address[10:12]

    return local_address

def p2p_respond_to_connect_cli(master_name, serial = None, accept_connect = True,
                               known_device=False, device_info=None):
    """
    Description: Responds to wifi direct connection requests made through CLI.
    Clicks on "Accept" button from connection invide window, if it exists
    """
    if serial:
        adb_connection = connection_adb(serial = serial)
    else:
        adb_connection = connection_adb()
    uidevice = ui_device(serial = serial)
    if not known_device:
        response = device_info.wifi_p2p_connect_response_accept_btn_id
        if not accept_connect:
            response = device_info.wifi_p2p_connect_response_decline_btn_id
        uidevice(**response).wait.exists(timeout=20000)
        tries = 0
        while uidevice(**response).wait.exists(timeout=1000) and tries < 3:
            uidevice(**response).click.wait()
            tries += 1

def p2p_start_ftpdd(port_number = "20211", root_dir = "data/ftpdfiles/", serial = None):
    if serial:
        adb_connection = connection_adb(serial=serial)
    else:
        adb_connection = connection_adb()
    cmd = "/data/busybox/tcpsvd -vE 0.0.0.0" + " " + port_number + " " + "/data/busybox/ftpd" + " " + root_dir
    createDaemon()
    adb_connection.parse_cmd_output(cmd, timeout = 5)


def check_airplane_mode_on(serial = None):
    if serial:
        adb_connection = connection_adb(serial=serial)
    else:
        adb_connection = connection_adb()

    cmd = "dumpsys wifi | grep mAirplaneModeOn"
    out = adb_connection.parse_cmd_output(cmd=cmd)
    if "true" in out.strip():
        return True
    elif "false" in out.strip():
        return False
    else:
        return None


def check_wifi_state_on(serial = None):
    if serial:
        adb_connection = connection_adb(serial = serial)
    else:
        adb_connection = connection_adb()
    # below sleep time will help get proper result when there is a sudden
    # toggle in wifi power state
    time.sleep(1)
    cmd = "dumpsys wifi | grep Wi-Fi"
    out = adb_connection.parse_cmd_output(cmd=cmd)
    if "enabled" in out.strip():
        return True
    elif "disabled" in out.strip():
        return False
    else:
        return None


def get_peer_ip(go_serial, slave_serial, peer_type):
    ip_found = False
    count = 3
    while not ip_found and count > 0:
        conn_info = p2p_get_connection_info(go_serial = go_serial, slave_serial = slave_serial)
        count -= 1
        time.sleep(2)
        if conn_info[peer_type] != '0.0.0.0':
            break
    if conn_info[peer_type] == '0.0.0.0':
        print "peer type : ", peer_type
        print "connection info : ", conn_info
        print "go serial : ", go_serial
        print "slave serial : ", slave_serial
        netcfg_go = get_netcfg_content(serial = go_serial)
        netcfg_slave = get_netcfg_content(serial = slave_serial)
        print "netcfg go : \n", netcfg_go
        print "netcfg slave : \n", netcfg_slave
        dumpsys_p2p_go = get_connection_content(serial = go_serial, service="wifip2p")
        dumpsys_p2p_slave = get_connection_content(serial = slave_serial, service="wifip2p")
        print "dumpsys wifi p2p go : \n", dumpsys_p2p_go
        print "dumpsys wifi p2p slave : \n", dumpsys_p2p_slave
        return False
    return conn_info[peer_type]

def get_device_wlan0_ip(serial = None, platform = None):
    """
    returns the IP Address of the Wlan0 interface
    """
    if serial:
        adb_connection = connection_adb(serial = serial)
    else:
        adb_connection = connection_adb()

    if platform:
        platform = platform
    else:
        platform = statics.Device(serial=serial)

    get_interfaces_tool_ip = platform.get_interfaces_tool
    grep_for = "wlan0"
    if get_interfaces_tool_ip == "ifconfig":
        grep_for = "inet"

    wlan0_line = adb_connection.parse_cmd_output(cmd = platform.get_interfaces_tool,
                                                 grep_for = grep_for).split()
    return wlan0_line[-3]

def get_device_type(serial = None):
    """
    returns device type from getprop
    """
    if serial:
        adb_connection = connection_adb(serial = serial)
    else:
        adb_connection = connection_adb()
    product_name_line = adb_connection.parse_cmd_output(cmd = "getprop",
                                                 grep_for = "ro.product.device").split() # split getprop line into a list
    product_name = product_name_line[1] # this contains product name inside brackets, remove them below
    return product_name[1:-1]

def get_dut_ipv6_address(serial=None, static_ip=False):
    """
    Returns device ipv6 address from the ifconfig command output
    :param serial: the DUT serial
    :param static_ip: boolean, true if ip to get is static, false if it is dynamic
    :return: the ipv6 address from the output
    """
    if static_ip:
        grep_for_text = "global mngtmpaddr"
    else:
        grep_for_text = "global temporary"

    if serial:
        adb_connection = connection_adb(serial=serial)
    else:
        adb_connection = connection_adb()
    for i in range(5):
        ipv6_address_line = adb_connection.parse_cmd_output(cmd="ip -6 addr show wlan0",
                                                            grep_for=grep_for_text).split()

        if not ipv6_address_line:
            time.sleep(2)
        else:
            break
    ipv6_address = ipv6_address_line[1]
    if "/" in ipv6_address:
        ipv6_address = ipv6_address.split("/")[0]
    return ipv6_address


def is_valid_ipv6_address(address):
    """
    Checks if the supplied address is valid
    :param address: ipv6 address
    :return: True if valid, False otherwise
    """
    try:
        socket.inet_pton(socket.AF_INET6, address)
        return True
    except socket.error:
        return False
