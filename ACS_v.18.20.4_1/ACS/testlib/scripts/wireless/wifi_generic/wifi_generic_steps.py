#!/usr/bin/env python

#######################################################################
#
# @filename:    wifi_generic_steps.py.py
# @description: The inerface for Wifi steps
# @author:      aurel.constantin@intel.com
#
#######################################################################

from testlib.base.abstract.abstract_step import abstract_step
from testlib.base.abstract.mock_step import mock_step
from testlib.base.abstract import abstract_utils


USE_MODULE = abstract_utils.get_module("wifi_module")
if USE_MODULE == None:
    # default 'wifi_steps'
    USE_MODULE = "wifi_steps"


@abstract_step(use_module = USE_MODULE)
class clear_saved_networks():
    """ description:
            Clear all saved networks. Can remove a maximum of <max_entries> networks.


        optional parameters:
            max_entries - maximum entries to remove (default is 20)
    """
    pass

@abstract_step(use_module = USE_MODULE)
class set_wifi_frequency_band():
    """ description:
            Sets the option to use (scan / connect) only to APs using the desired frequency band

        required parameters:
            frequency_band = 2 / 5 / auto
        optional parameters:
            wait_time - for slow platforms - specify the relative amount of time to wait for certain operations
            verify_dumpsys -> checks if the scanned APs list complies with
                the applied setting (set to False to avoid GMINL-8045); default is True
    """
    pass

@abstract_step(use_module = USE_MODULE)
class add_network():
    """ description:
            Add a Wi-Fi network.

        required parameters:
            ssid
            security

        optional parameters:
            password
            EAP_method
            phase_2_auth
            CA_certificate
            identity
            anonymous_identity
            proxy
            proxy_pac_url
            proxy_hostname
            proxy_port
            proxy_bypass
            ip_settings
            ip_address
            gateway
            network_prefix_length
            dns1
            dns2
            valid_config
    """
    pass

@abstract_step(use_module=USE_MODULE)
class get_dut_ip_address():

    pass

@abstract_step(use_module = USE_MODULE)
class check_connection_info():
    """ description:
            Check connection information

        usage:
            wifi_steps.check_connection_info(SSID = "ddwrt", Security='WPA-PSK')()

            Use <parama_name>="None" to expect the setting not to be present on DUT.

            Example of possible values for all supported parameters:
            {'DHCP_server': '192.168.1.1',
             'DNS_addresses': '8.8.8.8,8.8.4.4',
             'Frequency': '2437MHz',
             'Gateway': '192.168.1.1',
             'Link_speed': '65Mbps',
             'MAC': 'dc:85:de:b9:5c:db',
             'SSID': 'Android Core QA',
             'Security': 'WPA2-PSK',
             'group_cipher': 'TKIP',
             'ip_address': '192.168.1.122',
             'p2p_device_address': 'de:85:de:b9:5c:db',
             'pairwise_cipher': 'CCMP',
             'state': 'CONNECTED/CONNECTED'}

    """
    pass

@abstract_step(use_module = USE_MODULE)
class wait_until_connected():
    """ description: Waits untill the device is connected to a wifi network.

        optional parameters:
        timeout (default is 30)
    """
    pass


@abstract_step(use_module = USE_MODULE)
class scan_and_check_ap():
    """ description:
            Check weather an AP SSID can be found by a Wi-Fi scan.

            NOTE: Carefull when looking for an AP SSID that is saved as it will
            show up even when the AP SSID is not visible or in range!

        required parameters:
            ap

        optional parameters:
            trycount (default is 10)
            should_exist (default is True)
    """
    pass

@abstract_step(use_module = USE_MODULE)
class set_airplane_mode():
    """ description:
            Enables / disabled Airplane mode.

        required parameters:
            state (ON/OFF)
    """
    pass


@abstract_step(use_module = USE_MODULE)
class set_am_state():
    """ description:
            Enables / disabled Airplane mode.

        required parameters:
            state (ON/OFF)
    """
    pass


@abstract_step(use_module = USE_MODULE)
class remove_network():
    """ description:
            Removes SSID(if known) from known AP list.

        required parameters:
            ap_name
    """
    pass

@abstract_step(use_module = USE_MODULE)
class connect_with_password():
    """ description:
            Connects to the given Wifi SSID with the given password. If the
            wrong_password switch is True, the step passes if the connection
            was unsuccessful.

        required parameters:
            ap_name, password

        optional parameters:
            wrong_password (default = False)
    """
    pass

@abstract_step(use_module = USE_MODULE)
class set_wifi():
    """ description:
            Turns Wifi on or off.

        optional parameters:
            state (ON/OFF) (default is ON)
    """
    pass

@abstract_step(use_module = USE_MODULE)
class download_file():
    """ No specifications for this step yet.
    """
    pass

@abstract_step(use_module = USE_MODULE)
class create_download_url():
    """ Creates download URL for WiFi traffic tests.
    """
    pass

@abstract_step(use_module = USE_MODULE)
class create_download_url_multiserver_ftp():

    pass

@abstract_step(use_module = USE_MODULE)
class check_file_integrity():
    """ No specifications for this step yet.
    """
    pass

@abstract_step(use_module = USE_MODULE)
class wifi_check_SSID_known():
    """ Check if the SSID is known.
    """
    pass

@abstract_step(use_module = USE_MODULE)
class wait_for_state():
    """ description: Waits untill the device has the wifi state given as parameter.

        optional parameters:
        state (default is 'CONNECTED/CONNECTED')
        timeout (default is 30)
    """
    pass

@abstract_step(use_module = USE_MODULE)
class check_wifi_state_disconnected():
    """ description:
            Checks the connection state of the Wifi is disconnected.

        usage:
            wifi_generic_steps.check_wifi_state_disconnected(ap_name = ddwrt_ap_name,
                                                     security=dut_security,
                                                     timeout=100,
                                                     serial = serial)()
    """
    pass

@abstract_step(use_module = USE_MODULE)
class modify_network():
    """ description: Modify an existing network.
    """
    pass

@abstract_step(use_module = USE_MODULE)
class ping_gateway():
    """ Pings the DUT gateway <trycount> times.
        Will fail if all pings fail.
    """
@abstract_step(use_module=USE_MODULE)
class ping_ipv6_gateway():
    """ Pings the DUT ipv6 gateway <trycount> times.
        Will fail if all pings fail.
    """

@abstract_step(use_module = USE_MODULE)
class ping_ipv6_ip():
    """ description: Pings the IP address in the string <trycount> times

    """
    pass


@abstract_step(use_module=USE_MODULE)
class ping_ipv6_gateway():
    """ Pings the DUT ipv6 gateway <trycount> times.
        Will fail if all pings fail.
    """

@abstract_step(use_module = USE_MODULE)
class ping_ip():
    """ Pings the IP for <trycount> times.
        It will fail if the packet loss is less then or equal to <target_percent>
    """

@abstract_step(use_module = USE_MODULE)
class find_available_ip():
    """
        returns the first IP that is available from a range of IPs
    """

@abstract_step(use_module = USE_MODULE)
class install_WIFI_certificate():
    """ description:
            Installs a certificate for WIFI. The certificate must be placed in /sdcard/
    """
    pass

@abstract_step(use_module = USE_MODULE)
class remove_certificates():
    """ description:
            Removes all certificates from DUT.
    """
    pass


@abstract_step(use_module = USE_MODULE)
class check_mac():
    """ Verifies the presence of the wifi MAC address.
    """

@abstract_step(use_module = USE_MODULE)
class check_network_information():
    """ Verifies network configuration on DUT.
    """

@abstract_step(use_module = USE_MODULE)
class p2p_check_connection_info():
    """ Check the WiFi Direct connection information.
    """


@abstract_step(use_module = USE_MODULE)
class p2p_connect_devices():
    """ Connects 2 devices.
    """
@abstract_step(use_module = USE_MODULE)
class p2p_connect_devices_prompt():
    """
        Annotation
    """

@abstract_step(use_module = USE_MODULE)
class p2p_connect_devices_cli():
    """ Connects 2 devices.
    """

@abstract_step(use_module = USE_MODULE)
class p2p_disconect():
    """ Disconnects a device.
    """


@abstract_step(use_module = USE_MODULE)
class p2p_disconect_all():
    """ Disconnects all conenctions.
    """


@abstract_step(use_module = USE_MODULE)
class p2p_forget_all_groups():
    """ Removes all groups.
    """


@abstract_step(use_module = USE_MODULE)
class p2p_forget_group():
    """ Removes a wifi direct group.
    """


@abstract_step(use_module = USE_MODULE)
class p2p_rename_device():
    """ Renames a p2p device.
    """


@abstract_step(use_module = USE_MODULE)
class p2p_scan_devices():
    """ Scans for p2p devices.
    """

@abstract_step(use_module = USE_MODULE)
class p2p_create_download_url():
    """ Creates download URL for p2p traffic tests.
    """

@abstract_step(use_module = USE_MODULE)
class p2p_kill_ftpd():
    """ Kills ftpd processes running on DUT.
    """

@abstract_step(use_module = USE_MODULE)
class p2p_start_ftpd():
    """ Starts .
    """

@abstract_step(use_module = USE_MODULE)
class kill_iperf_server():
    """ kills iperf process.
    """

@abstract_step(use_module = USE_MODULE)
class start_iperf_server():
    """ starts iperf process as server.
    """

@abstract_step(use_module = USE_MODULE)
class iperf_transfer():
    """ starts iperf process as client and starts data transfer.
    """

@abstract_step(use_module = USE_MODULE)
class check_lease_time():
    """ verify the lease time in logcat.
    """

@abstract_step(use_module = USE_MODULE)
class check_ipv6_address():
    """ check the ipv6 address was set correctly.
    """

@abstract_step(use_module = USE_MODULE)
class set_dut_ipv6():
    """ set a static ipv6 address on the dut.
    """

@abstract_step(use_module = USE_MODULE)
class configure_hotSpot():
    """ set the hotspot on the DUT
    """

@abstract_step(use_module = USE_MODULE)
class set_hotSpot():
    """ turn on the hotspot
    """

@abstract_step(use_module = USE_MODULE)
class ping_gateway_hotSpot():
    """ping the gatewy of the hotspot
    """
