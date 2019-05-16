"""

:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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
:summary: This file implements the LLP-specifics Localconnectivity UEcmd for Android phone
:since: 10/12/2014
:author: jfranchx
:author: mmaraci
"""

from acs_test_scripts.Device.UECmd.Imp.Android.KK.LocalConnectivity.LocalConnectivity import \
    LocalConnectivity as LocalConnectivityKK
from acs_test_scripts.Device.UECmd.Imp.Android.Common.LocalConnectivity.LocalConnectivity import ScanMode
from acs_test_scripts.Device.UECmd.UECmdDecorator import need
from acs_test_scripts.Device.UECmd.UECmdTypes import BluetoothDevice, BluetoothAdvertisedDevice
from ErrorHandling.AcsConfigException import AcsConfigException

import acs_test_scripts.Utilities.NetworkingUtilities as NetworkingUtil


class LocalConnectivity(LocalConnectivityKK):
    _BLUETOOTH_LE_MODULE = "acscmd.connectivity.bt.BluetoothLeModule"

    def __init__(self, phone):
        """
        Constructor
        """
        LocalConnectivityKK.__init__(self, phone)

    @need('bluetooth')
    def set_bt_discoverable(self, mode, timeout):
        """
        Sets the device discoverable.

        :type mode: str or int
        :param mode: can be ('noscan', 'none', SCAN_MODE_NONE) for no scan
          ('inquiry') for disoverable not connectable. Not supported by android
          ('page', 'connectable', SCAN_MODE_CONNECTABLE) for connectable
          ('both', SCAN_MODE_CONNECTABLE_DISCOVERABLE) to discoverable
        :type timeout: int
        :param timeout: can be 0 for Never,
                               120 for 2 minutes
                               300 for 5 minutes
                               3600 for 1 hour

        :return: None
        """
        if mode in ('noscan', 'none', ScanMode.SCAN_MODE_NONE):
            mode = ScanMode.SCAN_MODE_NONE
        elif mode in ('page', 'connectable', ScanMode.SCAN_MODE_CONNECTABLE):
            mode = ScanMode.SCAN_MODE_CONNECTABLE
        elif mode in ('both', ScanMode.SCAN_MODE_CONNECTABLE_DISCOVERABLE):
            mode = ScanMode.SCAN_MODE_CONNECTABLE_DISCOVERABLE
        else:
            self._logger.error("set_bt_discoverable: Parameter mode %s is not supported" % mode)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Parameter mode is not valid!")

        if timeout not in (0, '0', None):
            self._logger.warning(
                "Lollipop doesn't support visibility timeout on Bluetooth. Timeout %s will not be used")

        if mode == ScanMode.SCAN_MODE_NONE:
            self._logger.info("Set BT scan mode to NONE")
            # Return on home page to disable Bluetooth visibility
            phone_api = self._device.get_uecmd("PhoneSystem")
            phone_api.home_page_menu()
            method = "setScanMode"
            args = "--es mode %d --es timeout %d" % (mode, timeout)
            self._internal_exec_v2(self._BLUETOOTH_MODULE, method, args, is_system=True)

        elif mode == ScanMode.SCAN_MODE_CONNECTABLE:
            self._logger.info("Set BT scan mode to CONNECTABLE")
            method = "setScanMode"
            args = "--es mode %d --es timeout %d" % (mode, timeout)

            self._internal_exec_v2(self._BLUETOOTH_MODULE, method, args, is_system=True)

        elif mode == ScanMode.SCAN_MODE_CONNECTABLE_DISCOVERABLE:
            self._logger.info("Set BT scan mode to DISCOVERABLE")
            # Open Bluetooth menu to enable Bluetooth visibility
            self.bluetooth_menu_settings()

        self._logger.info("Set BT discoverable - success")

    #
    #######################################################################
    #
    # BLE Central
    #
    #######################################################################
    #

    def millis_to_seconds(self, mills):
        """

        :param mills: String as sent by the test step parser
        :return:
        """
        return mills/1000

    @need("bluetooth")
    def ble_scan_devices(self, timeout):
        """
        This method searches for BLE active devices using the API 21 level and onwards
        rtype: list of BluetoothDevice object
        :return: list of found adapters (name/address)
        """
        method = "scanLe"
        args = "--ei cmdTimeout %s" % timeout
        device_list = []

        output = self._internal_exec_multiple_v2(self._BLUETOOTH_LE_MODULE, method, args,
                                                 timeout=self.millis_to_seconds(int(timeout))+5, is_system=True)

        for element in output:
            btdev = BluetoothDevice()
            if "Address" in element:
                btdev.address = element["Address"]
            if "Name" in element:
                btdev.name = element["Name"]
            device_list.append(btdev)

        for device in device_list:
            self._logger.debug("Device name: " + device.name + " address: " + device.address)

        return device_list

    @need('bluetooth')
    def get_ble_scan_mode(self):
        """
        Retrieves the Bluetooth scan mode.

        :rtype: str
        :return: scan mode (both, noscan, inquiry)
        """
        method = "getLeScanMode"

        mode = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, is_system=True)
        return mode

    @need('bluetooth')
    def ble_find_device_with_generic_le_scan(self, remote_device_info, timeout):
        """
        Looks for a Bluetooth device.

        :type remote_device_info: str
        :param remote_device_info: Name or address of the device

        :rtype: boolean
        :return: True if found , False otherwise
        """
        bt_found = False
        bt_devices_list = self.ble_scan_devices(timeout)

        if NetworkingUtil.is_valid_mac_address(remote_device_info):
            # remote_device_info is a bd address
            for device in bt_devices_list:
                if (str(device.address).upper() == str(remote_device_info).upper() or
                            str(device.name).upper() == str(remote_device_info).upper()):
                    bt_found = True
                    self._logger.info("Device %s found" % remote_device_info)
                    break
        else:
            # remote_device_info is not a valid bd address, maybe a name !
            for device in bt_devices_list:
                if str(device.name) == str(remote_device_info):
                    bt_found = True
                    self._logger.info("Device %s found" % remote_device_info)
                    break

        if not bt_found:
            self._logger.info("Device " + remote_device_info +
                              " is not in range !")
        return bt_found

    @need("bluetooth")
    def ble_scan_filter_address(self, remote_device_address, scan_mode, timeout):
        """
        BLE Filtered scanning for a device address.

        :rtype: str
        :return: Found Bluetooth Device
        """
        bt_found = False
        btdev = BluetoothDevice()

        method = "filterLeAddress"
        args = "--es address %s --ei scanMode %s --ei cmdTimeout %s" % (remote_device_address, scan_mode, timeout)

        output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args,
                                        timeout=self.millis_to_seconds(int(timeout))+5, is_system=True)

        if NetworkingUtil.is_valid_mac_address(remote_device_address):
            # remote_device_info is a bd address
            for device in output:
                if (str(device.address).upper() == str(remote_device_address).upper() or
                            str(device.name).upper() == str(remote_device_address).upper()):
                    bt_found = True
                    self._logger.info("Device %s found" % remote_device_address)

        if "Address" in output:
            btdev.address = output["Address"]
        if "Name" in output:
            btdev.name = output["Name"]

        self._logger.info("Device name: " + btdev.name + " address: " + btdev.address)

        if not bt_found:
            self._logger.info("Device " + remote_device_address +
                              " is not in range or could not be filtered!")
            return False
        else:
            return btdev

    @need("bluetooth")
    def ble_scan_filter_name(self, remote_device_name, scan_mode, timeout):
        """
        BLE Filtered scanning for a device address.

        :rtype: str
        :return: Found Bluetooth Device
        """
        btdev = BluetoothDevice()

        method = "filterLeName"
        args = "--es name %s --ei scanMode %s --ei cmdTimeout %s" % (remote_device_name, scan_mode, timeout)

        output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args,
                                        timeout=self.millis_to_seconds(int(timeout))+5, is_system=True)

        if "Address" in output:
            btdev.address = output["Address"]
        if "Name" in output:
            btdev.name = output["Name"]
        # btdev.init(output)

        return btdev

    @need("bluetooth")
    def ble_scan_filter_manufacturer_data(self, remote_advertiser_manufacturer_id, scan_mode, timeout):
        """
        BLE Filtered scanning for a device address.

        :rtype: str
        :return: Found Bluetooth Device
        """
        btdev = BluetoothDevice()

        method = "filterLeManData"
        args = "--ei manId %s --ei scanMode %s --ei cmdTimeout %s" % (remote_advertiser_manufacturer_id, scan_mode,
                                                                      timeout)

        output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args,
                                        timeout=self.millis_to_seconds(int(timeout))+5, is_system=True)

        if "Address" in output:
            btdev.address = output["Address"]
        if "Name" in output:
            btdev.name = output["Name"]
        return btdev

    @need("bluetooth")
    def ble_scan_filter_service_uuid(self, service_uuid, scan_mode, timeout):
        """
        BLE Filtered scanning for a device address.

        :rtype: str
        :return: Found Bluetooth Device
        """
        btdev = BluetoothAdvertisedDevice()

        method = "filterLeServiceUuid"
        args = "--es filterServUuid %s --ei scanMode %s --ei cmdTimeout %s" % (service_uuid, scan_mode, timeout)

        output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args,
                                        timeout=self.millis_to_seconds(int(timeout))+5, is_system=True)

        self._logger.debug("My output is: " + str(output))
        # if "Address" in output:
        #     btdev.address = output["Address"]
        # if "Name" in output:
        #     btdev.name = output["Name"]
        btdev.init(output)
        self._logger.info("Device info is: " + btdev.to_string())
        return btdev

    @need("bluetooth")
    def ble_scan_filter_service_data(self, service_uuid, scan_mode, timeout):
        """
        BLE Filtered scanning for a device address.

        :rtype: str
        :return: Found Bluetooth Device
        """
        btdev = BluetoothAdvertisedDevice()

        method = "filterLeServiceData"
        args = "--es filterServUuid %s --ei scanMode %s --ei cmdTimeout %s" % (service_uuid, scan_mode, timeout)

        output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args,
                                        timeout=self.millis_to_seconds(int(timeout))+5, is_system=True)

        self._logger.debug("My output is: " + str(output))
        # if "Address" in output:
        #     btdev.address = output["Address"]
        # if "Name" in output:
        #     btdev.name = output["Name"]
        btdev.init(output)
        self._logger.info("Device info is: " + btdev.to_string())
        return btdev

    @need("bluetooth")
    def ble_scan_for_unfiltered_advertisement(self, timeout):
        """

        :return:
        """
        device_list = []
        method = "scanForAdvertising"
        args = "--ei cmdTimeout %s" % timeout

        output = self._internal_exec_multiple_v2(self._BLUETOOTH_LE_MODULE, method, args,
                                                 timeout=self.millis_to_seconds(int(timeout))+5, is_system=True)

        # check that the result is success
        for element in output:
            btadv = BluetoothAdvertisedDevice()
            # if "Address" in element:
            #     btadv.address = element["Address"]
            # if "Name" in element:
            #     btadv.name = element["Name"]
            btadv.init(element)
            device_list.append(btadv)

        return device_list

    @need("bluetooth")
    def ble_scan_advertising_filter_address(self, remote_device_address, scan_mode, timeout):
        """
        BLE Filtered scanning for a device address.

        :rtype: str
        :return: Found Bluetooth Device
        """
        btdev = BluetoothAdvertisedDevice()

        method = "filterAddressLeAdvertising"
        args = "--es address %s --ei scanMode %s --ei cmdTimeout %s" % (remote_device_address, scan_mode, timeout)

        output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args,
                                        timeout=self.millis_to_seconds(int(timeout))+5, is_system=True)

        # if "Address" in output:
        #     btdev.address = output["Address"]
        # if "Name" in output:
        #     btdev.name = output["Name"]
        btdev.init(output)

        return btdev

    @need("bluetooth")
    def ble_scan_advertising_power_levels(self, scan_mode, timeout):
        """
        BLE Filtered scanning for a device address.

        :rtype: str
        :return: Found Bluetooth Device
        """
        # btdev = BluetoothAdvertisedDevice()
        device_list = []

        method = "filterLePowerLevels"
        args = "--ei scanMode %s --ei cmdTimeout %s" % (scan_mode, timeout)

        output = self._internal_exec_multiple_v2(self._BLUETOOTH_LE_MODULE, method, args,
                                                 timeout=self.millis_to_seconds(int(timeout))+5, is_system=True)

        # check that the result is success
        for element in output:
            self._logger.debug("Element in output is: ")
            self._logger.debug(element)
            btdev = BluetoothAdvertisedDevice()
            btdev.init(element)
            device_list.append(btdev)

        return device_list

    @need("bluetooth")
    def ble_start_beacon_observer(self, scan_mode, timeout):
        """

        :return:
        """

        device_list = []
        method = "startObserver"
        args = "--ei scanMode %s --ei cmdTimeout %s" % (scan_mode, timeout)

        output = self._internal_exec_multiple_v2(self._BLUETOOTH_LE_MODULE, method, args,
                                                 timeout=self.millis_to_seconds(int(timeout))+5, is_system=True)

        # check that the result is success
        for element in output:
            self._logger.debug("Element in output is: ")
            self._logger.debug(element)
            btdev = BluetoothAdvertisedDevice()
            btdev.init(element)
            device_list.append(btdev)
        return device_list

    @need("bluetooth")
    def ble_stop_beacon_observer(self):
        """

        :return:
        """
        method = "stopObserver"

        output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, is_system=True)

        # check that the result is success


    #
    #######################################################################
    #
    # BLE Peripheral
    #
    #######################################################################
    #

    @need("bluetooth")
    def ble_check_peripheral_advertising_supported(self):
        """

        :return:
        """
        method = "checkBlePeripheralSupported"
        output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, is_system=True)

        # check that the result is success

    @need("bluetooth")
    def ble_start_beacon_broadcast(self, tx_power_level, advertise_mode, is_connectable, advertise_timeout):
        """

        :return:
        """
        method = "startBroadcast"
        args = "--ei powLevel %s --ei advertiseMode %s --ez isConnectable %s --ei " \
               "advertiseTimeout %s" % (tx_power_level, advertise_mode, is_connectable, advertise_timeout)

        if advertise_timeout == "0":
            output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args, is_system=True)
        else:
            output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args,
                                            timeout=self.millis_to_seconds(int(advertise_timeout))+5, is_system=True)

        # check that the result is success

    @need("bluetooth")
    def ble_stop_beacon_broadcast(self):
        """

        :return:
        """
        method = "stopBroadcast"

        output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, is_system=True)

        # check that the result is success

    @need("bluetooth")
    def ble_start_advertising(self, tx_power_level, advertise_mode, advertise_code, is_connectable, advertise_timeout):
        """

        :return:
        """
        method = "startAdvertising"
        args = "--ei powLevel %s --ei advertiseMode %s --ei advertiseCode %s --ez isConnectable %s --ei " \
               "advertiseTimeout %s" \
               % (tx_power_level, advertise_mode, advertise_code, is_connectable, advertise_timeout)

        if advertise_timeout == "0":
            output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args, is_system=True)
        else:
            output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args,
                                            timeout=self.millis_to_seconds(int(advertise_timeout))+5, is_system=True)
        # check that the result is success

    @need("bluetooth")
    def ble_stop_advertising(self, advertise_code):
        """

        :return:
        """
        method = "stopAdvertising"
        args = "--ei advertiseCode %s" % advertise_code

        output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args, is_system=True)
        # check that the result is success

    #
    #######################################################################
    #
    # BLE GATT Server
    #
    #######################################################################
    #

    @need("bluetooth")
    def ble_gatt_advertise_server(self, tx_power_level, advertise_mode, advertise_timeout, gatt_server_type):
        """

        :return:
        """
        method = "advertiseGattServer"
        args = "--es gattServerType %s --ei powLevel %s --ei advertiseMode %s --ei  advertiseTimeout %s" \
               % (gatt_server_type, tx_power_level, advertise_mode, advertise_timeout)

        if advertise_timeout == "0":
            output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args, is_system=True)
        else:
            output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args,
                                            timeout=self.millis_to_seconds(int(advertise_timeout))+5, is_system=True)
        # check that the result is success
            return output


    @need("bluetooth")
    def ble_gatt_stop_server(self):
        """

        :return:
        """
        method = "gattStopAdvertisingServer"

        output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, is_system=True)
        # check that the result is success
        return output


    @need("bluetooth")
    def ble_gatt_start_server(self, gatt_server_type):
        """

        :return:
        """
        method = "startGattServer"
        args = "--es gattServerType %s " % gatt_server_type

        output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args, is_system=True)
        # check that the result is success
        return output


    @need("bluetooth")
    def ble_gatt_stop_server_no_advertising(self):
        """

        :return:
        """
        method = "gattStopServer"

        output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, is_system=True)
        # check that the result is success
        return output

    @need("bluetooth")
    def ble_gatt_server_add_service(self, tx_power_level, advertise_mode, advertise_timeout):
        """

        :return:
        """
        method = "gattServerAddService"
        args = "--ei powLevel %s --ei advertiseMode %s --ei  advertiseTimeout %s" \
               % (tx_power_level, advertise_mode, advertise_timeout)

        if advertise_timeout == "0":
            output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args, is_system=True)
        else:
            output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args,
                                            timeout=self.millis_to_seconds(int(advertise_timeout))+5, is_system=True)
        # check that the result is success
        return output

    @need("bluetooth")
    def ble_gatt_server_add_characteristic(self, tx_power_level, advertise_mode, advertise_timeout):
        """

        :return:
        """
        method = "gattServerAddCharacteristic"
        args = "--ei powLevel %s --ei advertiseMode %s --ei  advertiseTimeout %s" \
               % (tx_power_level, advertise_mode, advertise_timeout)

        if advertise_timeout == "0":
            output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args, is_system=True)
        else:
            output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args,
                                            timeout=self.millis_to_seconds(int(advertise_timeout))+5, is_system=True)
        # check that the result is success
        return output

    @need("bluetooth")
    def ble_gatt_server_add_descriptor(self, tx_power_level, advertise_mode, advertise_timeout):
        """

        :return:
        """
        method = "gattServerAddDescriptor"
        args = "--ei powLevel %s --ei advertiseMode %s --ei  advertiseTimeout %s" \
               % (tx_power_level, advertise_mode, advertise_timeout)

        if advertise_timeout == "0":
            output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args, is_system=True)
        else:
            output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args,
                                            timeout=self.millis_to_seconds(int(advertise_timeout))+5, is_system=True)
        # check that the result is success
        return output

    @need("bluetooth")
    def ble_gatt_server_read_characteristic_callback(self, tx_power_level, advertise_mode, advertise_timeout):
        """

        :return:
        """
        method = "gattServerReadCharacteristicCallback"
        args = "--ei powLevel %s --ei advertiseMode %s --ei  advertiseTimeout %s" \
               % (tx_power_level, advertise_mode, advertise_timeout)

        if advertise_timeout == "0":
            output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args, is_system=True)
        else:
            output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args,
                                            timeout=self.millis_to_seconds(int(advertise_timeout))+5, is_system=True)
        # check that the result is success
        return output

    @need("bluetooth")
    def ble_gatt_server_write_characteristic_callback(self, tx_power_level, advertise_mode, advertise_timeout):
        """

        :return:
        """
        method = "gattServerWriteCharacteristicCallback"
        args = "--ei powLevel %s --ei advertiseMode %s --ei  advertiseTimeout %s" \
               % (tx_power_level, advertise_mode, advertise_timeout)

        if advertise_timeout == "0":
            output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args, is_system=True)
        else:
            output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args,
                                            timeout=self.millis_to_seconds(int(advertise_timeout))+5, is_system=True)
        # check that the result is success
        return output

    @need("bluetooth")
    def ble_gatt_server_read_descriptor_callback(self, tx_power_level, advertise_mode, advertise_timeout):
        """

        :return:
        """
        method = "gattServerReadDescriptorCallback"
        args = "--ei powLevel %s --ei advertiseMode %s --ei  advertiseTimeout %s" \
               % (tx_power_level, advertise_mode, advertise_timeout)

        if advertise_timeout == "0":
            output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args, is_system=True)
        else:
            output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args,
                                            timeout=self.millis_to_seconds(int(advertise_timeout))+5, is_system=True)
        # check that the result is success
        return output

    @need("bluetooth")
    def ble_gatt_server_write_descriptor_callback(self, tx_power_level, advertise_mode, advertise_timeout):
        """

        :return:
        """
        method = "gattServerWriteDescriptorCallback"
        args = "--ei powLevel %s --ei advertiseMode %s --ei  advertiseTimeout %s" \
               % (tx_power_level, advertise_mode, advertise_timeout)

        if advertise_timeout == "0":
            output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args, is_system=True)
        else:
            output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args,
                                            timeout=self.millis_to_seconds(int(advertise_timeout))+5, is_system=True)
        # check that the result is success
        return output

    @need("bluetooth")
    def ble_gatt_server_reliable_write_callback(self, tx_power_level, advertise_mode, advertise_timeout):
        """

        :return:
        """
        method = "gattServerReliableWriteCallback"
        args = "--ei powLevel %s --ei advertiseMode %s --ei  advertiseTimeout %s" \
               % (tx_power_level, advertise_mode, advertise_timeout)

        if advertise_timeout == "0":
            output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args, is_system=True)
        else:
            output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args,
                                            timeout=self.millis_to_seconds(int(advertise_timeout))+5, is_system=True)
        # check that the result is success
        return output


    @need("bluetooth")
    def ble_gatt_server_interaction_no_advertising(self, gatt_server_operation):
        """

        :return:
        """

        method = "gattServerInteractionNoAdvertising"

        args = "--ei gattServerOperation %s" % gatt_server_operation

        output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args, is_system=True)
        # check that the result is success
        return output
    #
    #######################################################################
    #
    # BLE GATT Client
    #
    #######################################################################
    #

    @need("bluetooth")
    def ble_gatt_client_connect_to_server(self, address):
        """

        :param address:
        :return:
        """
        method = "connectGatt"
        args = "--es address %s" % address
        if not NetworkingUtil.is_valid_mac_address(address):
            return

        output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args, is_system=True)

        # check that the result is success
        return output

    @need("bluetooth")
    def ble_gatt_client_disconnect_from_server(self, address):
        """

        :return:
        """
        method = "disconnectGatt"
        args = "--es address %s" % address

        if not NetworkingUtil.is_valid_mac_address(address):
            return

        output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args, is_system=True)

        # check that the result is success
        return output

    @need("bluetooth")
    def ble_gatt_client_discover_services(self, address):
        """

        :return:
        """
        method = "discoverGattServices"
        args = "--es address %s" % address

        if not NetworkingUtil.is_valid_mac_address(address):
            return

        output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args, is_system=True)

        # check that the result is success
        return output

    @need("bluetooth")
    def ble_gatt_client_read_characteristic(self, address):
        """

        :return:
        """
        method = "readGattCharacteristic"
        args = "--es address %s" % address

        if not NetworkingUtil.is_valid_mac_address(address):
            return

        output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args, is_system=True)

        # check that the result is success
        return output

    @need("bluetooth")
    def ble_gatt_client_write_characteristic(self, address, write_characteristic_value):
        """

        :return:
        """
        method = "writeGattCharacteristic"
        args = "--es address %s --es writeCharacteristicValue %s" % (address, write_characteristic_value)

        if not NetworkingUtil.is_valid_mac_address(address):
            return

        output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args, is_system=True)

        # check that the result is success
        return output

    @need("bluetooth")
    def ble_gatt_client_read_descriptor(self, address):
        """

        :return:
        """
        method = "readGattDescriptor"
        args = "--es address %s" % address

        if not NetworkingUtil.is_valid_mac_address(address):
            return

        output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args, is_system=True)

        # check that the result is success
        return output

    @need("bluetooth")
    def ble_gatt_client_write_descriptor(self, address, write_descriptor_value):
        """

        :return:
        """
        method = "writeGattDescriptor"
        args = "--es address %s --es writeDescriptorValue %s" % (address, write_descriptor_value)

        if not NetworkingUtil.is_valid_mac_address(address):
            return

        output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args, is_system=True)

        # check that the result is success
        return output

    @need("bluetooth")
    def ble_gatt_client_reliable_write(self, address, reliable_write_value):
        """

        :return:
        """
        method = "gattReliableWrite"
        args = "--es address %s --es reliableWriteValue %s" % (address, reliable_write_value)

        if not NetworkingUtil.is_valid_mac_address(address):
            return

        output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args, is_system=True)

        # check that the result is success
        return output

    @need("bluetooth")
    def ble_gatt_client_read_rssi(self, address):
        """

        :return:
        """
        method = "gattReadRssi"
        args = "--es address %s" % address

        if not NetworkingUtil.is_valid_mac_address(address):
            return

        output = self._internal_exec_v2(self._BLUETOOTH_LE_MODULE, method, args, is_system=True)

        # check that the result is success
        return output

    @need("bluetooth")
    def ble_gatt_client_read_notification(self, address, which_characteristic, read_timeout):
        """

        :param address:
        :param which_characteristic:
        :param read_timeout:
        :return: a list of the notification values received
        """

        device_list = []

        method = "readGattNotification"
        args = "--es address %s --ei whichCharacteristic %s --ei  cmdTimeout %s" \
               % (address, which_characteristic, read_timeout)

        output = self._internal_exec_multiple_v2(self._BLUETOOTH_LE_MODULE, method, args,
                                                 timeout=self.millis_to_seconds(int(read_timeout))+5, is_system=True)

        # check that the result is success
        for element in output:
            self._logger.debug("Value in Received Notification: " + str(element['Received Notification Value']))
            device_list.append(element['Received Notification Value'])

        return device_list

    #
    #######################################################################
    #
    # Auxiliary methods
    #
    #######################################################################
    #
