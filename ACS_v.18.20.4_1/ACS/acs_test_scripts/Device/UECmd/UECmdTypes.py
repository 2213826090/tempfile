"""
Global types used in UECmd layer

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
:summary: This script implements the interfaces to unitary local
connectivity features
:since: 19/07/2010
:author: asebbane, vgombert
"""

import UtilitiesFWK.Utilities as Util

UECmdCat = {
    "VoiceCall": "Communication.VoiceCall",
    "SmsMessaging": "Communication.SmsMessaging",
    "Modem": "Communication.Modem",
    "CellBroadcastMessaging": "Communication.CellBroadcastMessaging",
    "SipCall": "Communication.SipCall",
    "MmsMessaging": "Communication.MmsMessaging",
    "SimCard": "Telephony.SimCard",
    "VideoCallSkype": "Communication.VideoCallSkype",
    "AudioAlgoController": "Communication.AudioAlgoController",

    "EnergyManagement": "EnergyManagement.EnergyManagement",
    "Aplog": "SysDebug.Aplog",
    "LocalConnectivity": "LocalConnectivity.LocalConnectivity",

    "Location": "Location.Location",

    "PhoneSystem": "Misc.PhoneSystem",
    "Watcher": "Misc.Watcher",
    "PsiRecorder": "Misc.PsiRecorder",

    "Camera": "Multimedia.Camera",
    "IntelRefCam": "Multimedia.IntelRefCam",
    "IntelRefCam2": "Multimedia.IntelRefCam2",
    "Multimedia": "Multimedia.Multimedia",
    "Image": "Multimedia.Image",
    "Audio": "Multimedia.Audio",
    "AudioHost": "Multimedia.AudioHost",
    "AudioRecorder": "Multimedia.AudioRecorder",
    "AudioRecorderHost": "Multimedia.AudioRecorderHost",
    "Video": "Multimedia.Video",
    "VideoRecorder": "Multimedia.VideoRecorder",

    "Networking": "Networking.Networking",
    "P2PClientCLI": "Networking.WifiP2PClientCLI",
    "P2PSupplicantCLI": "Networking.WifiP2PSupplicantCLI",
    "WifiDirect": "Networking.WifiDirect",
    "Ftp": "Networking.Ftp",

    "Contacts": "Contacts.Contacts",
    "File": "System.File",
    "Sensor": "System.Sensor",
    "System": "System.System",
    "AppMgmt": "System.AppMgmt",
    "ModemFlashing": "System.ModemFlashing",
    "KeyEvent": "System.KeyEvent",
    "SleepMode": "System.SleepMode",
    "Residencies": "System.Residencies",
    "Vibrator": "System.Vibrator",

    "Ui": "Ui.Ui",
    "PyUiAutomator": "Ui.PyUiAutomator",

    "SysDebug": "SysDebug.Manager",
    "Display": "Display.Display",
    "GPIO": "LowSpeedIO.GPIO",
    "I2C": "LowSpeedIO.I2C",
    "SPI": "LowSpeedIO.Spi"
}

class Measure:

    """
    Structure for generic measure objects
    """

    def __init__(self):
        self.value = float(0)
        self.units = "Empty"

SIP_CALL_STATE = Util.enum('DEREGISTERING', 'IN_CALL', 'INCOMING_CALL',
                           'INCOMING_CALL_ANSWERING', 'NOT_DEFINED', 'OUTGOING_CALL',
                           'OUTGOING_CALL_CANCELING', 'OUTGOING_CALL_RING_BACK',
                           'PINGING', 'READY_TO_CALL', 'REGISTERING')
VOICE_CALL_STATE = Util.enum('ACTIVE', 'ON_HOLD', 'DIALING', 'ALERTING',
                             'INCOMING', 'WAITING', 'DISCONNECTED', 'UNKNOWN',
                             'NOCALL')
VOICE_CALL_TYPE = Util.enum('OUTGOING', 'INCOMING', 'MISSED')
# Modem
DATA_STATE = Util.enum('CONNECTING', 'CONNECTED', 'SUSPENDED', 'DISCONNECTED')
SIM_ROLE = Util.enum('PRIMARY', 'SECONDARY')
BPLOG_LOC = Util.enum('SDCARD', 'EMMC', 'DATA')

AUDIO_STATE = Util.enum('IN_CALL', 'IN_COMMUNICATION', 'RINGTONE', 'NORMAL', 'INVALID', 'CURRENT')
AUTO_CONNECT_STATE = Util.enum('on', 'off')

XFER_DIRECTIONS = Util.enum('DL', 'UL')

EMERGENCY_NUMBERS_LIST = ("112", "911", "000", "08", "110", "118", "119", "999", "18", "17", "15", "114", "115")
BT_BOND_STATE = Util.enum('BOND_BONDED', 'BOND_BONDING', 'BOND_NONE')
BT_STATE = Util.enum('STATE_OFF', 'STATE_ON', 'STATE_TURNING_OFF',
                     'STATE_TURNING_ON', 'STATE_CONNECTED', 'STATE_CONNECTING',
                     'STATE_DISCONNECTED', 'STATE_DISCONNECTING')
BT_SCAN_MODE = Util.enum('SCAN_MODE_NONE', 'SCAN_MODE_CONNECTABLE',
                         'SCAN_MODE_CONNECTABLE_DISCOVERABLE')
BT_PINVARIANT = Util.enum('PIN', 'PASSKEY', 'REPLY', 'NONE', 'ERROR')

# All MMS types authorized.
MMS_ATTACH_TYPE_LIST = ("PICTURE", "TEXT", "AUDIO", "VIDEO")

MSG_DB_PATH = "/data/data/com.android.providers.telephony/databases/mmssms.db"
MSG_DB_PATH_N = "/data/user_de/0/com.android.providers.telephony/databases/mmssms.db"

PACKAGE_STATE = Util.enum('ENABLED', 'DISABLED', 'ALL', 'UNKNOWN')

"""
A L[list] containing all possible integer representations
of network type.
"""
NETWORK_TYPES = [
    # common      Android      Windows
    "UNKNOWN",  # Unknown
    "GPRS",  #   GPRS - GSM       0x00000001, "GPRS"
    "EGPRS",  #  EDGE             0x00000002, "EDGE"
    "UMTS",  #   UMTS             0x00000004, "UMTS"
    "CDMA",  #   CDMA
    "EVDO_0",  # EVDO rev. 0      0x00020000, "1X EVDO"
    "EVDO_A",  # EVDO rev. A      0x00040000, "1XEVD_REVA"
    "1xRTT",  #  1xRTT            0x00010000, "1XRTT"
    "HSDPA",  #  HSDPA            0x00000008, "HSDPA"
    "HSUPA",  #  HSUPA            0x00000010, "HSUPA"
    "HSPA",  #   HSPA             0x00000018, "HSPA"
    "IDEN",  #   iDen
    "EVDO_B",  # EVDO rev. B      0x00200000, "1XEVDO_REVB"
    "LTE",  #    LTE              0x00000020, "LTE"
    "EHRPD",  #  eHRPD
    "HSPAP",  #  HSPA +
    "3xRTT",  #                 0x00100000, "3XRTT"
    "EVDV",  #                  0x00080000, "1XEVDV"
    "UMB",  #                   0x00400000, "UMB"
]

"""
A L[list] containing all possible integer representations
of preferred network type.
"""
NETWORK_PREFERENCES = [
# Generic value, #Corresponding Android value             Corresponding Windows value
    "3G_PREF",     # 0: "WCDMA_PREF",                    0x0000001F: "DATA_CLASS_HSPA_HSUPA_HSDPA_UMTS_EDGE_GPRS"
    "2G_ONLY",     # 1: "GSM_ONLY",                      0x00000003: "DATA_CLASS_EDGE_GPRS"
    "4G_PREF",     # 9: "LTE_GSM_WCDMA",                 0x0000003F: "DATA_CLASS_LTE_HSUPA_HSDPA_UMTS_EDGE_GPRS"
    "3G_ONLY",     # 2: "WCDMA_ONLY",                    0x0000001C: "DATA_CLASS_HSPA_HSUPA_HSDPA_UMTS
    "2G_3G",       # 3: "GSM_UMTS",                      0x0000001F: "DATA_CLASS_HSPA_HSUPA_HSDPA_UMTS_EDGE_GPRS"
    "CDMA_PREF",   # 4: "CDMA",                          0x003F0000: "DATA_CLASS_1XRTT_3XRTT_1XEVDO_REVA_REVB_EVDV"
    "CDMA_ONLY",   # 5: "CDMA_NO_EVDO"                   0x00110000: "DATA_CLASS_1XRTT_3XRTT"
    "EVDO_ONLY",   # 6: "EVDO_NO_CDMA"                   0x002D0000: "DATA_CLASS_1XEVDO_REVA_REVB_EVDV"
    "GLOBAL",      # 7: "GLOBAL",                        0x03F0003E: "DATA_CLASS_1XRTT_EVDO_HSUPA_HSDPA_UMTS_EDGE_GPRS"
    "4G_PREF_US",  # 8: "LTE_CDMA_EVDO",                 0x03F00020: "DATA_CLASS_LTE_1XRTT_3XRTT_1XEVDO_REVA_REVB_EVDV "
    "WORLD_MODE",  # 10: "LTE_CMDA_EVDO_GSM_WCDMA",      0x03F0003F: "DATA_CLASS_LTE_1XRTT_3XRTT_1XEVDO_REVA_REVB_EVDV_HSUPA_HSDPA_UMTS_EDGE_GPRS"
    "4G_ONLY",     # 11: "LTE_ONLY"                      0x00000020: "DATA_CLASS_LTE"
]


class PreferredNetwork(object):
    """
    Tuples used by preferred network function
    """
    GSM_PREFERRED_NETWORKS = ("3G_PREF", "2G_ONLY", "2G_3G", "GLOBAL", "4G_PREF", "WORLD_MODE")
    CDMA_PREFERRED_NETWORKS = ("CDMA_PREF", "CDMA_ONLY", "GLOBAL", "4G_PREF_US", "WORLD_MODE")
    EVDO_PREFERRED_NETWORKS = ("CDMA_PREF", "EVDO_ONLY", "GLOBAL", "4G_PREF_US", "WORLD_MODE")
    WCDMA_PREFERRED_NETWORKS = ("3G_PREF", "3G_ONLY", "2G_3G", "GLOBAL", "4G_PREF", "WORLD_MODE")
    LTE_PREFERRED_NETWORKS = ("4G_PREF_US", "4G_PREF", "WORLD_MODE", "4G_ONLY")

    GSM_RAT = ("GPRS", "EGPRS", "IDEN")
    CDMA_RAT = ("CDMA",)
    EVDO_RAT = ("EVDO_0", "EVDO_A", "1xRTT", "EVDO_B", "EHRPD", "3xRTT", "EVDV")
    WCDMA_RAT = ("UMTS", "HSDPA", "HSUPA", "HSPA", "HSPAP")
    LTE_RAT = ("LTE")

    GSM_ONLY = "2G_ONLY"
    WCDMA_ONLY = "3G_ONLY"
    LTE_ONLY = "4G_ONLY"
    WCDMA_PREF = "3G_PREF"


class BtProfile(object):
    """
    Class enumerating Bluetooth Profile connection state value :
    """
    NAME = "Profile"
    A2DP = "A2DP"
    HSP = "HSP"
    OPP = "OPP"
    PAN = "PAN"
    HID = "HID"
    GAP = "GAP"
    SDAP = "SDAP"
    GATT = "GATT"


class BtConState(object):
    """
    Class enumerating Bluetooth Profile connection state value :
    """
    NAME = "ConnectionState"

    CONNECTED = "connected"
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    DISCONNECTING = "disconnecting"
    UNKNOWN = "unknown"

    # Dictionary of constant for BT connection state
    d = {
        UNKNOWN: (-1),
        DISCONNECTED: 0,
        CONNECTED: 1,
        CONNECTING: 2,
        DISCONNECTING: 3
    }


class BtAudioState(object):
    """
    Class enumerating Bluetooth Profile audio state value :
    """
    NAME = "AudioState"
    PLAYING = "playing"
    STOPPED = "stopped"

    # Dictionary of constant for BT audio state (valid for A2DP, HSP)
    d = {
        STOPPED: 0,
        PLAYING: 1
    }


class BtAudioCmd(object):
    """
    Class enumerating Bluetooth Profile audio command value :
    """
    NAME = "AudioCmd"
    PLAY = "PLAY"
    STOP = "STOP"
    PAUSE = "PAUSE"
    FW = "FW"
    RW = "RW"


class BtOppDir(object):
    """
    Class enumerating Bluetooth OPP transfer direction value :
    """
    NAME = "OppDir"
    UL = "Upload"
    DL = "Download"

    # Dictionary of constant for BT opp transfer direction
    d = {
        0: UL,
        1: DL
    }


class BluetoothDevice:

    """
    Structure that represent bluetooth device for UECmd
    """

    def __init__(self):
        self.address = ""
        self.name = ""
        self.bonded = ""
        self.interface = ""
        self.uuids = []


class WidiDevice:

    """
    Structure that represent WiDi device for UECmd
    """

    def __init__(self):
        self.address = ""
        self.name = ""
        self.adapter = ""


class BtServiceClass:

    """
    Class enumerating Bluetooth service class and associated UUID :
    """

    BT_SERVICE_CLASS = {"A2DP": "0000110D-0000-1000-8000-00805F9B34FB",
                        "A2DP-SNK": "0000110B-0000-1000-8000-00805F9B34FB",
                        "A2DP-SRC": "0000110A-0000-1000-8000-00805F9B34FB",
                        "AVRCP": "0000110E-0000-1000-8000-00805F9B34FB",
                        "AVRCP-CT": "0000110F-0000-1000-8000-00805F9B34FB",
                        "AVRCP-TG": "0000110C-0000-1000-8000-00805F9B34FB",
                        "BIP": "0000111A-0000-1000-8000-00805F9B34FB",
                        "BIP-ARCH": "0000111C-0000-1000-8000-00805F9B34FB",
                        "BIP-REFOBJ": "0000111D-0000-1000-8000-00805F9B34FB",
                        "BIP-RESP": "0000111B-0000-1000-8000-00805F9B34FB",
                        "BPP": "00001122-0000-1000-8000-00805F9B34FB",
                        "BPP-DP": "00001118-0000-1000-8000-00805F9B34FB",
                        "BPP-DPRO": "00001120-0000-1000-8000-00805F9B34FB",
                        "BPP-PS": "00001123-0000-1000-8000-00805F9B34FB",
                        "BPP-RP": "00001119-0000-1000-8000-00805F9B34FB",
                        "BPP-RUI": "00001121-0000-1000-8000-00805F9B34FB",
                        "CIP": "00001128-0000-1000-8000-00805F9B34FB",
                        "CTP": "00001109-0000-1000-8000-00805F9B34FB",
                        "DID": "00001200-0000-1000-8000-00805F9B34FB",
                        "DUN": "00001103-0000-1000-8000-00805F9B34FB",
                        "FAX": "00001111-0000-1000-8000-00805F9B34FB",
                        "FTP": "00001106-0000-1000-8000-00805F9B34FB",
                        "GNSS": "00001135-0000-1000-8000-00805F9B34FB",
                        "GNSS-SE": "00001136-0000-1000-8000-00805F9B34FB",
                        "HCRP": "00001125-0000-1000-8000-00805F9B34FB",
                        "HCRP-PRINT": "00001126-0000-1000-8000-00805F9B34FB",
                        "HCRP-SCAN": "00001127-0000-1000-8000-00805F9B34FB",
                        "HDP": "00001400-0000-1000-8000-00805F9B34FB",
                        "HDP-SNK": "00001402-0000-1000-8000-00805F9B34FB",
                        "HDP-SRC": "00001401-0000-1000-8000-00805F9B34FB",
                        "HFP": "0000111E-0000-1000-8000-00805F9B34FB",
                        "HFP-AG": "0000111F-0000-1000-8000-00805F9B34FB",
                        "HID": "0000110D-0000-1000-8000-00805F9B34FB",
                        "HSP": "00001108-0000-1000-8000-00805F9B34FB",
                        "HSP-AG": "00001112-0000-1000-8000-00805F9B34FB",
                        "HSP-HS": "00001131-0000-1000-8000-00805F9B34FB",
                        "ICP": "00001110-0000-1000-8000-00805F9B34FB",
                        "LANAP": "00001102-0000-1000-8000-00805F9B34FB",
                        "MAP": "00001134-0000-1000-8000-00805F9B34FB",
                        "MAP-NO": "00001133-0000-1000-8000-00805F9B34FB",
                        "MAP-SE": "00001132-0000-1000-8000-00805F9B34FB",
                        "OPP": "00001105-0000-1000-8000-00805F9B34FB",
                        "PAN-GN": "00001117-0000-1000-8000-00805F9B34FB",
                        "PAN-NAP": "00001116-0000-1000-8000-00805F9B34FB",
                        "PANU": "00001115-0000-1000-8000-00805F9B34FB",
                        "PBAP": "00001130-0000-1000-8000-00805F9B34FB",
                        "PBAP-PCE": "0000112E-0000-1000-8000-00805F9B34FB",
                        "PBAP-PSE": "0000112F-0000-1000-8000-00805F9B34FB",
                        "SAP": "0000112D-0000-1000-8000-00805F9B34FB",
                        "SPP": "00001101-0000-1000-8000-00805F9B34FB",
                        "SYNC": "00001104-0000-1000-8000-00805F9B34FB",
                        "SYNC-CMD": "00001107-0000-1000-8000-00805F9B34FB",
                        "VDP-DIS": "00001305-0000-1000-8000-00805F9B34FB",
                        "VDP-SNK": "00001304-0000-1000-8000-00805F9B34FB",
                        "VDP-SRC": "00001303-0000-1000-8000-00805F9B34FB"
                        }
    # GATT SERVICES: :see: https://developer.bluetooth.org/gatt/services/Pages/ServicesHome.aspx
    # Standard services are named according BLE standard naming
    # device_information = Device Information Service (ble standard)
    # generic_access = Generic Access (ble standard)
    # generic_attribute =  Generic Attribute (ble standard)
    # battery_service = Battery Service (ble standard)
    # heart_rate = Heart Rate Service (ble standard)
    # INTEL-UNS = Intel User Notification Service (led/vibrator patterns)
    # INTEL-EMS = Intel Event Monitoring Service (run/walk/biking, tap, logs)
    # INTEL-DCS = Intel Device Configuration Service (settings)
    # INTEL-FUS = Intel Firmware Update Service (FOTA update)
    # INTEL-ANS-LS = Intel ANCS Loopback Service

    GATT_SERVICE = {"device_information": "0000180a-0000-1000-8000-00805f9b34fb",  # BLE standard
                    "generic_access": "00001800-0000-1000-8000-00805f9b34fb",  # BLE standard
                    "generic_attribute": "00001801-0000-1000-8000-00805f9b34fb",  # BLE standard
                    "battery_service": "0000180f-0000-1000-8000-00805f9b34fb",  # BLE standard
                    "heart_rate": "0000180d-0000-1000-8000-00805f9b34fb",
                    "INTEL-UNS": "bb2b4a1e-474d-4c99-9314-b34c13571727",  # Intel specific
                    "INTEL-EMS": "ed93d4ce-e650-11e3-bd71-82687f4fc15c",  # Intel specific
                    "INTEL-DCS": "4c3402d5-b2f6-45ec-8c9a-f74f0ab5e8c9",  # Intel specific
                    "INTEL-FUS": "f2185c63-6852-4697-b678-cc8d72c5a5ff",  # Intel specific
                    "INTEL-ANS-LS": "ad52b192-2f88-11e4-8c9c-a6c5e4d22fb7"  # Intel specific
                    }

    # see: https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicsHome.aspx
    # Standard characteristics are named according BLE standard naming
    # USERNOTIFICATION_PATTERN = led/vibrator patterns for Clark/Diana/Lois devices
    # GATT_ACTIVITY_VALUE = notification from run/walk/biking, tap, logs events
    # SETTINGS_ID = settings identifier
    # SETTINGS_VALUE = settings value when ID set
    GATT_CHARACTERISTIC = {"model_number_string": "00002a24-0000-1000-8000-00805f9b34fb",  # BLE standard
                            "serial_number_string": "00002a25-0000-1000-8000-00805f9b34fb",  # BLE standard
                            "firmware_revision_string": "00002a26-0000-1000-8000-00805f9b34fb",  # BLE standard
                            "hardware_revision_string": "00002a27-0000-1000-8000-00805f9b34fb",  # BLE standard
                            "software_revision_string": "00002a28-0000-1000-8000-00805f9b34fb",  # BLE standard
                            "manufacturer_name_string": "00002a29-0000-1000-8000-00805f9b34fb",  # BLE standard
                            "device_name": "00002a00-0000-1000-8000-00805f9b34fb",  # BLE standard (generic_access)
                            "appearance": "00002a01-0000-1000-8000-00805f9b34fb",  # BLE standard (generic_access)
                            "peripheral_preferred_connection_parameters": "00002a04-0000-1000-8000-00805f9b34fb",  # BLE standard (generic_access)
                            "service_changed": "00002a05-0000-1000-8000-00805f9b34fb",  # BLE standard
                            "battery_level": "00002a19-0000-1000-8000-00805f9b34fb",  # BLE standard
                            "heart_rate_measurement": "00002a37-0000-1000-8000-00805f9b34fb",  # BLE standard (heart_rate)
                            "body_sensor_location": "00002a38-0000-1000-8000-00805f9b34fb",  # BLE standard (heart_rate)
                            "USERNOTIFICATION_PATTERN": "bb2b0001-474d-4c99-9314-b34c13571727",  # Intel specific
                            "GATT_ACTIVITY_VALUE": "ed930001-e650-11e3-bd71-82687f4fc15c",  # Intel specific
                            "SETTINGS_ID": "4c340001-b2f6-45ec-8c9a-f74f0ab5e8c9",  # Intel specific
                            "SETTINGS_VALUE": "4c340002-b2f6-45ec-8c9a-f74f0ab5e8c9",  # Intel specific
                            "IUFS_CONTROL": "f2180001-6852-4697-b678-cc8d72c5a5ff",  # Intel specific
                            "IUFS_DATA": "f2180002-6852-4697-b678-cc8d72c5a5ff",  # Intel specific
                            "ANCS_NOTIFICATION_SRC": "ad520001-2f88-11e4-8c9c-a6c5e4d22fb7",  # Intel specific
                            "ANCS_DATA_SRC": "ad520002-2f88-11e4-8c9c-a6c5e4d22fb7"  # Intel specific
                            }


class BtGattNotification(object):
    """
    Structure that represent bluetooth gatt (ble) notification for UECmd

    @cvar date: [datetime] The date when notification occurred (datetime format)
    @cvar service: [str] The service which is responsible of the notification
    @cvar characteristic: [str] The characteristic which is responsible of the notification
    @cvar type: [str] The notification type (gesture, charge, ...)
    @cvar data: [list] Additional data of the notification    """

    def __init__(self):
        self.date = None
        self.service = ""
        self.characteristic = ""
        self.type = ""
        self.data = {}


class BluetoothAdvertisedDevice():
    """

    """

    def __init__(self):
        self.address = ""  # key: advertisedDeviceAddress
        self.advertised_name = ""  # key: advertisedDeviceName
        self.rssi = ""  # key: advertisedDeviceRssi
        self.timestamp_nanos = ""  # key: timestampNanos
        self.advertise_flags = ""  # key: advertiseFlags
        self.service_uuids = []  # key: serviceUuids
        self.tx_power_level = ""  # key: advertisedTxPowerLevel
        self.scan_record = ""  # key: ScanRecord
        self.manufacturer_data = ""
        self.service_data = ""

    def init(self, acs_result_string):
        """
        This method takes an ACS result string and transforms it into our bluetooth device object
        :param acs_result_string:
        :return:
        """
        if "advertisedDeviceAddress" in acs_result_string:
            self.address = acs_result_string["advertisedDeviceAddress"]
        if "advertisedDeviceName" in acs_result_string:
            self.name = acs_result_string["advertisedDeviceName"]
        if "advertiseFlags" in acs_result_string:
            self.advertise_flags = acs_result_string["advertiseFlags"]
        if "advertisedDeviceRssi" in acs_result_string:
            self.rssi = acs_result_string["advertisedDeviceRssi"]
        if "timestampNanos" in acs_result_string:
            self.timestamp_nanos = acs_result_string["timestampNanos"]
        if "serviceUuids" in acs_result_string:
            self.service_uuids = acs_result_string["serviceUuids"]
        if "advertisedTxPowerLevel" in acs_result_string:
            self.tx_power_level = acs_result_string["advertisedTxPowerLevel"]
        if "ScanRecord" in acs_result_string:
            self.scan_record = acs_result_string["ScanRecord"]
            self.manufacturer_data = self.get_manufacturer_specific_data()
            self.service_data = self.get_service_data()

    def get_manufacturer_specific_data(self):
        """
        Example of such a string:

        ScanRecord [mAdvertiseFlags=-1, mServiceUuids=null, mManufacturerSpecificData={2=[-66,
         -84, 12, -16, 82, -62, -105, -54, 64, 124, -124, -8, -74, 42, -84, 78, -112, 32, 0,
         9, 0, 6, -75, 0]}, mServiceData={}, mTxPowerLevel=-2147483648, mDeviceName=null]
        self.scan_record.strip('ScanRecord ') will return the string without the 'ScanRecord '
        part:
        [mAdvertiseFlags=-1, mServiceUuids=null, mManufacturerSpecificData={2=[-66,-84, 12,
        -16, 82, -62, -105, -54, 64, 124, -124, -8, -74, 42, -84, 78, -112, 32, 0, 9, 0, 6,
        -75, 0]}, mServiceData={}, mTxPowerLevel=-2147483648, mDeviceName=null]
        self.scan_record.strip('ScanRecord ').split(',', 2)[2] will split after the first 3
        commas and will return only:
        mManufacturerSpecificData={2=[-66,-84, 12, -16, 82, -62, -105, -54, 64, 124, -124,
        -8, -74, 42, -84, 78, -112, 32, 0, 9, 0, 6, -75, 0]}, mServiceData={},
        mTxPowerLevel=-2147483648, mDeviceName=null]
        self.scan_record.strip('ScanRecord ').split(',', 2)[2].rsplit(',', 3)[0] will split
        again by comma, but from the back of the string, and by using the 0 index it will return
        only the manufacturerSpecificData:
        mManufacturerSpecificData={2=[-66,-84, 12, -16, 82, -62, -105, -54, 64, 124, -124,
        -8, -74, 42, -84, 78, -112, 32, 0, 9, 0, 6, -75, 0]}
        .strip('mManufacturerSpecificData=') will result in:
        {2=[-66,-84, 12, -16, 82, -62, -105, -54, 64, 124, -124, -8, -74, 42, -84, 78, -112, 32,
        0, 9, 0, 6, -75, 0]}
        .replace('=', ':') will turn our remaining element into a "stringed" dictionary
        :return: a dictionaryfo manufacturer specific data, identified with the manufacturer
                ID
        """
        import ast
        if "]}, mTxPowerLevel" in self.scan_record:
            man_specific_data = self.scan_record.replace(str(self.service_uuids), "").strip('ScanRecord ')\
                .split(',', 2)[2].rsplit(',', 2)[0].replace(", mServiceData=", '#').strip(" mManufacturerSpecificData=")\
                .replace('=', ':').split('#')[0]
        else:
            man_specific_data = self.scan_record.replace(str(self.service_uuids), "").strip('ScanRecord ')\
                .split(',', 2)[2].rsplit(',', 3)[0].strip(' mManufacturerSpecificData=').replace('=', ':')
        return ast.literal_eval(man_specific_data)

    def get_service_data(self):
        """
        """
        import ast
        if "]}, mTxPowerLevel" in self.scan_record:
            service_data = self.scan_record.replace(str(self.service_uuids), "").strip('ScanRecord ').split(',', 2)[2]\
                .rsplit(',', 2)[0].replace(", mServiceData=", '#').strip(" mManufacturerSpecificData=")\
                .replace('=', ':').split('#')[1].replace('{', "{'").replace(":[", "':[")
        else:
            service_data = self.scan_record.replace(str(self.service_uuids), "").strip('ScanRecord ').split(',', 2)[2]\
                .rsplit(',', 3)[0].strip(' mManufacturerSpecificData=').replace('=', ':')

        return ast.literal_eval(service_data)

    def to_string(self):
        return "Address: {0} AdvertisedName: {1} Rssi: {2} Timestamp_Nanos: {3} AdvertiseFlags: " \
               "{4} Service_Uuids: {5} tx_power_level: {6} ManData {7}" \
               " ServiceData {8}".format(
                self.address if self.address != "" else "null",
                self.advertised_name if self.advertised_name != "" else "null",
                self.rssi if self.rssi != "" else "null",
                self.timestamp_nanos if self.timestamp_nanos != "" else "null",
                self.advertise_flags if self.advertise_flags != "" else "null",
                self.service_uuids if self.service_uuids is not None else "null",
                self.tx_power_level if self.tx_power_level != "" else "null",
                # self.scan_record if self.scan_record != "" else "null",
                self.manufacturer_data if self.manufacturer_data != "" else "null",
                self.service_data if self.service_data != "" else "null")
