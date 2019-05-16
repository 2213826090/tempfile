from PyUiApi.adb_helper.adb_utils import *
import re


class TelephonyUtils(object):
    wifi_cmd = "dumpsys wifi"
    wifi_enabled_txt = "Wi-Fi is enabled"

    data_enable_cmd = "svc data enable"
    data_disable_cmd = "svc data disable"

    states_2G = ["GPRS", "EDGE"]
    states_3G = ["3G", "HSDPA", "HSPAP"]
    states_4G = ["LTE"]

    @staticmethod
    def get_telephony_registry():
        telephony_registry = AdbUtils.run_adb_cmd("dumpsys telephony.registry")
        return telephony_registry

    @staticmethod
    def get_data_connection_state():
        data_connection_state = re.findall("mDataConnectionState=([^\s]+)",
                                           TelephonyUtils.get_telephony_registry())
        LOG.info("Data connection state: " + str(data_connection_state))
        if data_connection_state is not []:
            return data_connection_state[0]

    @staticmethod
    def is_data_connection_possible():
        data_connection_state = re.findall("mDataConnectionPossible=([^\s]+)",
                                           TelephonyUtils.get_telephony_registry())
        if data_connection_state is not []:
            return data_connection_state[0] == "true"

    @staticmethod
    def get_data_connection_apn_status():
        data_connection_apn_status = re.findall("mDataConnectionReason=([^\s]+)",
                                                TelephonyUtils.get_telephony_registry())
        LOG.info("APN status: " + str(data_connection_apn_status))
        if data_connection_apn_status is not []:
            return data_connection_apn_status[0]

    @staticmethod
    def get_service_state():
        service_state = re.findall("(?s)(?<=mServiceState=).+?(?=mSignalStrength)",
                                   TelephonyUtils.get_telephony_registry())
        LOG.info("Service state: " + str(service_state))
        if service_state is not []:
            return service_state[0]

    @staticmethod
    def is_wifi_enabled():
        dumpsys_output = AdbUtils.run_adb_cmd(TelephonyUtils.wifi_cmd)
        if TelephonyUtils.wifi_enabled_txt in dumpsys_output:
            return True
        return False

    @staticmethod
    def enable_mobile_data():
        AdbUtils.run_adb_cmd(TelephonyUtils.data_enable_cmd)

    @staticmethod
    def disable_mobile_data():
        AdbUtils.run_adb_cmd(TelephonyUtils.data_disable_cmd)

    @staticmethod
    def get_connected_service():
        service_state = TelephonyUtils.get_service_state()
        for state in TelephonyUtils.states_2G:
            if state in service_state:
                return "2G"
        for state in TelephonyUtils.states_3G:
            if state in service_state:
                return "3G"
        for state in TelephonyUtils.states_4G:
            if state in service_state:
                return "4G"
        return None

    @staticmethod
    def is_network_camped():
        service_state = TelephonyUtils.get_service_state()
        possible_states = TelephonyUtils.states_2G + TelephonyUtils.states_3G + TelephonyUtils.states_4G
        for state in possible_states:
            if state in service_state:
                LOG.info("DUT camped in %s network" % state)
                return True
        LOG.info("DUT not camped on any network")
        return False

