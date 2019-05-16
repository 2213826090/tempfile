#!/usr/bin/env python

class Constants():
    """
    """

    class BtConstants():
        """
        """
        # BT constants
        SVC_NAME = "org.bluez"
        ADAPTER_IFACE = SVC_NAME + ".Adapter1"
        DEV_IFACE = SVC_NAME + ".Device1"
        AGENT_IFACE = SVC_NAME +".Agent1"
        AGENT_MNGR_IFACE = SVC_NAME + ".AgentManager1"
        NETWORK_SRV_IFACE = SVC_NAME + ".NetworkServer1"
        NETWORK_IFACE = SVC_NAME + ".Network1"
        PROFILE_MNGR_IFACE = SVC_NAME + ".ProfileManager1"
        PROFILE_IFACE = SVC_NAME + ".Profile1"

    class DbusConstants():
        """
        """
        PROPS = "org.freedesktop.DBus.Properties"
        OBJ_MNGR = "org.freedesktop.DBus.ObjectManager"
        OBJ_MNGR_PATH = "/"
        BT_PATH = "/org/bluez"

    Bt = BtConstants()
    DBus = DbusConstants()
