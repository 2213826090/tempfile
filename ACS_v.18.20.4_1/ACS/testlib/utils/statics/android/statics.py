from testlib.utils.connections.adb import Adb as connection_adb
from testlib.base import base_utils
from testlib.utils.defaults import wifi_defaults
from testlib.scripts.connections.local import local_utils
from testlib.scripts.android.fastboot import fastboot_utils

import sys
import traceback


class StaticsError(Exception):
    """Error for critical steps"""
    pass


class BuildType(object):
    def __init__(self, **kwargs):
        self.has_root = False


class UserDebug(BuildType):
    def __init__(self, **kwargs):
        BuildType.__init__(self, **kwargs)
        self.has_root = True
        self.name = "userdebug"


class User(BuildType):
    def __init__(self, **kwargs):
        BuildType.__init__(self, **kwargs)
        self.name = "user"


class UserSigned(BuildType):
    def __init__(self, **kwargs):
        BuildType.__init__(self, **kwargs)
        self.name = "usersigned"


class SoC(object):
    def __init__(self, **kwargs):
        self.p2p_wpa_cli_connect = "wpa_cli -p/data/misc/wifi/sockets -ip2p0 p2p_connect" + \
                             " [mac_addr] pbc persistent go_intent=15 freq=[p2p_freq]"
        self.wifi_driver = "8723bs"
        self.resource_folder = "img"
        self.edit_flash_file = True
        self.parallel_flash = True
        self.fastboot_to_flash = True
        self.battery_path = 'dollar_cove_battery'
        self.has_bios = True
        self.go_timeout_status = "DISCONNECTED/"
        self.cli_timeout_status = "(DISCONNECTED)|(CONNECTING)"
        self.p2p_mac_mode = "locally administered bit"
        self.reset_button_text = "Reset tablet"
        self.repackage_userdata_on_flash = False
        self.getvar_test_regex_patterns = {"bootloader": "version-bootloader: .+",
                                           "product_regex": "product: \w+",
                                           "secure_regex": "secure: \w+",
                                           "unlocked_regex": "unlocked: \w+",
                                           "charge_regex": "off-mode-charge: \d+",
                                           "battery_regex": "battery-voltage: \d+mV",
                                           "serialno_regex": "serialno: {0}".format(kwargs["serial"])
                                           }
        self.ros_menu_entry = {"android": 0,
                               "bootloader": 1,
                               "factory_reset": 4,
                               "power_off": 8
                              }
        self.fastboot_menu_entry = {"normal_boot": 0,
                                    "power_off": 1,
                                    "bootloader": 2,
                                    "recovery": 3,
                                    "reboot": 4,
                                   }


class BYT(SoC):
    def __init__(self, **kwargs):
        SoC.__init__(self, **kwargs)
        self.flash_configuration_id = "blank_$platform"
        self.partition_bounds_check_string = "FAILED (remote: target partition too small!)"
        self.partition_for_bounds_test = "misc"
        self.repackage_userdata_on_flash = False


class CHT(SoC):
    def __init__(self, **kwargs):
        SoC.__init__(self, **kwargs)
        self.p2p_wpa_cli_connect = "wpa_cli p2p_connect [mac_addr] pbc persistent go_intent=15 freq=[p2p_freq]"
        self.battery_path = 'max170xx_battery'
        self.go_timeout_status = "DISCONNECTED/"
        self.flash_configuration_id = "blank"
        self.resource_folder = "img_cht"
        self.partition_bounds_check_string = "FAILED (remote: Flash failure: Invalid Parameter)"
        self.partition_for_bounds_test = "misc"
        self.fastboot_menu_entry = {"normal_boot": 0,
                                    "bootloader": 1,
                                    "recovery": 2,
                                    "reboot": 3,
                                    "power_off": 4,
                                    "crashmode": 5,
                                   }
        self.crashmode_menu_entry = {"normal_boot": 0,
                                     "bootloader": 1,
                                     "recovery": 2,
                                     "reboot": 3,
                                     "power_off": 4,
                                    }
        self.repackage_userdata_on_flash = False


class SoFIA(SoC):
    def __init__(self, **kwargs):
        SoC.__init__(self, **kwargs)
        self.p2p_wpa_cli_connect = "wpa_cli p2p_connect [mac_addr] pbc persistent go_intent=15 freq=[p2p_freq]"
        self.wifi_driver = "iwlwifi"
        self.resource_folder = "fls"
        self.parallel_flash = False
        self.fastboot_to_flash = True
        self.battery_path = 'battery'
        self.has_bios = False
        self.go_timeout_status = "DISCONNECTED/"
        self.flash_configuration_id = "smp_fls_config_$platform"
        self.edit_flash_file = False
        self.partition_bounds_check_string = "FAILED (remote: Download image size is larger than partition size)"
        self.partition_for_bounds_test = "prg"
        self.reset_button_text = "Reset phone"
        self.provisioning_type = "kb_provisioning"
        self.repackage_userdata_on_flash = True
        self.getvar_test_regex_patterns = {"bootloader": "version-bootloader: .+",
                                           "product_regex": "product: \w+",
                                           "secure_regex": "secure: \w+",
                                           "unlocked_regex": "unlocked: \w+",
                                           "charge_regex": "off-mode-charge: \d+",
                                           }


class BXT(SoC):
    def __init__(self, **kwargs):
        SoC.__init__(self, **kwargs)
        self.wifi_driver = "iwlwifi"
        self.p2p_mac_mode = "increment"
        self.fastboot_menu_entry = {"normal_boot": 0,
                                    "bootloader": 1,
                                    "recovery": 2,
                                    "reboot": 3,
                                    "power_off": 4,
                                    "crashmode": 5,
                                   }
        self.crashmode_menu_entry = {"normal_boot": 0,
                                     "bootloader": 1,
                                     "recovery": 2,
                                     "reboot": 3,
                                     "power_off": 4,
                                    }
        self.repackage_userdata_on_flash = False


class SoFIA_LTE(SoFIA):
    def __init__(self, **kwargs):
        SoFIA.__init__(self, **kwargs)
        self.resource_folder = "fls_lte"


class nexus(SoC):
    def __init__(self, **kwargs):
        SoC.__init__(self, **kwargs)
        self.p2p_wpa_cli_connect = "wpa_cli p2p_connect [mac_addr] pbc persistent go_intent=15 freq=[p2p_freq]"
        self.wifi_driver = "8723bs"
        self.go_timeout_status = "DISCONNECTED/"
        self.flash_configuration_id = None


class coho(BYT):
    def __init__(self, **kwargs):
        BYT.__init__(self, **kwargs)
        self.has_multiple_flashfiles = True


class coho_edu(BYT):
    def __init__(self, **kwargs):
        BYT.__init__(self, **kwargs)
        self.has_multiple_flashfiles = True


class cohol(BYT):
    def __init__(self, **kwargs):
        BYT.__init__(self, **kwargs)
        self.has_multiple_flashfiles = True


class coho_64(BYT):
    def __init__(self, **kwargs):
        BYT.__init__(self, **kwargs)
        self.has_multiple_flashfiles = True


class byt_mrd7(BYT):
    def __init__(self, **kwargs):
        BYT.__init__(self, **kwargs)
        self.has_multiple_flashfiles = True


class cohos3gr(SoFIA):
    def __init__(self, **kwargs):
        SoFIA.__init__(self, **kwargs)
        self.has_multiple_flashfiles = True
        self.resource_folder = "fls_cohos3gr"


class s3gr10m6s(SoFIA):
    def __init__(self, **kwargs):
        SoFIA.__init__(self, **kwargs)
        self.has_multiple_flashfiles = False


class s3gr10m6s_trusty(SoFIA):
    def __init__(self, **kwargs):
        SoFIA.__init__(self, **kwargs)
        self.has_multiple_flashfiles = False


class r2_s3gr10m6s(SoFIA):
    def __init__(self, **kwargs):
        SoFIA.__init__(self, **kwargs)
        self.has_multiple_flashfiles = False


class r2_s3gr20m6s(SoFIA):
    def __init__(self, **kwargs):
        SoFIA.__init__(self, **kwargs)
        self.has_multiple_flashfiles = False


class slti20mr6(SoFIA):
    def __init__(self, **kwargs):
        SoFIA.__init__(self, **kwargs)
        self.has_multiple_flashfiles = False
        self.resource_folder = "fls_lte"


class r2_slti20mr6(SoFIA):
    def __init__(self, **kwargs):
        SoFIA.__init__(self, **kwargs)
        self.has_multiple_flashfiles = False
        self.resource_folder = "fls_lte"


class sltmrdV34(SoFIA_LTE):
    def __init__(self, **kwargs):
        SoFIA_LTE.__init__(self, **kwargs)
        self.has_multiple_flashfiles = False


class r2_sltmrdV34(SoFIA_LTE):
    def __init__(self, **kwargs):
        SoFIA_LTE.__init__(self, **kwargs)
        self.has_multiple_flashfiles = False


class sltmrdV12(SoFIA_LTE):
    def __init__(self, **kwargs):
        SoFIA_LTE.__init__(self, **kwargs)
        self.has_multiple_flashfiles = False


class r2_sltmrdV12(SoFIA_LTE):
    def __init__(self, **kwargs):
        SoFIA_LTE.__init__(self, **kwargs)
        self.has_multiple_flashfiles = False
        self.resource_folder = "fls_lte_A1"


class cht_cr_rvp(CHT):
    def __init__(self, **kwargs):
        CHT.__init__(self, **kwargs)
        self.has_multiple_flashfiles = True
        self.battery_path = 'dollar_cove_battery'


class cht_ffd(CHT):
    def __init__(self, **kwargs):
        CHT.__init__(self, **kwargs)
        self.has_multiple_flashfiles = True
        self.wifi_driver = "bcmdhd_pcie"


class cht_hr(CHT):
    def __init__(self, **kwargs):
        CHT.__init__(self, **kwargs)
        self.has_multiple_flashfiles = True


class cht_rvp(CHT):
    def __init__(self, **kwargs):
        CHT.__init__(self, **kwargs)
        self.has_multiple_flashfiles = True


class r2_cht_mrd(CHT):
    def __init__(self, **kwargs):
        CHT.__init__(self, **kwargs)
        self.has_multiple_flashfiles = True
        self.go_timeout_status = "DISCONNECTED/"
        self.cli_timeout_status = "DISCONNECTED/"


class rvp_b1_bxt(BXT):
    def __init__(self, **kwargs):
        BXT.__init__(self, **kwargs)


class sds_b1_bxt(BXT):
    def __init__(self, **kwargs):
        BXT.__init__(self, **kwargs)


class bxtp_abl(BXT):
    def __init__(self, **kwargs):
        BXT.__init__(self, **kwargs)


class bxtp_abl_car(BXT):
    def __init__(self, **kwargs):
        BXT.__init__(self, **kwargs)


class androidia_64(BXT):
    def __init__(self, **kwargs):
        BXT.__init__(self, **kwargs)

class gordon_peak(BXT):
    def __init__(self, **kwargs):
        BXT.__init__(self, **kwargs)


class Desert(object):
    def __init__(self, **kwargs):
        self.get_interfaces_tool="netcfg"
        self.confirm_pin_go_back={"textContains": "Cancel"}
        self.remove_pin_confirm_desc="Remove unlock PIN"
        self.remove_pin_confirm_button="OK"
        self.download_path = "/storage/sdcard0/Download/"
        self.confirm_view_pin_oem_unlock="Enter your PIN"
        self.remove_trusted_location_vertical_percentage = .85
        self.remove_trusted_location_horizontal_percentage = .75
        self.wifi_more_options_id = {"description":"More options"}
        self.wifi_saved_networks_list_id = {"className":"android.widget.ListView"}
        self.wifi_saved_networks_list_element_id = {"className":"android.widget.TextView"}
        self.wifi_saved_network_forget_btn_id = {"text":"Forget"}
        self.wifi_saved_network_cancel_btn_id = {"text":"Cancel"}
        self.wifi_saved_network_done_btn_id = {"text":"Done"}
        self.wifi_add_network_save_btn_id = {"textContains":"Save"}
        self.wifi_add_network_cancel_btn_id = {"textContains":"Cancel"}
        self.wifi_add_network_connect_btn_id = {"textContains":"Connect"}
        self.quick_settings_wifi_id = {"className": "android.view.View",
                                       "descriptionStartsWith": "Wifi "}
        self.quick_settings_wifi_disconnected_id = {"className": "android.view.View",
                                       "descriptionStartsWith": "Wifi disconnected"}
        self.quick_settings_wifi_off_id = {"className": "android.view.View",
                                       "descriptionStartsWith": "Wifi off"}

        self.chrome_accept_welcome_btn_id = {"text": "Accept & continue"}
        self.chrome_welcome_sign_in_no_thanks_btn_id = {"textMatches": "^(?i)no thanks$"}
        self.airplane_mode_switch_id = {"resourceId": "android:id/switchWidget"}
        self.wifi_p2p_rename_device_id = {"text": "Rename device", "description": "Rename device"}
        self.wifi_p2p_device_searching_id = {"text": u"Searching\u2026"}
        self.wifi_p2p_search_for_devices_id = {"text": "Search for devices"}
        self.wifi_p2p_connect_response_accept_btn_id = {"text": "Accept"}
        self.wifi_p2p_connect_response_decline_btn_id = {"text": "Decline"}
        self.wifi_ca_certificate_none_id = "unspecified"
        self.wifi_network_connect_id = {"text": "Connect"}
        self.oem_unlock_btn_id = {"text": "Enable"}
        self.password_done_btn_id = {"textContains": "Done"}
        self.predefined_language_text_id = "United States"
        self.skip_wifi_btn_id = {"text": "Skip"}
        self.wifi_skip_anyway_btn_id = {"text": "Skip anyway"}
        self.skip_anyway_btn_id = {"text": "Skip anyway"}
        self.skip_pin_btn_id = {"text": "Skip"}
        self.finish_startup_btn_id = {"text": "Next"}
        self.next_btn_id = "Next"
        self.cts_runner_type = "module_based"
        self.all_apps_icon = [
            {"description" : "Apps"},
            {"resourceid" : "com.android.launcher3:id/all_apps_handle"}]


class Lollipop(Desert):
    def __init__(self, **kwargs):
        Desert.__init__(self, **kwargs)
        self.name = "L"
        self.cts_runner_type = "plan_based"


class Marshmallow(Desert):
    def __init__(self, **kwargs):
        Desert.__init__(self, **kwargs)
        self.name = "M"
        self.get_interfaces_tool="ifconfig"
        self.confirm_pin_go_back={"description": "Navigate up"}
        self.remove_pin_confirm_desc="Remove device protection"
        self.remove_pin_confirm_button="Yes, remove"
        self.download_path = "/storage/emulated/0/Download/"
        self.confirm_view_pin_oem_unlock="Confirm your PIN"
        self.cts_runner_type = "plan_based"


class N(Desert):
    def __init__(self, **kwargs):
        Desert.__init__(self, **kwargs)
        ###################### to check ############33
        self.name = "N"
        self.get_interfaces_tool="ifconfig"
        self.confirm_pin_go_back={"description": "Navigate up"}
        self.download_path = "/storage/emulated/0/Download/"
        self.confirm_view_pin_oem_unlock="Confirm your PIN"
        ##################################################

        self.remove_pin_confirm_desc="Remove device protection"
        self.remove_pin_confirm_button="YES, REMOVE"
        self.oem_unlock_btn_id = {"text": "ENABLE"}
        self.wifi_more_options_id = {"description":"Configure"}
        self.wifi_saved_networks_list_id = {"className":"android.support.v7.widget.RecyclerView"}
        self.wifi_saved_networks_list_element_id = {"className":"android.widget.TextView"}
        self.wifi_saved_network_forget_btn_id = {"text":"FORGET"}
        self.wifi_saved_network_cancel_btn_id = {"text":"CANCEL"}
        self.wifi_saved_network_done_btn_id = {"text":"DONE"}
        self.wifi_add_network_save_btn_id = {"textContains":"SAVE"}
        self.wifi_add_network_cancel_btn_id = {"textContains":"CANCEL"}
        self.wifi_add_network_connect_btn_id = {"textContains":"CONNECT"}
        self.quick_settings_wifi_id = {"className": "android.widget.Button",
                                       "descriptionStartsWith": "Wi-Fi "}
        self.quick_settings_wifi_disconnected_id = {"className": "android.widget.Button",
                                       "descriptionStartsWith": "Wi-Fi On,,"}
        self.quick_settings_wifi_off_id = {"className": "android.widget.Button",
                                       "descriptionStartsWith": "Wi-Fi Off"}

        self.chrome_accept_welcome_btn_id = {"text": "ACCEPT & CONTINUE"}
        self.chrome_welcome_sign_in_no_thanks_btn_id = {"text": "NO THANKS"}
        self.airplane_mode_switch_id = {"resourceId": "android:id/switch_widget"}
        self.wifi_p2p_rename_device_id = {"text": "RENAME DEVICE", "description": "Rename device"}
        self.wifi_p2p_device_searching_id = {"text": u"SEARCHING\u2026"}
        self.wifi_p2p_search_for_devices_id = {"text": "SEARCH FOR DEVICES"}
        self.wifi_p2p_connect_response_accept_btn_id = {"text": "ACCEPT"}
        self.wifi_p2p_connect_response_decline_btn_id = {"text": "DECLINE"}
        self.wifi_ca_certificate_none_id = "Do not validate"
        self.wifi_network_connect_id = {"text": "CONNECT"}
        self.password_done_btn_id = {"textContains": "DONE"}
        self.predefined_language_text_id = "UNITED STATES"
        self.skip_wifi_btn_id = {"resourceId": "com.google.android.setupwizard:id/network_dont_connect"}
        self.wifi_skip_anyway_btn_id = {"text": "CONTINUE"}
        self.skip_wifi_text_to_find_id = "CONTINUE"
        self.next_btn_id = "NEXT"
        self.skip_pin_btn_id = {"text": "Not now"}
        self.skip_anyway_btn_id = {"text": "SKIP ANYWAY"}
        self.finish_startup_btn_id = {"text": "Set up later"}
        self.cts_runner_type = "module_based"


class Oreo(Desert):
    def __init__(self, **kwargs):
        Desert.__init__(self, **kwargs)
        ###################### to check ############33
        self.name = "O"
        self.get_interfaces_tool="ifconfig"
        self.confirm_pin_go_back=None
        self.download_path = "/storage/emulated/0/Download/"
        self.confirm_view_pin_oem_unlock="Confirm your PIN"
        ##################################################

        self.remove_pin_confirm_desc="Remove device protection"
        self.remove_pin_confirm_button="YES, REMOVE"
        self.oem_unlock_btn_id = {"text": "ENABLE"}
        self.wifi_more_options_id = {"description":"Configure"}
        self.wifi_saved_networks_list_id = {"className":"android.support.v7.widget.RecyclerView"}
        self.wifi_saved_networks_list_element_id = {"className":"android.widget.TextView"}
        self.wifi_saved_network_forget_btn_id = {"text":"FORGET"}
        self.wifi_saved_network_cancel_btn_id = {"text":"CANCEL"}
        self.wifi_saved_network_done_btn_id = {"text":"DONE"}
        self.wifi_add_network_save_btn_id = {"textContains":"SAVE"}
        self.wifi_add_network_cancel_btn_id = {"textContains":"CANCEL"}
        self.wifi_add_network_connect_btn_id = {"textContains":"CONNECT"}
        self.quick_settings_wifi_id = {"className": "android.widget.Button",
                                       "descriptionStartsWith": "Wi-Fi "}
        self.quick_settings_wifi_disconnected_id = {"className": "android.widget.Button",
                                       "descriptionStartsWith": "Wi-Fi On,,"}
        self.quick_settings_wifi_off_id = {"className": "android.widget.Button",
                                       "descriptionStartsWith": "Wi-Fi Off"}

        self.chrome_accept_welcome_btn_id = {"text": "ACCEPT & CONTINUE"}
        self.chrome_welcome_sign_in_no_thanks_btn_id = {"text": "NO THANKS"}
        self.airplane_mode_switch_id = {"resourceId": "android:id/switch_widget"}
        self.wifi_p2p_rename_device_id = {"text": "RENAME DEVICE"}
        self.wifi_p2p_device_searching_id = {"text": u"SEARCHING\u2026"}
        self.wifi_p2p_search_for_devices_id = {"text": "SEARCH FOR DEVICES"}
        self.wifi_p2p_connect_response_accept_btn_id = {"text": "ACCEPT"}
        self.wifi_p2p_connect_response_decline_btn_id = {"text": "DECLINE"}
        self.wifi_ca_certificate_none_id = "Do not validate"
        self.wifi_network_connect_id = {"text": "CONNECT"}
        self.password_done_btn_id = {"textContains": "DONE"}
        self.predefined_language_text_id = "UNITED STATES"
        self.skip_wifi_btn_id = {"resourceId": "com.google.android.setupwizard:id/network_dont_connect"}
        self.wifi_skip_anyway_btn_id = {"text": "CONTINUE"}
        self.skip_wifi_text_to_find_id = "CONTINUE"
        self.next_btn_id = "NEXT"
        self.skip_pin_btn_id = {"text": "Not now"}
        self.skip_anyway_btn_id = {"text": "SKIP ANYWAY"}
        self.finish_startup_btn_id = {"text": "Set up later"}
        self.cts_runner_type = "module_based"


class Device(object):
    __metaclass__ = base_utils.SingletonType

    __DESSERTS = {
        "L": "Lollipop",
        "M": "Marshmallow",
        "N": "N",
        "O": "Oreo"
    }

    __BUILD_TYPES = {
        "user": "User",
        "userdebug": "UserDebug",
        "usersigned": "UserSigned",
    }

    __PLATFORMS = {
        ### coho target
        # ECS_E7
        "st70408_4_coho": "coho",
        "one7_0_4_coho": "coho",
        "cruise7_coho": "coho",
        ################
        ### cohol target
        # ECS27B
        "ecs27b_0_coho": "cohol",
        "one695_1_coho": "cohol",
        "vsi7q_1_coho": "cohol",
        # ECS28A
        "ecs28a_0_coho": "cohol",
        "one8_0_1_coho": "cohol",
        "vsi8q_1_coho": "cohol",
        "tc80ra3_1_coho": "cohol",
        # ECS210A
        "ecs210a_0_coho": "cohol",
        "tc10ra3_1_coho": "cohol",
        "t10a2ig_1_coho": "cohol",
        # CHIPHD
        "chiphd8_0_coho": "cohol",
        # MALATA8LOW
        "malata8low_2_coho": "cohol",
        "a82i_2_coho": "cohol",
        # MALATA8
        "malata8_0_coho": "cohol",
        # MALATA10
        "a105i_1_coho": "cohol",
        "malata10_0_coho": "cohol",
        # T15
        "t15_0_coho": "cohol",
        "ira101_0_coho": "cohol",
        # I8170
        "i8170_0_coho": "cohol",
        "vta0705_0_coho": "cohol",
        # I8880
        "vta0803_0_coho": "cohol",
        ################
        ### coho_edu target
        "tf16_0_coho": "coho_edu",
        ################
        ### cohos3gr target
        "oars7_0_coho": "cohos3gr",
        "oars7_1_coho": "cohos3gr",
        "s3gr10m6s_0_coho": "cohos3gr",
        "th8_1_coho": "cohos3gr",
        ################
        ### Sofia targets
        "s3gr10m6s_trusty": "s3gr10m6s_trusty",
        "s3gr10m6s": "s3gr10m6s",
        "r2_s3gr10m6s": "r2_s3gr10m6s",
        "r2_s3gr20m6s": "r2_s3gr20m6s",
        "slti20mr6": "slti20mr6",
        "Sf3gr_mrd6_p2": "s3gr10m6s",
        "sltmrdV34": "sltmrdV34",
        "sltmrdV12": "sltmrdV12",
        "r2_slti20mr6": "r2_slti20mr6",
        "r2_sltmrdV12": "r2_sltmrdV12",
        "r2_sltmrdV34": "r2_sltmrdV34",
        ################
        ### CHT targets
        "cht_cr_rvp": "cht_cr_rvp",
        #############
        ##AIA 2.0
        "androidia_64": "androidia_64",
        "r2_cht_cr": "cht_cr_rvp",
        "cht_ffd": "cht_ffd",
        "r2_cht_ffd": "cht_ffd",
        "cht_rvp": "cht_rvp",
        "r2_cht_mrd": "r2_cht_mrd",
        "r2_cht_ffd_m": "cht_ffd",
        ################
        ### nexus target
        "bullhead":"nexus",
        "angler":"nexus",
        "shamu": "nexus",
        "hammerhead": "nexus",
        "fugu": "nexus",
        "volantis": "nexus",
        "volantisg": "nexus",
        "razor": "nexus",
        "razorg": "nexus",
        "mantaray": "nexus",
        "nakasi": "nexus",
        "nakasig": "nexus",
        'vta0705_0_coho': 'cohol',
        ################
        #### BXT #######
        "bxtp_abl": "bxtp_abl",
        "rvp_b1_bxt": "rvp_b1_bxt",
        "sds_b1_bxt": "sds_b1_bxt",
        "bxtp_abl_car": "bxtp_abl_car",
        "gordon_peak": "gordon_peak"
    }

    __SW_PLATFORMS = {
        # ECS_E7
        "TREKSTOR": "st70408_4_coho",
        "CLOUDFONE": "one7_0_4_coho",
        "ECS_E7": "cruise7_coho",

        # ECS27B
        "ECS27B": "ecs27b_0_coho",
        "ONE695": "one695_1_coho",
        "VSI7Q" : "vsi7q_1_coho",
        # ECS28A
        "ECS28A": "ecs28a_0_coho",
        "ONE8_0": "one8_0_1_coho",
        "VSI8Q": "vsi8q_1_coho",
        "TC80RA3": "tc80ra3_1_coho",
        # ECS210A
        "ECS210A" : "ecs210a_0_coho",
        "TC10RA3": "tc10ra3_1_coho",
        "T10A2IG": "t10a2ig_1_coho",
        # CHIPHD
        "CHIPHD8": "chiphd8_0_coho",
        # MALATA8LOW
        "MALATA8LOW": "malata8low_2_coho",
        "A82I": "a82i_2_coho",
        # MALATA8
        "MALATA8": "malata8_0_coho",
        # MALATA10
        "A105I" : "a105i_1_coho",
        # T15
        "T15": "t15_0_coho",
        "IRA101": "ira101_0_coho",
        # I8170
        "I8170": "i8170_0_coho",
        "VTA0705": "vta0705_0_coho",
        # I8880
        "VTA0803": "vta0803_0_coho",

        # TF16
        "TF16": "tf16_0_coho",

        # COHO 3GR
        "OARS7_EVT": "oars7_0_coho",
        "OARS7": "oars7_1_coho",
        "S3GR10M6S": "s3gr10m6s_0_coho",
        "TH8_PVT": "th8_1_coho",

        # SOFIA
        "s3gr10m6s": "s3gr10m6s",
        "r2_s3gr10m6s": "r2_s3gr10m6s",
        "sf_3gr": "r2_s3gr10m6s",
        "r2_s3gr20m6s": "r2_s3gr20m6s",
        "slti20mr6": "slti20mr6",
        "sltmrdV34": "sltmrdV34",
        "sltmrdV12": "sltmrdV12",
        "r2_slti20mr6": "r2_slti20mr6",
        "r2_sltmrdV12": "r2_sltmrdV12",
        "r2_sltmrdV34": "r2_sltmrdV34",
        "SF_LTE": "r2_sltmrdV34",

        # CHT
        "r2_cht_cr": "r2_cht_cr",
        "r2_cht_ffd": "r2_cht_ffd",
        "cht_ffd": "cht_ffd",
        "r2_cht_mrd": "r2_cht_mrd",
        "r2_cht_ffd_m": "r2_cht_ffd_m",

        #AIA 2.0
        "androidia_64": "androidia_64",
        # BXT
        "bxtp_abl": "bxtp_abl",
        "rvp_b1_bxt": "rvp_b1_bxt",
        "sds_b1_bxt": "sds_b1_bxt",
        "bxtp_abl_car": "bxtp_abl_car",
        "gordon_peak": "gordon_peak"
    }

    def __init__(self, **kwargs):
        self.kwargs = kwargs
        if kwargs.has_key("platform") and kwargs["platform"] is not None:
            platform = self.__SW_PLATFORMS[kwargs["platform"]]
            target = self.__PLATFORMS[platform]
            self._target = globals()[target](**kwargs)
        else:
            self._target = None
        if kwargs.has_key("dessert") and kwargs["dessert"] is not None:
            dessert = self.__DESSERTS[kwargs["dessert"]]
            self._dessert = globals()[dessert](**kwargs)
        else:
            self._dessert = None

        if kwargs.has_key("build_type") and kwargs["build_type"] is not None:
            build_type = self.__BUILD_TYPES[kwargs["build_type"]]
            self._build_type = globals()[build_type](**kwargs)
        else:
            self._build_type = None

        self.test = False

    def _initialize_target(self):
        try:
            boot_state = local_utils.get_device_boot_state(serial=self.kwargs["serial"])
            if boot_state == "android":
                adb_connection = connection_adb(serial = self.kwargs["serial"])
                platform = adb_connection.get_prop(prop = "ro.product.device")
            elif boot_state == "fastboot":
                if fastboot_utils.var_exists(var="product-string", serial=self.kwargs["serial"]):
                    product_string = fastboot_utils.get_var(var="product-string", serial=self.kwargs["serial"])
                    platform = str(product_string).split("/")[-1]
                elif fastboot_utils.var_exists(var="product", serial=self.kwargs["serial"]):
                    product_string = fastboot_utils.get_var(var="product", serial=self.kwargs["serial"])
                    platform = self.__SW_PLATFORMS[product_string]
            else:
                raise Exception("Invalid boot state - {0}".format(boot_state))
            target = self.__PLATFORMS[platform]
            self._platform = platform
            self._target = globals()[target](**self.kwargs)
        except:
            raise StaticsError("{0}: Error while trying to get target from the device - {1}".format(self.kwargs["serial"], traceback.format_exc()))

    def _initialize_build_type(self):
        try:
            adb_connection = connection_adb(serial = self.kwargs["serial"])
            build_type = adb_connection.get_prop(prop = "ro.build.type")
            keys = adb_connection.get_prop(prop = "ro.build.fingerprint")
            if "release-keys" in  keys:
                if build_type == "user":
                    self._build_type = UserSigned(**self.kwargs)
                else:
                    raise StaticsError("{0}: Error while trying to get build type: wrong ro.build.type for release keys {1}".format(self.kwargs["serial"],
                                                                                                                                    build_type))
            elif "test-keys" in  keys:
                if build_type == "user":
                    self._build_type = User(**self.kwargs)
                elif build_type == "userdebug":
                    self._build_type = UserDebug(**self.kwargs)
                else:
                    raise StaticsError("{0}: Error while trying to get "
                                       "build type: wrong ro.build.type"
                                       ": {1} - {2}"\
                                       .format(self.kwargs["serial"],
                                               build_type,
                                               traceback.format_exc()))
            else:
                raise StaticsError("{0}: Error while trying to get build key type: wrong ro.build.fingerprint: {1}".format(self.kwargs["serial"],
                                                                                                                           build_type))
        except:
            raise StaticsError("{0}: Error while trying to get build type from the device - {1}".format(self.kwargs["serial"],
                                                                                                        traceback.format_exc()))

    def _initialize_dessert(self):
        try:
            adb_connection = connection_adb(serial = self.kwargs["serial"])
            api_level = adb_connection.get_prop(prop = "ro.build.version.sdk")
            if int(api_level) < 23 and int(api_level) > 20:
                dessert = self.__DESSERTS["L"]
            elif int(api_level) == 23 and adb_connection.get_prop(prop = "ro.build.version.release") != "N":
                dessert = self.__DESSERTS["M"]
            elif int(api_level) == 24:
                dessert = self.__DESSERTS["N"]
            elif int(api_level) == 25:
                dessert = self.__DESSERTS["N"]
            elif adb_connection.get_prop(prop = "ro.build.version.release") == "N":
                dessert = self.__DESSERTS["N"]
            elif int(api_level) == 26 or int(api_level) == 27:
                dessert = self.__DESSERTS["O"]
            else:
                raise StaticsError("{0}: Unknown API level: {1}".format(self.kwargs["serial"], str(api_level)))
            self._dessert = globals()[dessert](**self.kwargs)
        except:
            raise StaticsError("{0}: Error while trying to get API level from the device - {1}".format(self.kwargs["serial"], traceback.format_exc()))

    @property
    def partition_bounds_check_string(self):
        if self._target == None:
            self._initialize_target()
        return self._target.partition_bounds_check_string

    @property
    def partition_for_bounds_test(self):
        if self._target == None:
            self._initialize_target()
        return self._target.partition_for_bounds_test

    @property
    def getvar_test_regex_patterns(self):
        if self._target == None:
            self._initialize_target()
        return self._target.getvar_test_regex_patterns

    @property
    def resource_folder(self):
        if self._target == None:
            self._initialize_target()
        return self._target.resource_folder

    @property
    def has_multiple_flashfiles(self):
        if self._target == None:
            self._initialize_target()
        return self._target.has_multiple_flashfiles

    @property
    def edit_flash_file(self):
        if self._target == None:
            self._initialize_target()
        return self._target.edit_flash_file

    def p2p_wpa_cli_connect(self, mac_address, p2p_freq = wifi_defaults.p2p['p2p_freq']):
        # setting the default p2p frequency the same as the STA frequency <=> single channel
        if self._target == None:
            self._initialize_target()
        return self._target.p2p_wpa_cli_connect.replace("[mac_addr]", mac_address).replace("[p2p_freq]", p2p_freq)

    @property
    def wifi_driver(self):
        if self._target == None:
            self._initialize_target()
        return self._target.wifi_driver

    @property
    def parallel_flash(self):
        if self._target == None:
            self._initialize_target()
        return self._target.parallel_flash

    @property
    def fastboot_to_flash(self):
        if self._target == None:
            self._initialize_target()
        return self._target.fastboot_to_flash

    @property
    def battery_path(self):
        if self._target == None:
            self._initialize_target()
        return self._target.battery_path

    @property
    def has_bios(self):
        if self._target == None:
            self._initialize_target()
        return self._target.has_bios

    @property
    def provisioning_type(self):
        if self._target == None:
            self._initialize_target()
        return self._target.provisioning_type

    @property
    def cli_timeout_status(self):
        if self._target == None:
            self._initialize_target()
        if self._dessert == None:
            self._initialize_dessert()
        if (isinstance(self._target, cht_ffd) or isinstance(self._target, cht_rvp)) and isinstance(self._dessert, Marshmallow):
            return "DISCONNECTED"
        else:
            return self._target.cli_timeout_status

    @property
    def p2p_mac_mode(self):
        if self._target == None:
            self._initialize_target()
        if self._dessert == None:
            self._initialize_dessert()
        if isinstance(self._target, SoFIA_LTE) and isinstance(self._dessert, Marshmallow):
            return "increment"
        else:
            return self._target.p2p_mac_mode

    @property
    def go_timeout_status(self):
        if self._target == None:
            self._initialize_target()
        return self._target.go_timeout_status

    @property
    def flash_configuration_id(self):
        if self._target == None:
            self._initialize_target()
        return self._target.flash_configuration_id

    @property
    def reset_button_text(self):
        if self._target == None:
            self._initialize_target()
        return self._target.reset_button_text

    @property
    def ros_menu_entry(self):
        if self._target == None:
            self._initialize_target()
        return self._target.ros_menu_entry

    @property
    def fastboot_menu_entry(self):
        if self._target == None:
            self._initialize_target()
        return self._target.fastboot_menu_entry

    @property
    def crashmode_menu_entry(self):
        if self._target == None:
            self._initialize_target()
        return self._target.crashmode_menu_entry

    @property
    def download_path(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.download_path

    @property
    def get_interfaces_tool(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.get_interfaces_tool

    @property
    def confirm_pin_go_back(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.confirm_pin_go_back

    @property
    def remove_pin_confirm_desc(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.remove_pin_confirm_desc

    @property
    def remove_pin_confirm_button(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.remove_pin_confirm_button

    @property
    def confirm_view_pin_oem_unlock(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.confirm_view_pin_oem_unlock

    @property
    def wifi_more_options_id(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.wifi_more_options_id

    @property
    def wifi_saved_networks_list_id(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.wifi_saved_networks_list_id

    @property
    def wifi_saved_networks_list_element_id(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.wifi_saved_networks_list_element_id

    @property
    def wifi_saved_network_forget_btn_id(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.wifi_saved_network_forget_btn_id

    @property
    def wifi_saved_network_cancel_btn_id(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.wifi_saved_network_cancel_btn_id

    @property
    def wifi_saved_network_done_btn_id(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.wifi_saved_network_done_btn_id

    @property
    def wifi_add_network_connect_btn_id(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.wifi_add_network_connect_btn_id

    @property
    def quick_settings_wifi_id(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.quick_settings_wifi_id


    @property
    def quick_settings_wifi_disconnected_id(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.quick_settings_wifi_disconnected_id

    @property
    def quick_settings_wifi_off_id(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.quick_settings_wifi_off_id

    @property
    def wifi_add_network_save_btn_id(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.wifi_add_network_save_btn_id

    @property
    def wifi_add_network_cancel_btn_id(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.wifi_add_network_cancel_btn_id

    @property
    def chrome_accept_welcome_btn_id(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.chrome_accept_welcome_btn_id

    @property
    def chrome_welcome_sign_in_no_thanks_btn_id(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.chrome_welcome_sign_in_no_thanks_btn_id

    @property
    def airplane_mode_switch_id(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.airplane_mode_switch_id

    @property
    def wifi_p2p_rename_device_id(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.wifi_p2p_rename_device_id

    @property
    def wifi_p2p_device_searching_id(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.wifi_p2p_device_searching_id

    @property
    def wifi_p2p_search_for_devices_id(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.wifi_p2p_search_for_devices_id

    @property
    def wifi_p2p_connect_response_accept_btn_id(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.wifi_p2p_connect_response_accept_btn_id

    @property
    def wifi_p2p_connect_response_decline_btn_id(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.wifi_p2p_connect_response_decline_btn_id

    @property
    def wifi_ca_certificate_none_id(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.wifi_ca_certificate_none_id

    @property
    def wifi_network_connect_id(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.wifi_network_connect_id

    @property
    def remove_trusted_location_vertical_percentage(self):
        return self._dessert.remove_trusted_location_vertical_percentage

    @property
    def remove_trusted_location_horizontal_percentage(self):
        return self._dessert.remove_trusted_location_horizontal_percentage

    @property
    def cts_runner_type(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.cts_runner_type

    @property
    def dessert(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.name

    @property
    def build_type(self):
        if self._build_type == None:
            self._initialize_build_type()
        return self._build_type.name

    @property
    def repackage_userdata_on_flash(self):
        return self._target.repackage_userdata_on_flash

    @property
    def oem_unlock_btn_id(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.oem_unlock_btn_id

    @property
    def password_done_btn_id(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.password_done_btn_id

    @property
    def predefined_language_text_id(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.predefined_language_text_id

    @property
    def skip_wifi_btn_id(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.skip_wifi_btn_id

    @property
    def wifi_skip_anyway_btn_id(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.wifi_skip_anyway_btn_id

    @property
    def skip_pin_btn_id(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.skip_pin_btn_id

    @property
    def skip_anyway_btn_id(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.skip_anyway_btn_id

    @property
    def next_btn_id(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.next_btn_id

    @property
    def finish_startup_btn_id(self):
        if self._dessert == None:
            self._initialize_dessert()
        return self._dessert.finish_startup_btn_id

    @property
    def platform(self):
        if self._target == None:
            self._initialize_target()
        return self._platform

    @property
    def all_apps_icon(self):
        if self._dessert == None:
            self._initialize_dessert()
        if self.platform != 'gordon_peak':
            return self._dessert.all_apps_icon
        else:
            return None