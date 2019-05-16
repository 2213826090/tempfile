""" PUPDR_common Basic functions for PUPDR tests implementation
Offers a set of custom functions for PUPDR testing team.

"""

"""
==================================================================================================

(c)Copyright 2013, Intel Corporation All Rights Reserved.

The source code contained or described here in and all documents related to the source code
("Material") are owned by Intel Corporation or its suppliers or licensors. Title to the Material
remains with Intel Corporation or its suppliers and licensors. The Material contains trade secrets
and proprietary and confidential information of Intel or its suppliers and licensors. The Material
is protected by worldwide copyright and trade secret laws and treaty provisions. No part of the
Material may be used, copied, reproduced, modified, published, uploaded, posted, transmitted,
distributed, or disclosed in any way without Intels prior express written permission.

No license under any patent, copyright, trade secret or other intellectual property right is granted
to or conferred upon you by disclosure or delivery of the Materials, either expressly, by
implication, inducement, estoppel or otherwise. Any license under such intellectual property rights
must be express and approved by Intel in writing.


====================================================================================================
"""

import sys
import os
import re
import json
import ast
import UtilitiesFWK.Utilities as Util
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsBaseException import AcsBaseException
from Core.Report.Live.LiveReporting import LiveReporting
sys.path.append(LIBRARY_PATH_LIST)
import PupdrLibrary as PL

SPLIT_LOG = " | "

##Declare variables
NETWORK_API = None
TC_CRASHLOGS = None
TCFULLNAME = str()
TCNAME = str()
HARDWARE = str()
BUILD_NAME = str()
BRANCH_NAME = str()
NOMOS = False

##Device specifics
BOARD_TYPE = str()
BUILDVARIANT = str()
DEVICE_NAME = ""
CODE_VERSION = str()
WORKDIR = str()
FLASH_FILES = str()
PROVISIONING_DATA = dict()

# External Library
GLOBAL_CONF = dict()

#Context
EXEC_SCRIPT_CTX = None

TC_GLOBAL_VARS = ["TCFULLNAME", "TCNAME", "DEVICE_NAME", "NOMOS", "WORKDIR",
                  "HARDWARE", "BUILD_NAME", "FLASH_FILES",
                  "BRANCH_NAME", "BOARD_TYPE", "BUILDVARIANT"]

def init(glb):
    globals().update(glb)
    global EXEC_SCRIPT_CTX
    EXEC_SCRIPT_CTX = glb

    global GLOBAL_CONF
    GLOBAL_CONF = dict()

    # PUPDR code version:
    global CODE_VERSION
    CODE_VERSION = "Release_2017WW12.5.14h05"

    # ACS verdicts
    global SUCCESS
    SUCCESS = Util.Global.SUCCESS
    global FAILURE
    FAILURE = Util.Global.FAILURE
    global BLOCKED
    BLOCKED = Util.Global.BLOCKED

    # VERDICT shall not be set to SUCCESS anywhere else!!
    EXEC_SCRIPT_CTX["VERDICT"] = SUCCESS

    GLOBAL_CONF["DEFAULT_INIT"] = True

    # Reset NOMOS
    global NOMOS
    NOMOS = False

    # Set flash files
    global FLASH_FILES
    FLASH_FILES = PATH_MANAGER.FLASH_FILES
    if FLASH_FILES is None:
        FLASH_FILES = ""

    # Get device name ($DEVICE in environment and -d parameter in ACS)
    global DEVICE_NAME
    DEVICE_NAME = DEVICE_MANAGER.get_global_config().deviceConfig.get('PHONE1').get('Name')
    GLOBAL_CONF["DEVICE_NAME"] = DEVICE_NAME

    # set external configuration
    GLOBAL_CONF["EXTERNAL_CONFIGURATION"] = {"Configuration": dict()}
    GLOBAL_CONF["EXTERNAL_CONFIGURATION"]["Configuration"]["Timeout"] = dict()
    GLOBAL_CONF["EXTERNAL_CONFIGURATION"]["Configuration"]["Timeout"]["MOS_BOOT"] = int(DEVICE.get_config("mainOsBootDuration", 0, int))
    GLOBAL_CONF["EXTERNAL_CONFIGURATION"]["Configuration"]["Timeout"]["ROS_BOOT"] = int(DEVICE.get_config("recoveryOsBootDuration", 0, int))
    GLOBAL_CONF["EXTERNAL_CONFIGURATION"]["Configuration"]["Timeout"]["POS_BOOT"] = int(DEVICE.get_config("provOsBootDuration", 0, int))
    GLOBAL_CONF["EXTERNAL_CONFIGURATION"]["Configuration"]["Timeout"]["COS_BOOT"] = int(DEVICE.get_config("chargingOsBootDuration", 0, int))
    GLOBAL_CONF["EXTERNAL_CONFIGURATION"]["Configuration"]["Timeout"]["BOOT_TIMEOUT"] = int(DEVICE.get_config("bootTimeout", 0, int))
    GLOBAL_CONF["EXTERNAL_CONFIGURATION"]["Configuration"]["Timeout"]["SHUTDOWN_TIMEOUT"] = int(DEVICE.get_config("softShutdownDuration", 0, int))
    GLOBAL_CONF["EXTERNAL_CONFIGURATION"]["Configuration"]["Timeout"]["TIMEOUT_TOLERANCE"] = int(DEVICE.get_config("globalBootTimeoutTolerance", 0, int))
    GLOBAL_CONF["EXTERNAL_CONFIGURATION"]["Configuration"]["Timeout"]["PKPON"] = int(DEVICE.get_config("pressPowerBtnTimeSwitchOn", 0, int))
    GLOBAL_CONF["EXTERNAL_CONFIGURATION"]["Configuration"]["Timeout"]["PKPOFF"] = int(DEVICE.get_config("pressPowerBtnTimeSwitchOff", 0, int))
    GLOBAL_CONF["EXTERNAL_CONFIGURATION"]["Configuration"]["Timeout"]["WATCHDOG_EXPIRATION"] = int(DEVICE.get_config("watchdogExpirationDuration", 0, int))
    GLOBAL_CONF["EXTERNAL_CONFIGURATION"]["Configuration"]["Timeout"]["ROS_EXPIRATION"] = int(DEVICE.get_config("recoveryOsExpirationDuration", 0, int))
    GLOBAL_CONF["EXTERNAL_CONFIGURATION"]["Configuration"]["Timeout"]["ADB_ROOT_TIMEOUT"] = int(DEVICE.get_config("adbRootTimeout", 0, int))
    # update only if param is false
    if not get_pupdrParam("pupdrParamCheckLastBootData"):
        GLOBAL_CONF["EXTERNAL_CONFIGURATION"]["Configuration"]["BootVars"] = dict()
        GLOBAL_CONF["EXTERNAL_CONFIGURATION"]["Configuration"]["BootVars"]["BOOT_DATA_CHECK"] = str(DEVICE.get_config("pupdrParamCheckLastBootData", False, int))

    # Relay buttons
    GLOBAL_CONF["EXTERNAL_RELAY_CARD"] = IO_CARD
    try:
        GLOBAL_CONF["RELAY_CARD_CONFIGURATION"] = dict()
        GLOBAL_CONF["RELAY_CARD_CONFIGURATION"]["relays"] = dict()
        iocard = BENCH_CONFIG.get_parameters(DEVICE.get_config("IoCard", "IO_CARD", str))
        try:
            GLOBAL_CONF["RELAY_CARD_CONFIGURATION"]["relays"]["UsbHostPcConnect"] = int(iocard.get_param_value("UsbHostPcConnect"))
        except:
            pass
        try:
            GLOBAL_CONF["RELAY_CARD_CONFIGURATION"]["relays"]["SwitchOnOff"] = int(iocard.get_param_value("SwitchOnOff"))
        except:
            pass
        try:
            GLOBAL_CONF["RELAY_CARD_CONFIGURATION"]["relays"]["J6B2_relay"] = int(iocard.get_param_value("J6B2_relay"))
        except:
            pass
        try:
            GLOBAL_CONF["RELAY_CARD_CONFIGURATION"]["relays"]["VolumeUp"] = int(iocard.get_param_value("VolumeUp"))
        except:
            pass
        try:
            GLOBAL_CONF["RELAY_CARD_CONFIGURATION"]["relays"]["VolumeDown"] = int(iocard.get_param_value("VolumeDown"))
        except:
            pass
        try:
            GLOBAL_CONF["RELAY_CARD_CONFIGURATION"]["relays"]["CameraShort"] = int(iocard.get_param_value("CameraShort"))
        except:
            pass
        try:
            GLOBAL_CONF["RELAY_CARD_CONFIGURATION"]["relays"]["CameraLong"] = int(iocard.get_param_value("CameraLong"))
        except:
            pass
        try:
            GLOBAL_CONF["RELAY_CARD_CONFIGURATION"]["relays"]["Dediprog"] = int(iocard.get_param_value("Dediprog"))
        except:
            pass
        try:
            GLOBAL_CONF["RELAY_CARD_CONFIGURATION"]["relays"]["PowerSupply"] = int(iocard.get_param_value("PowerSupply"))
        except:
            pass
        try:
            GLOBAL_CONF["RELAY_CARD_CONFIGURATION"]["ComPort"] = iocard.get_param_value("ComPort")
        except:
            pass
        try:
            GLOBAL_CONF["RELAY_CARD_CONFIGURATION"]["WiringTable"] = iocard.get_param_value("WiringTable")
        except:
            pass
        try:
            GLOBAL_CONF["RELAY_CARD_CONFIGURATION"]["DefaultStates"] = iocard.get_param_value("DefaultStates")
        except:
            pass
    except:
        pass

    GLOBAL_CONF["SKIP_WIFI_CHECK"] = get_pupdrParam("pupdrParamSkipWifiCheck")
    GLOBAL_CONF["FORCE_SHUTDOWN_IN_GOTO_MOS"] = get_pupdrParam("pupdrParamGotoMosForceShtn")
    if DEVICE.get_config("pupdrInternalDownload", "", str):
        try:
            value = ast.literal_eval(DEVICE.get_config("pupdrInternalDownload", "", str))
        except Exception as e:
            print_log("WARNING", "issue while converting '{0}' into dictionary (error={1})".format(DEVICE.get_config("pupdrInternalDownload", "", str), e))
        else:
            GLOBAL_CONF["EXTERNAL_CONFIGURATION"]["Configuration"]["Flash"] = dict()
            GLOBAL_CONF["EXTERNAL_CONFIGURATION"]["Configuration"]["Flash"]["INTERNAL_DOWNLOAD"] = value
    GLOBAL_CONF["FLASH_BIOS"] = DEVICE.get_config("flashBIOS", False, "str_to_bool")
    GLOBAL_CONF["REPORT_PATH"] = REPORT_PATH
    GLOBAL_CONF["EXTERNAL_LOCAL_EXEC"] = LOCAL_EXEC
    GLOBAL_CONF["EXTERNAL_LOGGER"] = print_log

    # credential handling
    GLOBAL_CONF["CREDENTIALS"] = ""
    try:
        GLOBAL_CONF["CREDENTIALS"] = CREDENTIALS
        if not GLOBAL_CONF["CREDENTIALS"]:
            GLOBAL_CONF["CREDENTIALS"] = ""
    except Exception as e:
        print_log("WARNING", "issue with credentials from ACS (error={0})".format(e))

    # if SSN is defined in Bench_Config, use it for all adb and fastboot commands
    GLOBAL_CONF["DEVICE_SSN"] = DEVICE.get_config("serialNumber", "", str)
    GLOBAL_CONF["FASTBOOT_SSN"] = DEVICE.get_config("fastbootSerialNumber", "", str)
    GLOBAL_CONF["DNX_SSN"] = DEVICE.get_config("dnxSerialNumber", "", str)

    enable_group = DEVICE.get_config("pupdrFastbootEnableGroups", "", str)
    disable_group = DEVICE.get_config("pupdrFastbootDisableGroups", "", str)
    override_dict = {}
    if enable_group:
        enable_group = enable_group.split(";")
        override_dict["fastboot"] = {"groups": {}}
        for single_group in enable_group:
            override_dict["fastboot"]["groups"][single_group] = True
    if disable_group:
        disable_group = disable_group.split(";")
        if "fastboot" not in override_dict:
            override_dict["fastboot"] = {"groups": {}}
        for single_group in disable_group:
            override_dict["fastboot"]["groups"][single_group] = False
    if override_dict:
        GLOBAL_CONF["INTERNAL_DOWNLOAD_OVERRIDE"] = override_dict

    custom_config_json = DEVICE.get_config("pupdrConfigFile", "", str)
    if not custom_config_json:
        print_log("WARNING", "pupdrConfigFile bench config property empty")
    elif not os.path.isfile(custom_config_json):
        print_log("WARNING", "file not found: {}".format(custom_config_json))
    else:
        try:
            with open(custom_config_json) as f:
                json_data = json.load(f)
        except Exception as e:
            print_log("WARNING", "failure to load json file {} (error={})".format(custom_config_json, e))
        else:
            GLOBAL_CONF["CUSTOM_CONFIG"] = json_data

    # Set HDK Bool if present
    global HDK
    HDK = True if "HDK" in DEVICE_NAME else False
    GLOBAL_CONF["HDK"] = HDK

    global NETWORK_API
    NETWORK_API = DEVICE.get_uecmd("Networking")

    # TCR and report
    GLOBAL_CONF["EXTERNAL_DATA_PUSH"] = TCR_wrapper
    GLOBAL_CONF["APLOGS_PUSH"] = True

    PL.Pupdr().init(GLOBAL_CONF)
    PL.Pupdr().Output.appendOutput("", None)

# !!!
# Call of methods for TCs Initialization must be done in tail of this file
# !!!

#------------------------------------------------------------------------------#

def get_pupdrParam(param):
    """ Read xml parameter from benchConfig/DeviceCatalog in Phone part
    """
    return DEVICE.get_config(param, True, "str_to_bool")

#------------------------------------------------------------------------------#

def blocked(output, direct_exit=False):
    """ Raise an exception and report a BLOCKED status
    """
    EXEC_SCRIPT_CTX["OUTPUT"] = TCNAME + " # " + output + " --> BLOCKED"
    print_log("WARNING", EXEC_SCRIPT_CTX["OUTPUT"])
    ex = AcsBaseException(EXEC_SCRIPT_CTX["OUTPUT"])
    ex._error_code = BLOCKED
    EXEC_SCRIPT_CTX["VERDICT"] = BLOCKED
    finalStep(direct_exit=direct_exit)
    PL.Pupdr().Output.appendOutput("", None)
    raise ex

#------------------------------------------------------------------------------#

def failure(output, exception=True):
    """ Report a failure on the output string in parameter.
    If the exception parameter is True it raise a DeviceException
    """
    EXEC_SCRIPT_CTX["VERDICT"] = FAILURE
    print_log("WARNING", TCNAME + " # " + output + " --> FAILURE")
    if exception:
        EXEC_SCRIPT_CTX["OUTPUT"] = output
        finalStep()
        PL.Pupdr().Output.appendOutput("", None)
        raise DeviceException(DeviceException.OPERATION_FAILED, EXEC_SCRIPT_CTX["OUTPUT"])
    else:
        PL.Pupdr().Output.appendOutput("", None)

#------------------------------------------------------------------------------#

def print_log(logtype, content):
    """ Call ACS methods PRINT_INFO, PRINT_ERROR and PRINT_DEBUG
        Purpose : Add TEST variable by default
        Usage   : print_log("INFO", "test is successful")
    """
    if logtype not in ("INFO", "WARNING", "DEBUG", "ERROR"):
        PRINT_ERROR("print_log(): bad use of %s (%s)" % (str(logtype)))
    else:
        eval("PRINT_" + logtype)(str(content))

#------------------------------------------------------------------------------#

def local_exec(command, timeout):
    """ Call ACS method LOCAL_EXEC
    """
    return LOCAL_EXEC(command, timeout)

#------------------------------------------------------------------------------#

def append_OUTPUT(output, split=SPLIT_LOG):
    """ append OUTPUT
    """
    if not output:
        return

    if EXEC_SCRIPT_CTX["OUTPUT"]:
        EXEC_SCRIPT_CTX["OUTPUT"] += split + str(output)
    else:
        EXEC_SCRIPT_CTX["OUTPUT"] = str(output)

#------------------------------------------------------------------------------#
# Check and disable flight mode
#
def disable_flight_mode():
    log = "disable_flight_mode(): "
    print_log("INFO", log + "start")

    exec_status, output = PL.Pupdr().Host.commandExecAdb("shell settings get global airplane_mode_on")
    if exec_status != 0:
        print_log("WARNING", log + "unable to get airplane mode status before intent %s" % output)
        return FAILURE
    else:
        if output not in ("0", ""):
            exec_status, output = PL.Pupdr().Host.commandExecAdb("shell ps")
            if exec_status != 0:
                print_log("WARNING", log + "can't get the list of running processes %s" % output)
                return FAILURE
            else:
                if "com.intel.acs.agentv2" in output:
                    if NETWORK_API:
                        print_log("INFO", log + "ACS agent running , sending disabling intent ...")
                        DEVICE.connect_board()
                        NETWORK_API.set_flight_mode("off")
                        DEVICE.disconnect_board()
                    else:
                        print_log("WARNING", log + "NETWORK_API object is None")
                        return FAILURE
                    exec_status, output = PL.Pupdr().Host.commandExecAdb("shell settings get global airplane_mode_on")
                    if exec_status != 0:
                        print_log("WARNING", log + "unable to get airplane mode status after intent {0}".format(output))
                    else:
                        if output != "0":
                            print_log("WARNING", log + "intent fail to change flight mode status")
                            return FAILURE
                else:
                    print_log("WARNING", log + "com.intel.acs.agentv2 not present on device, unable to disable flight mode")
                    return FAILURE
    print_log("INFO", log + "flight mode has been successfully disabled")
    return SUCCESS

 #------------------------------------------------------------------------------#

def getBoardName(exception=False):
    """ Return the ro.hardware property and set the HARDWARE global variable.
    """
    hw = PL.Pupdr().Device.getProperty("ro.hardware")
    hw = str(hw)
    if not hw:
        hw = "unknown"
    print_log("INFO", "Getprop ro.hardware: " + hw)
    return hw

#------------------------------------------------------------------------------#

def getBuildDataFromBoard():
    """ Return the software build and branch name running on the board
    """
    if DEVICE.get_config("pupdrTag", "", str):
        print_log("INFO", "getBuildDataFromBoard() forcing value from bench config")
        build_name = DEVICE.get_config("pupdrTag", "", str)
    else:
        build_name = PL.Pupdr().Device.getProperty("ro.build.version.incremental")
    variant = PL.Pupdr().Device.getProperty("ro.build.type")
    if build_name:
        branch_name = PL.Pupdr().Download.getBranchFromTag(build_name)
    else:
        branch_name = ""
    print_log("INFO", "getBuildDataFromBoard() found: build_name='{0}', branch_name='{1}', variant='{2}'".format(build_name, branch_name, variant))
    return build_name, branch_name, variant

#------------------------------------------------------------------------------#

def getBuildDataFromFlashfiles():
    """ Return the build and branch names from workdir if files exist
    """

    # if value overridden in bench_config
    if DEVICE.get_config("pupdrTag", "", str):
        print_log("INFO", "getBuildDataFromFlashfiles(): found values in bench configuration")
        build_name = DEVICE.get_config("pupdrTag", "", str)
        branch_name = PL.Pupdr().Download.getBranchFromTag(build_name)
        variant =  DEVICE.get_config("pupdrTag", "userdebug", str)

    elif "url_buildinfo" in PROVISIONING_DATA:
        print_log("INFO", "getBuildDataFromFlashfiles(): found values in provisioning data")
        build_name = PL.Pupdr().Download.getBuildTag(PROVISIONING_DATA["url_buildinfo"])
        branch_name = PL.Pupdr().Download.getBranchFromTag(PROVISIONING_DATA["url_buildinfo"])
        variant = PROVISIONING_DATA["build_variant"]
        if not build_name:
            # try getting short build tag from build_info.json
            json_data = PL.Pupdr().Download.getBuildInfoJsonData(PROVISIONING_DATA["url_buildinfo"])
            build_name = json_data.get("buildbot_properties", {}).get("buildbot.props.short_build_number", "")

    elif not FLASH_FILES:
        print_log("WARNING", "getBuildDataFromFlashfiles(): 'FLASH_FILES' variable is empty, cannot determine build name from workdir files")
        build_name = ""
        branch_name = ""
        variant =  ""

    else:
        print_log("INFO", "getBuildDataFromFlashfiles(): finding values in BUT files")
        build_name = ""
        branch_name = ""
        variant = ""
        for single_file in FLASH_FILES.split(";"):
            variant = "userdebug"
            local_fastboot_file = None
            if single_file.endswith(".json"):
                try:
                    with open(single_file) as f:
                        local_json_data = json.load(f)
                        if "localFlashFile" in local_json_data:
                            local_fastboot_file = os.path.basename(local_json_data["localFlashFile"])
                except Exception as e:
                    print_log("INFO", "getBuildDataFromFlashfiles(): cannot parsing json file: {0} ({1})".format(single_file, e))
            if not local_fastboot_file:
                local_fastboot_file = os.path.basename(single_file)
            build_name = PL.Pupdr().Download.getBuildTag(local_fastboot_file)
            branch_name = PL.Pupdr().Download.getBranchFromTag(local_fastboot_file)
            if re.search("(?<=-)[a-z]*(?=-)", local_fastboot_file):
                variant = re.search("(?<=-)[a-z]*(?=-)", local_fastboot_file).group(0)
                if variant not in ["user", "userdebug", "eng"]:
                    variant = "userdebug"
            if build_name and branch_name:
                break

    print_log("INFO", "getBuildDataFromFlashfiles(): found build_name='{0}', branch_name='{1}', variant='{2}'".format(build_name, branch_name, variant))
    return build_name, branch_name, variant

#------------------------------------------------------------------------------#
def apply_expected_fail():
    """ Replace expected result in TC xml
    """
    print_log("INFO", "applying EXPECTED: FAIL")
    EXEC_UC._tc_parameters._attrs_override["TcExpectedResult"] = Util.Verdict.FAIL

#------------------------------------------------------------------------------#

def apply_expected_blocked():
    """ Replace expected result in TC xml
    """
    print_log("INFO", "applying EXPECTED: BLOCKED")
    EXEC_UC._tc_parameters._attrs_override["TcExpectedResult"] = Util.Verdict.BLOCKED

#------------------------------------------------------------------------------#

def getBuildvariant(variant="userdebug"):
    """ Returns buildvariant from json file
    """
    buildvariant = ""

    global PROVISIONING_DATA
    # provision.json
    if not any(element not in PROVISIONING_DATA for element in ["build_target", "build_variant"]):
        build_target = PROVISIONING_DATA["build_target"]
        build_variant = PROVISIONING_DATA["build_variant"]
        buildvariant = build_target + "-" + build_variant
    # ACS model name
    else:
        try:
            if "SF" in DEVICE_NAME:
                # WA for Sofia
                buildvariant = DEVICE_NAME.split("-")[0].capitalize() + "-" + variant
            else:
                buildvariant = DEVICE_NAME.split("-")[0].lower() + "-" + variant
            PROVISIONING_DATA["build_target"] = buildvariant.split("-")[0]
            PROVISIONING_DATA["build_variant"] = variant
        except:
            print_log("WARNING", "getBuildvariant(): DEVICE_NAME is not in proper format: {0}".format(DEVICE_NAME))

    if not buildvariant:
        output = "failure to compute buildvariant"
        print_log("WARNING", output)
        append_OUTPUT(output)
        blocked(EXEC_SCRIPT_CTX["OUTPUT"])
    else:
        print_log("DEBUG", "getBuildvariant(): {0} found".format(buildvariant))


    return buildvariant

#------------------------------------------------------------------------------#

def computeDefaultBoardType():
    """ Returns board_type from DEVICE_NAME
    """
    device = DEVICE_NAME

    try:
        board_type = device.split("-")[0].lower()
    except:
        board_type = ""
        print_log("WARNING", "impossible to extract boardType from '{0}'".format(device))

    # specific rules
    if "mofd" in board_type:
        board_type = "mofd"
    elif "saltbay" in board_type:
        board_type = "saltbay_pr2"
    elif "redhookbay" in board_type:
        board_type = "ctp_pr3"

    print_log("INFO", "computeDefaultBoardType(): '{0}' computed from {1}".format(board_type, device))

    return board_type

#------------------------------------------------------------------------------#

def getBoardType():
    """ Returns board_type
    """
    board_type = ""
    log = "getBoardType(): "
    global PROVISIONING_DATA

    if "board_type" in PROVISIONING_DATA:
        board_type = PROVISIONING_DATA["board_type"]
        print_log("INFO", log + "'{0}' board type found in provisioning json file".format(board_type))

    if not board_type and DEVICE.get_config("pupdrBoardType", "", str):
        board_type = DEVICE.get_config("pupdrBoardType", "", str)
        PROVISIONING_DATA["board_type"] = board_type
        print_log("INFO", log + "'{0}' board type found in bench configuration file".format(board_type))

    # compute default value
    if not board_type:
        board_type = computeDefaultBoardType()
        PROVISIONING_DATA["board_type"] = board_type

    # Check board_type value
    if not board_type:
        append_OUTPUT(log + "board_type is empty")
        blocked(EXEC_SCRIPT_CTX["OUTPUT"])
    PL.Pupdr().Output.appendOutput("", None)
    return board_type

#------------------------------------------------------------------------------#

class TCR_wrapper():
    def __init__(self):
        pass
    @staticmethod
    def resultPush(custom_dict):
        LiveReporting.instance().update_running_tc_info(test_info=custom_dict)
    @staticmethod
    def attachmentPush(local_path, file_name, retention):
        LiveReporting.instance().send_test_case_resource(local_path, display_name=file_name, retention=retention, iteration=False)

#------------------------------------------------------------------------------#

def displayTcGlobalVars():
    print_log("INFO","")
    print_log("INFO","Variables initiated in library:")
    for item in sorted(TC_GLOBAL_VARS):
        if not eval(item) and not isinstance(eval(item), bool):
            print_log("INFO","Var %s value is not set" % item)
        else:
            print_log("INFO","Var %s value is: %s" % (item, eval(item)))

#-----------------------#
# End method of TCs     #
#-----------------------#
def finalStep(direct_exit=False, acs_reconnect=True):
    """ Last method that must be called by each TC script.
        This method contains the common steps done at end of a TC.
    """
    if "finalStep()" in EXEC_SCRIPT_CTX["OUTPUT"]:
        return
    print_log("INFO", "Final step")

    tc_verdict_log = ("SUCCESS" if EXEC_SCRIPT_CTX["VERDICT"] == SUCCESS else "FAILURE")

    finalstep_log = "finalStep(): (TC VERDICT=%s)" % tc_verdict_log
    if finalstep_log not in EXEC_SCRIPT_CTX["OUTPUT"]:
        EXEC_SCRIPT_CTX["OUTPUT"] = PL.Pupdr().Logs.tc_name_number.upper() + " # " + EXEC_SCRIPT_CTX["OUTPUT"]
        append_OUTPUT("TC VERDICT={0}".format(tc_verdict_log), split=" # ")

    # in case of blocked TC, store data
    if NOMOS:
        # System flash is not an test case (but a use case), force verdict PASS
        expected = Util.Verdict.PASS
    else:
        expected = EXEC_UC._tc_parameters.get_tc_expected_result()
    if EXEC_SCRIPT_CTX["VERDICT"] == BLOCKED and expected != Util.Verdict.BLOCKED:
        local_verdict = "BLOCKED"
    else:
        local_verdict = "SUCCESS"
    if EXEC_SCRIPT_CTX["VERDICT"] == BLOCKED:
        PL.Pupdr().Campaign.Campaign_information.updateJsonWithTcInformationData(verdict=local_verdict, output=EXEC_SCRIPT_CTX["OUTPUT"])

    print_log("INFO", finalstep_log)

    if acs_reconnect:
        print_log("INFO", "Reconnect ACS before leaving")
        DEVICE.connect_board()
    #****** End of FINAL EXECUTION  ******#

#-----------------------#
# Initialization of TCs #
#-----------------------#
def setup():
    # print PUPDR code version
    print_log("INFO", "PUPDR Code Version: %s" % CODE_VERSION)

    global GLOBAL_CONF

    # Disconnect ACS
    DEVICE.disconnect_board()
    # use acs function to connect board (specific actions are done for PNP benches)
    IO_CARD.battery_connector(True)
    IO_CARD.usb_host_pc_connector(True)

    # Set TCNAME
    global TCFULLNAME
    TCFULLNAME = str(EXEC_UC._tc_parameters.get_name())
    GLOBAL_CONF["TCFULLNAME"] = TCFULLNAME
    global TCNAME
    TCNAME = os.path.basename(TCFULLNAME)
    GLOBAL_CONF["TCNAME"] = TCNAME

    # Relay card config
    if not 'UsbHostPcConnect' in PL.Pupdr().RelayCard.relayConfiguration or not 'SwitchOnOff' in PL.Pupdr().RelayCard.relayConfiguration:
        output = "SwitchOnOff or UsbHostPcConnect are not set in IO_CARD Equipment in Bench_Config"
        print_log("WARNING", output)
        append_OUTPUT(output)
        PL.Pupdr().Output.appendOutput("", None)
        blocked(EXEC_SCRIPT_CTX["OUTPUT"])

    global PROVISIONING_DATA
    files = FLASH_FILES.split(";")
    provisioning_file = None
    if files and PL.Pupdr().Misc.isExpectedJsonFormat(files[0], ["build_target","build_variant","board_type","url_buildinfo"],top_keys=["provisioning_properties"]):
        provisioning_file = files[0]
    elif files and os.path.isfile(os.path.join(os.path.dirname(files[0]), "provisioning.json")):
        local_file = os.path.join(os.path.dirname(files[0]), "provisioning.json")
        if PL.Pupdr().Misc.isExpectedJsonFormat(local_file, ["build_target","build_variant","board_type","url_buildinfo"],top_keys=["provisioning_properties"]):
            provisioning_file = local_file
    if provisioning_file:
        with open(files[0]) as f:
            json_data = json.load(f)
        for element in json_data["provisioning_properties"]:
            PROVISIONING_DATA[str(element)] = str(json_data["provisioning_properties"][element])
        print_log("INFO", "loading provisioning_properties:\n" +
            "\n".join(["{0:25} {1}".format(element, PROVISIONING_DATA[element]) for element in PROVISIONING_DATA]))
    else:
        print_log("WARNING", "no provisioning json found")
    GLOBAL_CONF["PROVISIONING_DATA"] = PROVISIONING_DATA

    # Set BOARD_TYPE
    global BOARD_TYPE
    BOARD_TYPE = getBoardType()

    global BUILD_NAME, BRANCH_NAME
    BUILD_NAME, BRANCH_NAME, _ = getBuildDataFromFlashfiles()
    GLOBAL_CONF["BUILD_NAME"] = BUILD_NAME
    PL.Pupdr().Configuration.updateConfig(BRANCH_NAME, BOARD_TYPE, print_config=NOMOS,
                                          build_target=GLOBAL_CONF["PROVISIONING_DATA"].get("build_target", ""))
    PL.Pupdr().Campaign.Campaign_information.createCampaignJson()

    # Set BUILDVARIANT
    global BUILDVARIANT
    BUILDVARIANT = getBuildvariant()

    if NOMOS is False:
        # Start is in MOS
        if not PL.Pupdr().OsManager.gotoMos():
            PL.Pupdr().Output.appendOutput("", None)
            blocked(EXEC_SCRIPT_CTX["OUTPUT"])

        # Ensure we are in root
        if not PL.Pupdr().Device.adbRoot():
            PL.Pupdr().Output.appendOutput("", None)
            blocked(EXEC_SCRIPT_CTX["OUTPUT"])

        # Set HARDWARE (ro.hardware)
        global HARDWARE
        HARDWARE = getBoardName(exception=True)

        # check BUILD_NAME, BRANCH_NAME
        local_build_name, local_branch_name, local_variant = getBuildDataFromBoard()
        name_difference = False
        update_config = False
        if local_build_name != BUILD_NAME:
            name_difference = True
            print_log("WARNING", "build name error (from zip file: '{0}', from board: '{1}')".format(BUILD_NAME, local_build_name))
        if local_branch_name != BRANCH_NAME:
            name_difference = True
            update_config = True
            print_log("WARNING", "branch name error (from zip file: '{0}', from board: '{1}')".format(BRANCH_NAME, local_branch_name))
        if name_difference:
            output = "names from flash files and board are not aligned, using data from board"
            print_log("WARNING", output)
            BUILD_NAME = local_build_name
            BRANCH_NAME = local_branch_name
            GLOBAL_CONF["BUILD_NAME"] = BUILD_NAME
        if update_config:
            PL.Pupdr().Configuration.updateConfig(BRANCH_NAME, BOARD_TYPE, build_target=BUILDVARIANT.split("-")[0])

    # wait for eventual OS boot on trail boards (PnP benches starts offline)
    elif PL.Pupdr().Configuration.flash.DEDIPROG and PL.Pupdr().OsManager.getOs(check_charger=False) == "offline":
        PL.Pupdr().Misc.waitDelay("wait for eventual COS boot", PL.Pupdr().Configuration.timeout.COS_BOOT)

    # Set MODEM for hardware without modem
    global MODEM
    MODEM = True
    if BRANCH_NAME == "r44b_sand" and BOARD_TYPE == "saltbay_pr2":
        MODEM = False
        print_log("INFO", "Set MODEM to False on r44b_sand branch")
    elif "NO_MODEM" in DEVICE_NAME:
        MODEM = False
        print_log("INFO", "Set MODEM to False because 'no_modem' keyword in '{0}'".format(DEVICE_NAME))
    else:
        # set board as MODEM if specified in bench_configuration xml
        pupdrParamModemBootCheck = get_pupdrParam("pupdrParamModemBootCheck")
        print_log("INFO", "In Bench_Config, pupdrParamModemBootCheck = "
                      + str(pupdrParamModemBootCheck))
        if not pupdrParamModemBootCheck:
            print_log("INFO", "Force MODEM to False because of pupdrParamModemBootCheck value")
            MODEM = False
    GLOBAL_CONF["MODEM"] = MODEM

    global WORKDIR
    WORKDIR = ""
    files = FLASH_FILES.split(";")
    but = os.path.dirname(files[0]) if files else ""
    if but and os.path.basename(but) == "BUT":
        # for PUPDR farm + pft json files as -f input
        WORKDIR = os.path.dirname(but)
    elif but and os.path.basename(but) == "workdir":
        # for PUPDR farm + provisioning json files as -f input
        WORKDIR = but
    elif but and os.path.basename(but) == "tr_workdir":
        # for buildbot + provisioning json files as -f input
        WORKDIR = but
    elif PL.Pupdr().Misc.isExpectedJsonFormat(files[0], ["build_target","build_variant","board_type","url_buildinfo"],top_keys=["provisioning_properties"]):
        # for standalone users + provisioning json files as -f input
        print_log("INFO", "setup(): using provisioning json directory as download working directory: {0}".format(but))
        WORKDIR = but
    else:
        # on buildbot
        path = PATH_MANAGER.EXECUTION_CONFIG
        print_log("INFO", "setup(): finding 'tr_workdir' from: {0}".format(path))
        found_path = path
        for i in range(len(path.split(os.sep))-1):
            found_path = os.path.abspath(os.path.join(found_path, ".."))
            if os.path.basename(found_path) == "tr_workdir":
                WORKDIR = found_path
                break
    if WORKDIR:
        print_log("INFO", "setup(): WORKDIR={0}".format(WORKDIR))
    else:
        print_log("INFO", "setup(): could not determine working directory")
    GLOBAL_CONF["WORKDIR"] = WORKDIR

    #---------------------------------------------------------------------------
    # check workaround
    if BRANCH_NAME:
        # Remove the _dev tag to search through the workaround.xml
        is_workaround, wa_output = PL.Pupdr().Workaround.isWorkaround(PL.Pupdr().Workaround.TC, name=TCNAME.replace("_dev", ""), check_skip_only=True)
        if is_workaround:
            apply_expected_blocked()
            # Block now to skip TC, expected will be automatically BLOCKED
            PL.Pupdr().Output.appendOutput("", None)
            blocked(wa_output, direct_exit=True)

    # Prepare board after TC WA
    if NOMOS is False:
        # Disable flight mode
        disable_flight_mode()

    # Reset OUTPUT before starting TC
    EXEC_SCRIPT_CTX["OUTPUT"] = ""

    # print info for debug, only in system_flash_local TC
    if NOMOS is True:
        print_log("INFO", "setup(): Display TC globals:")
        displayTcGlobalVars()
    PL.Pupdr().Output.appendOutput("", None)
