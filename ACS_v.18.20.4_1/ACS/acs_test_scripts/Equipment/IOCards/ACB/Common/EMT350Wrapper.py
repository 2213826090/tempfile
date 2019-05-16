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

:organization: INTEL MCG PSI
:summary: Wrapper for EMT350 equipment function
:author: vgomberx
:since: 15/01/2014
"""
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.IOCards.ACB.Common.EMT350Cards import USB_CARD, POWER_CARD
from acs_test_scripts.Equipment.IOCards.ACB.Common.EMTClient import EMTClient
from acs_test_scripts.Equipment.IOCards.ACB.Common.SerialComms import SerialComms
from acs_test_scripts.Equipment.IOCards.Interface.IIOCard import IIOCard
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT
import sys, time


class EMT350Wrapper():
    __CST_CONNECT = ["CONNECT", "DISCONNECT"]
    __CST_RES_TYPE = ["PULL_UP", "PULL_DOWN"]
    __CST_BATTERY_TYPE = ["ANALOG", "DIGITAL", "INVALID", "DIG_INVALID", "NONE"]
    __CTS_BATTERY_TYPE_FOR_EMT350 = {"ANALOG": "ANA_VALID",
                                    "INVALID": "ANA_INVALID",
                                    "DIGITAL": "DIG_VALID",
                                    "DIG_INVALID": "DIG_INVALID",
                                    "NONE": "NONE"}
    __CTS_SUPPORTED_DEVICE_TYPE = [IIOCard.SDP, IIOCard.DCP, IIOCard.AC_CHGR, IIOCard.OTG, IIOCard.CDP, IIOCard.ACA,
                                   "NONE"]
    __CST_PLATFORM = ["1", "2", "_NONE"]
    __TAG = "EMT350Wrapper\t"

    __DAUGTHER_BOARD_COMMS_OK = "DAUGHTER_BOARD-COMMUNICATION_OK"
    __USB_RETURN_CODE = {'MOTHER_BOARD-END_SYNTAX_INVALID': 'A frame with an invalid end was send to the EMT350',
                         'MOTHER_BOARD-WRONG_DAUGHTER_BOARD': 'A command was send to the EMT350 wrong daughter board',
                         'MOTHER_BOARD-NO_DAUGHTER_BOARD_CONNECTED': 'No daughter board is connected on the EMT 350',
                         'MOTHER_BOARD-SLOT_INVALID': 'Frame send to an invalid slot',
                         'MOTHER_BOARD-PARAMETER_1_INVALID': 'The first parameter is invalid',
                         'MOTHER_BOARD-PARAMETER_2_INVALID': 'The second parameter is invalid',
                         'MOTHER_BOARD-PARAMETER_3_INVALID': 'The third parameter is invalid',
                         'MOTHER_BOARD-PARAMETER_4_INVALID': 'The fourth parameter is invalid',
                         'MOTHER_BOARD-COMMAND_INVALID': 'Frame contain an invalid command',
                         'MOTHER_BOARD-ALERT TEMPERATURE! SLOT 1 BLOCKED': 'Card 1 Temperature is in red temperature zone',
                         'MOTHER_BOARD-ALERT TEMPERATURE! SLOT 2 BLOCKED': 'Card 2 Temperature is in red temperature zone',
                         'MOTHER_BOARD-ALERT TEMPERATURE! SLOT 3 BLOCKED': 'Card 3 Temperature is in red temperature zone',
                         'MOTHER_BOARD-ALERT TEMPERATURE! SLOT 4 BLOCKED': 'Card 4 Temperature is in red temperature zone',
                         'MOTHER_BOARD-WARNING TEMPERATURE FOR SLOT 1': 'Card 1 Temperature is in orange temperature zone',
                         'MOTHER_BOARD-WARNING TEMPERATURE FOR SLOT 2': 'Card 2 Temperature is in orange temperature zone',
                         'MOTHER_BOARD-WARNING TEMPERATURE FOR SLOT 3': 'Card 3 Temperature is in orange temperature zone',
                         'MOTHER_BOARD-WARNING TEMPERATURE FOR SLOT 4': 'Card 4 Temperature is in orange temperature zone',
                         __DAUGTHER_BOARD_COMMS_OK :'EMT350 Communication is OK',
                         'DAUGHTER_BOARD-COMMUNICATION_ERROR':'EMT350 Communication is NOK'}


    def __init__(self):
        """
        Init client var
        """
        self.__ac_card = None
        self.__batt_card = None
        self.__usb_card = None
        self.__com_object = None
        self.__card_conf = None
        # set an init value different from None to force an usb unplug then plug
        self.__last_usb_plugged = "init_value_different_from_none"
        # this var track the time between 2 command to avoid overflowing the serial Comms
        self.__last_command_date = 0

    def __del__(self):
        """
        ensure that communication is closed when this object is destroy
        """
        if self.__com_object is not None:
            self.__com_object.release()
            del self.__com_object

    def init(self, card_configuration):
        """
        check that all card have been well affected

        :type card_configuration: dict of dict
        :param card_configuration: card configuration found on your benchconfig
               and cleaned after comparison with device info
        """
        if len(card_configuration) == 0 or not (type(card_configuration) is dict):
            msg = "problem with your io card configuration, cant read card configuration from benchconfig"
            LOGGER_TEST_SCRIPT.error(self.__TAG + msg)
            raise TestEquipmentException(
                TestEquipmentException.READ_PARAMETER_ERROR, msg)

        # at this level, the card_configuration should be cleaned
        # from any potential error like duplicate output
        for slot in card_configuration.keys():
            # evaluate power cards
            if card_configuration[slot].get("TYPE") == POWER_CARD.TYPE:
                for element in card_configuration[slot].keys():
                    if POWER_CARD.OUTPUT_EMU_BATT in element:
                        self.__batt_card = {"SLOT": slot, "PORT": card_configuration[slot][element]}
                        self.DaughterBoardReset(self.__batt_card, "POWER_BOARD")
                        break

                for element in card_configuration[slot].keys():
                    if POWER_CARD.OUTPUT_AC in element:
                        self.__ac_card = {"SLOT": slot, "PORT": card_configuration[slot][element]}
                        break

            # evaluate usb card
            elif card_configuration[slot].get("TYPE") == USB_CARD.TYPE:
                for element in card_configuration[slot].keys():
                    if USB_CARD.OUTPUT_USB in element:
                        self.__usb_card = {"SLOT": slot, "PORT": card_configuration[slot][element]}
                        self.DaughterBoardReset(self.__usb_card, "USB_BOARD")
                        break

        # no card is set we can assume that this io card cant be used
        # maybe raise an error here
        # call super function
        self.__card_conf = card_configuration

    def release(self):
        """
        release equipment connection
        """
        if self.__com_object is not None:
            self.__com_object.release()

    def init_multi_execution(self, server_ip, server_port, com_port, baud_rate, retry_nb):
        """
        use a client/server system for multi use

        :type server_ip: str
        :param server_ip: address of the server
        :type server_port: int
        :param server_port: server port

        :type com_port: int
        :param com_port: com port number to connect with
        :type baud_rate: int
        :param baud_rate: baud rate for the serial connection
        :type retry_nb: int
        :param retry_nb: Number of serial connection attempt
        """
        obj = EMTClient()
        obj.init(self.__card_conf, server_ip, server_port, com_port, baud_rate, retry_nb)
        self.__com_object = obj

    def init_local_execution(self, com_port, baud_rate, retry_nb):
        """
        use simple serial mode

        :type com_port: int
        :param com_port: com port number to connect with
        :type baud_rate: int
        :param baud_rate: baud rate for the serial connection
        :type retry_nb: int
        :param retry_nb: Number of serial connection attempt
        """
        obj = SerialComms()
        obj.init(com_port, baud_rate, retry_nb)
        self.__com_object = obj

    def send_cmd(self, cmd, card="", restart_com=True):
        """
        format your command before sending it to equipment.
        if the communication is closed, restart it.

         :type card: int
         :param card: card slot request to format the cmd, may be empty in some case

         :type cmd: str
         :param cmd: command to send to the equipment

         :type restart_com: boolean
         :param restart_com: if true, will restart the server if it does not respond

        """
        slot = card
        if card is None:
            name = sys._getframe(1).f_code.co_name
            LOGGER_TEST_SCRIPT.error(self.__TAG + "A card has not been specified, skipping function: %s" % name)
        else:

            # all parameters after the command will be in upper case
            try:
                time_to_wait = time.time() - self.__last_command_date
                if time_to_wait < 2:
                    sleep_time = 2 - time_to_wait
                    LOGGER_TEST_SCRIPT.debug(self.__TAG + "waiting %ss before sending command to avoid overflowing comms port" % str(sleep_time))
                    time.sleep(sleep_time)

                # format the command
                if ":" in cmd:
                    split_cmd = cmd.split(":", 1)
                    cmd = split_cmd[0] + ":" + split_cmd[1].upper()
                if type(card) is dict and card.get("SLOT"):
                    slot = str(card["SLOT"]).strip()

                if card != "":
                    formated_cmd = "%s%s;*" % (slot, cmd.strip())
                else:
                    formated_cmd = "%s;*" % (cmd.strip())

                if not self.__com_object.is_communication_up() and restart_com:
                    self.__com_object.open_communication()

                # send the command
                LOGGER_TEST_SCRIPT.debug(self.__TAG + "sending command [%s] to EMT350" % formated_cmd)
                res = self.__com_object.communicate(formated_cmd).replace('\n', "").strip()
                self.__last_command_date = time.time()

                # compute response
                LOGGER_TEST_SCRIPT.debug(res)
                if res in EMT350Wrapper.__USB_RETURN_CODE.keys():
                    LOGGER_TEST_SCRIPT.debug(self.__TAG + EMT350Wrapper.__USB_RETURN_CODE[res])
                    if res != self.__DAUGTHER_BOARD_COMMS_OK:
                        # WARNING : exception not raised yet because of the emt350 instability
                        raise TestEquipmentException(TestEquipmentException.COMMAND_LINE_ERROR, EMT350Wrapper.__USB_RETURN_CODE[res])
                return res

            except Exception as e:
                error = self.__TAG + "error happen during communication with server : %s" % str(e)
                LOGGER_TEST_SCRIPT.error(error)
                raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, error)

    def get_last_usb_plugged(self):
        """
        return the type of the last usb cable plugged

        :rtype: string
        :return: DCP, SDP, CDP , OTG, ACA
        """
        return self.__last_usb_plugged

#    BOARD FUNCTIONS
# command should be simplified for user, like avoid having weird name for slot for example
# you should prefer to use int instead of str when possible
# you should add robustness to catch bad format when possible to avoid
    def SwitchPlatform(self, card, platform):
        """
        switch to a given output of a platform

        :type card: dict
        :param card: contain the information of a given card

        :type platform: int
        :param platform: the output to switch on, limited by the number of output your daughter card support
        """
        if str(platform) not in EMT350Wrapper.__CST_PLATFORM:
            msg = "unknown platform type, can only be %s" % str(EMT350Wrapper.__CST_PLATFORM)
            LOGGER_TEST_SCRIPT.error(self.__TAG + msg)
            raise TestEquipmentException(
                TestEquipmentException.READ_PARAMETER_ERROR, msg)
        cmd = "HwControlPlatformConnect:PLATFORM%s" % (str(platform))
        self.send_cmd(cmd, card)
        self.__last_usb_plugged = None

    def GlobalCardReset(self):
        """
        reset all the tester

        """
        cmd = "S0TesterReset"
        self.send_cmd(cmd)

    def DaughterBoardReset(self, card, CardType):
        """
        Reset a selected Daughter Board
        :type CardType: str
        :param CardType: USB_BOARD or POWER_BOARD

        """
        cmd = "BoardFunctionReset:%s" % (CardType)
        self.send_cmd(cmd, card)

    def GetPlatform(self, board):
        """
        return the platform number used by a board of a slot

        :type slot: str
        :param slot: POWER or USB
        :rtype: str
        :rreturn: Platform value

        """
        if board is not None:
            platform = board.get("PORT")
        else:
            msg = "GetPlatform: no board declared"
            LOGGER_TEST_SCRIPT.error(self.__TAG + msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)
        return str(platform).upper().strip()

    def BoardFunctionAcConnect(self, action):
        """
        Connect or disconnect an external power to replace the 5V of AC charger (tablet only)

        :type action: str
        :param action: CONNECT or DISCONNECT
        """
        action = action.upper()
        if action not in EMT350Wrapper.__CST_CONNECT:
            msg = "BoardFunctionAcConnect: bad value for parameter [action] can only be :%s" % (str(EMT350Wrapper.__CST_CONNECT))
            LOGGER_TEST_SCRIPT.error(self.__TAG + msg)
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER, msg)

        cmd = "BoardFunctionAcConnect:%s" % (action)
        self.send_cmd(cmd, self.__ac_card)

    def BoardFunctionUsbDisconnect(self):
        """
        do usb Disconnection

        :No Param : PLATFORM_NONE, NO_PWR, RESISTANCE, NONE
        """
        LOGGER_TEST_SCRIPT.debug(self.__TAG + "Disconnecting USB ...")
        # Keep the connected platform
        cmd = "BoardFunctionUsbSelect:PLATFORM%s;NO_PWR;RESISTANCE;NONE" % (self.GetPlatform(self.__usb_card))
        self.send_cmd(cmd, self.__usb_card)
        self.__last_usb_plugged = None

    def BoardFunctionUsbSelect(self, usb_id_mode, path_supply, raw_path_usb=None, platform="default"):
        """
        do usb connection

        :type platform: str
        :param platform: platform1,
                        platform2,
                        none - to disconnect all

        :type path_usb: str
        :param path_usb: dcp, sdp, cdp, otg, aca, none

        :type usb_id_mode: str
        :param usb_id_mode: through, resistance, platform_host, platform_device

        :type path_supply: str
        :param path_supply: ext - USB power supply is coming from external source
                           usb - USB power supply is coming from itself
                           none - no power supply
        """

        # if platform set to default then get the usb card port value for usb switching
        if str(platform).lower().strip() == "default":
            platform = self.GetPlatform(self.__usb_card)

        formated_platform = platform
        if formated_platform.isdigit():
            formated_platform = "PLATFORM" + formated_platform

        if formated_platform not in ["PLATFORM1", "PLATFORM2", "PLATFORM_NONE"]:
            msg = "BoardFunctionUsbSelect: bad value %s for parameter[platform] can only be :1, 2 or platform_none" % platform
            LOGGER_TEST_SCRIPT.error(self.__TAG + msg)
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER, msg)

        path_usb = str(raw_path_usb).upper()
        if path_usb not in self.__CTS_SUPPORTED_DEVICE_TYPE:
            msg = "BoardFunctionUsbSelect: bad value for parameter [path_usb] can only be :1, 2 DCP, SDP, CDP, OTG, ACA, none"
            LOGGER_TEST_SCRIPT.error(self.__TAG + msg)
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER, msg)

        usb_id_mode = str(usb_id_mode).upper()
        if usb_id_mode not in ["THROUGH", "RESISTANCE", "PLATFORM_HOST", "PLATFORM_DEVICE"]:
            msg = "BoardFunctionUsbSelect: bad value for parameter [usb_id_mode] can only be : RESISTANCE, PLATFORM_HOST, PLATFORM_DEVICE"
            LOGGER_TEST_SCRIPT.error(self.__TAG + msg)
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER, msg)

        path_supply = str(path_supply).upper()
        if path_supply not in ["EXT_SUPPLY", "USB_SUPPLY", "NO_PWR"]:
            msg = "BoardFunctionUsbSelect: bad value for parameter [path_supply] can only be : EXT_SUPPLY, USB_SUPPLY, NO_PWR , not %s" % path_supply
            LOGGER_TEST_SCRIPT.error(self.__TAG + msg)
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER, msg)

        cmd = "BoardFunctionUsbSelect:%s;%s;%s;%s" % (formated_platform, path_supply, usb_id_mode, path_usb)
        self.send_cmd(cmd, self.__usb_card)
        self.__last_usb_plugged = raw_path_usb

    def HwControlPressPowerButton(self, delay):
        """
        Push the Power Button

        :type delay: int
        :param delay: in milliseconds.
                      Power Button value is commanded by a timer.
                      Put the push button time in milliseconds
        """

        cmd = "HwControlPressOnOffButton:%d" % (delay)
        self.send_cmd(cmd, self.__usb_card)
        # Permits to wait the command execution
        LOGGER_TEST_SCRIPT.debug(self.__TAG + "Wait %s seconds..." % (delay / 1000))
        time.sleep((delay / 1000))

    def BoardFunctionBatteryConnect(self, action, bptherm_res, battid_res, battid_type):
        """
        Connect the battery

        :type action: str
        :param action:   Select the connetion or disconnetion

        :type bptherm_res: int
        :param bptherm_res:   Value of the Bptherm resistance

        :type battid_res: int
        :param battid_res:   Value of the Battid resistance

        :type battid_type: str
        :param battid_type:   Type of the battID (ANALOG, INVALID, DIGITAL, DIG_INVALID)

        """

        # Check the BattID type
        action = action.upper()
        if action not in EMT350Wrapper.__CST_CONNECT:
            msg = "BoardFunctionBatteryConnect: bad value for parameter [action] can only be :%s" % \
                  (str(EMT350Wrapper.__CST_CONNECT))
            LOGGER_TEST_SCRIPT.error(self.__TAG + msg)
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER, msg)

        # Check the Action
        battid_type = str(battid_type).upper()
        LOGGER_TEST_SCRIPT.info(self.__TAG + "     BATTERY_TYPE %s" % battid_type)
        if battid_type not in self.__CST_BATTERY_TYPE:
            msg = "BoardFunctionBattConnect: bad value for parameter [battid_type] can only be : ANALOG, INVALID, DIGITAL, DIG_INVALID"
            LOGGER_TEST_SCRIPT.error(self.__TAG + msg)
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER, msg)

        # Check the BattID Value
        if not POWER_CARD.BATTID_MIN <= battid_res <= POWER_CARD.BATTID_MAX:
            txt = "your io card does not support this battid value (%d) " % battid_res
            LOGGER_TEST_SCRIPT.warning(txt)

        # Check the BPTherm value
        if not POWER_CARD.BPTERM_MIN <= bptherm_res <= POWER_CARD.BPTERM_MAX:
            txt = "your io card does not support this battid value (%d) " % bptherm_res
            LOGGER_TEST_SCRIPT.warning(txt)

        # Correspondance for BattID Type
        ID_type = self.__CTS_BATTERY_TYPE_FOR_EMT350[battid_type]

        if action == "CONNECT":
            self.SwitchPlatform(self.__batt_card, self.GetPlatform(self.__batt_card))

        # Send the command
        cmd = "BoardFunctionBattConnect:%s;%d;%d;%s" % (action, bptherm_res, battid_res, ID_type)
        self.send_cmd(cmd, self.__batt_card)
