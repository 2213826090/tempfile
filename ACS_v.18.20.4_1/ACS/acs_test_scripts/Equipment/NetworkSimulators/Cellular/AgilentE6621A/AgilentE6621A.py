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
:summary: implementation of Agilent E6621A PXT cellular network simulator
:since: 13/01/2012
:author: ssavrimoutou
"""
import weakref
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.VisaEquipment import VisaEquipment
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Interface.ICellNetSim import ICellNetSim
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.AgilentE6621A.Tech4G.Cell4G import Cell4G


class AgilentE6621A(ICellNetSim, VisaEquipment):

    """
    Implementation of Agilent E6621A (PXT) cellular network simulator
    It is based on full python PyVisa library.

    Please read following instructions to implement gpib commands for this equipment:
    - if you need to send a query command use following VisaEquipment method 'query'
    You will have to manage error in your method if needed.

    - if you need to send a write command, use method 'send_command' below.
    It will check errors by reading the response from the equipment.
    In case of PXT equipment it will return the message 'no command'
    else it will return the error message (i.e : 'Invalid header', 'Wrong param type' ...)
    """

    GPIB_SUCCESS = '+0,"No error"'
    """
    The string message returned by the equipment when no error occured
    after sending a gpib command
    """

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        :type name: str
        :param name: the bench configuration name of the equipment
        :type model: str
        :param model: the model of the equipment
        :type eqt_params: dict
        :param eqt_params: the dictionary containing equipment parameters
        :type bench_params: dict
        :param bench_params: the dictionary containing equipment bench parameters
        """
        # Initialize class parent
        ICellNetSim.__init__(self)
        VisaEquipment.__init__(self, name, model, eqt_params, bench_params)

        # Initialize attributes
        self.__bench_params = bench_params
        self.__cell4g = None
        self.__alt_cell4g = None
        self._status = None

        # Initialize features
        self.__init_features()

    def __del__(self):
        """
        Destructor: releases all allocated resources.
        """
        self.release()
        del self.__cell4g
        del self.__alt_cell4g

    def __init_features(self):
        """
        Initializes features according to equipment catalog parameters
        """
        features = self.get_eqt_dict()[self.get_model()]["Features"]
        features_name = features.keys()
        if "LTE" in features_name:
            if features["LTE"] == "enable":
                self.__cell4g = Cell4G(weakref.proxy(self))
                self.__alt_cell4g = Cell4G(weakref.proxy(self), "B")

    def __error_check(self, msg):
        """
        Error checking and warning reporting
        :raise TestEquipmentException: if err != SUCCESS
        """
        if msg != AgilentE6621A.GPIB_SUCCESS:
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, msg)

    def init(self):
        """
        Initializes the equipment. Final state: the equipment is ready to use.
        """
        self.connect()

    def release(self):
        """
        Releases the connection with the equipment and all resources allocated
        to its use.
        """
        self.disconnect()

    def get_cell_4g(self):
        """
        Access to LTE cellular interface.
        :rtype: Cell4G
        :return: the 4G cellular object.
        """
        if self.__cell4g is not None:
            return self.__cell4g
        msg = "LTE is not available"
        self.get_logger().error(msg)
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, msg)

    def get_alt_cell_4g(self):
        """
        Access to the alternative LTE cellular interface.

        note: The Agilent support second 4g cell emulation at same time
        :rtype: Cell4G
        :return: the 4G cellular object.
        """
        if self.__alt_cell4g is not None:
            return self.__alt_cell4g
        msg = "Alt LTE is not available"
        self.get_logger().error(msg)
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, msg)

    def perform_full_preset(self):
        """
        Resets all equipment parameters to defaults.
        """
        self.get_logger().info("Perform full preset")
        self.send_command("STAT:PRES")

    def send_command(self, command):
        """
        Send a GPIB Command.

        :type command: str
        :param command: String representing a well formed GPIB command.
        """
        # Send command *CLS to the equipment to clear the Error Queue
        self.get_logger().debug("Send GPIB command: *CLS")
        self.write("*CLS")

        self.get_logger().debug("Send GPIB command: %s" % command)
        # Send command to the equipment
        self.write(command)
        # Check errors in the System Error buffer
        return_msg = self.query("SYST:ERR?")

        # Return the error message
        self.__error_check(return_msg)

    def query_command(self, command):
        """
        Send a GPIB Command.

        :type command: str
        :param command: String representing a well formed GPIB command.

        :rtype: String
        :return: Message from equipment giving the value of a dedicated GPIB command
        """
        # Send command *CLS to the equipment to clear the Error Queue
        self.get_logger().debug("Send GPIB command: *CLS")
        self.write("*CLS")

        self.get_logger().debug("Send GPIB command: %s" % command)

        # Send command to the equipment
        response = self.query(command)
        # Check errors in the System Error buffer
        return_msg = self.query("SYST:ERR?")

        # Return the error message if present and different than "No Error"
        self.__error_check(return_msg)

        return response

    def load_configuration_state_file(self, filename):
        """
        Loads equipment parameters from a file.
        @type filename: string
        @param filename: the configuration file to load with his path
        """
        self.get_logger().info("Load configuration state file : %s" % filename)
        self.send_command('REG:REC:FILE "%s"' % filename)


    def load_configuration_file(self, filename):
        """
        Loads equipment parameters from a file.
        :type filename: str
        :param filename: the configuration file to load with his path
        """
        # to simplly code, deleted calling "load_configuration_state_file function",
        # as this function is called in different places such in use case directy LTE_BASE and in
        # RegistratinUtilites.py to Load the scenario state file as our lab could not modify .lbmf file
        file_name_len = len(filename)
        self.get_logger().info("Load configuration state file : %s" % filename)
        if (filename[file_name_len - 3:file_name_len]).lower() == "xml":
            self.send_command('REG:REC:FILE "%s"' % filename)
        else:
            self.send_command('SCENA:LOAD "%s"' % filename)

    def start_scenario(self):
        """
        Start the scenario
        """
        self.get_logger().info("Start scenario")
        self.send_command("BSE:SIMUL RUN")

    def stop_scenario(self):
        """
        Stop the scenario
        """
        self.get_logger().info("Stop scenario")
        self.send_command("BSE:SIMUL STOP")

    def load_cell_config(self, config_type, eqt_id=1):
        """
        Configure equipment to specific cells via .xml files
        in order to reach specific throughput

        :type config_type: str
        :param config_type: the type of configuration(band, category, etc) to load

        :type eqt_id: int
        :param eqt_id: Equipment number
        """
        pass

    def get_app_format(self):
        """
        Gets the application format of the network simulator

        :rtype: str
        :return: the str representation of the SMS send state.
            Possible returned values:
                - "LTE FDD"
                - "LTE TDD"
        """
        self.get_logger().info("Get Application Format")
        app_format = self.query_command("SYST:APPL:FORM?")

        return app_format

    def switch_app_format(self, app_format):
        """
        Switches application format of the network simulator
        :type format: str
        :param format: the desired application format. Possible values:
                - "LTE FDD"
                - "LTE TDD"
        """
        # Get current application format
        self.get_logger().info("Get Application Format")
        current_format = self.query_command("SYST:APPL:FORM?")
        # Switch application if necessary
        if app_format not in current_format:
            self.get_logger().info(
                "Switch application format to %s",
                app_format)
            self.send_command("SYST:APPL:FORM %s" % app_format)

    def send_ho_message(self, number):
        """
        Send Handover message
        :type number: str
        :param number: the desired handover message to send.
        Possible values:
                - 1 : DL info CS Notify
                - 2 : B1 RRC Handover A to B
                - 3 : B3 RRC Handover A to B
                - 4 : B13 RRC Handover A to B
                - 5 : B17 RRC Handover A to B
                - 6 : PS HO To WCDMA
                - 7 : Blind HO
                - 8 : SRVCC To WCDMA
        """
        self.send_command("BSE:FUNC:HANDO:MESSA%s:SEND" % number)

    def configure_amplitude_offset_table(self, frequency_list=None, offset_list=None, second_antenna_offset_list=None):
        """
        Configures the amplitude offset table to compensate cable loss.
        :type frequency_list: str
        :param frequency_list: the frequency list.
        :type offset_list: str
        :param offset_list: the offset list corresponding to the frequency
        :type second_antenna_offset_list: str
        :param second_antenna_offset_list: the offset list corresponding to the frequency
        listed above for the diverse antenna.
        """
        pass

    def apply_bench_config(self, bench_config=None):
        """
        Applies configuration provided in the Bench Config
        :type bench_config: BenchConfigParameters
        :param bench_config: Bench config parameters
        """
        self.get_logger().info("apply_bench_config function is stubbed for AgilentE6621A")
        pass

    def set_ip4_default_gateway(self, gateway):
        """
        Sets default IPv4 gateway address
        :type gateway: str
        :param gateway: the gateway to set. 15 characters formatted
        as follows: A.B.C.D where A = 0 to 223 B,C,D = 0 to 255
        (no embedded spaces).
        """
        self.get_logger().info("set_ip4_default_gateway function is stubbed for AgilentE6621A")
        pass

    def set_ip4_subnet_mask(self, mask):
        """
        Sets IPv4 subnet mask
        :type mask: str
        :param mask: the subnet mask to set. 15 characters formatted
        as follows: A.B.C.D where A,B,C,D are between = 0 to 255
        (no embedded spaces).
        """
        self.get_logger().info("set_ip4_subnet_mask function is stubbed for AgilentE6621A")
        pass
