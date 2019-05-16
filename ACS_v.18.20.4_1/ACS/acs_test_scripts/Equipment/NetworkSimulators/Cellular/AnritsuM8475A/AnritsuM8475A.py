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

:organization: INTEL QCTV
:summary: implementation of Anritsu M8475A cellular network simulator
:since: 20/03/2015
:author: gcharlex
"""
import os

from Core.PathManager import Paths
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.TestEquipmentException import TestEquipmentException
from Core.Report.ACSLogging import ACS_LOGGER_NAME, EQT_LOGGER_NAME
from Core.Factory import Factory

from acs_test_scripts.Equipment.NetworkSimulators.Cellular.AnritsuM8475A.Tech2G.Cell2G import Cell2G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.AnritsuM8475A.Tech3G.Cell3G import Cell3G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.AnritsuM8475A.Tech4G.Cell4G import Cell4G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Interface.ICellNetSim import ICellNetSim
from acs_test_scripts.TestStep.Utilities.Visa import VisaInterface, VisaEquipmentBase
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.AnritsuM8475A.AnritsuM8475AErrorCodes import AnritsuM8475A_ErrorCodes
from acs_test_scripts.Utilities.RegistrationUtilities import ImsConfigGenerator
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Database.CommandsDatabase import CommandsDatabase


class AnritsuM8475A(ICellNetSim, VisaEquipmentBase):
    """
    Implementation of Anritsu M8475A cellular network simulator
    It is based on full python PyVisa library.
    """
    DB_FILE = os.path.join(ICellNetSim.DATABASE_DIR, "commands.db")
    _CELLULAR_PDN_NUMBER = 1
    _CELLULAR_PDN_NUMBER_2 = 3
    _IMS_PDN_NUMBER = 2
    _MAX_PDN_NUMBER = 9
    _IMS_CONFIG_GPIB = {"Enabled": "IMSCSCF ",
                        "Host name": "IMSCSCFNAME ",
                        "IP Version": "IMSCSCFIPTYPE ",
                        "IP Address (IPv4)": "IMSCSCFIPV4 ",
                        "IP Address (IPv6)": "IMSCSCFIPV6 ",
                        "Port": "IMSCSCFPORT ",
                        "Monitoring UA": "IMSCSCFUAURI ",
                        "SMSC Auto Forward": "IMSCSCFSMSCAUTOFORWARD ",
                        "IMS Authentication": "IMSCSCFAUTH ",
                        "User List": "IMSCSCFUSERSLISTSIZE?",
                        "Virtual UA Enabled": "IMSCSCFVUA ",
                        "Virtual UA": "IMSCSCFVUAURI ",
                        "Max-Expires": "IMSCSCFMAXEXPIRETIME ",
                        "Min-Expires": "IMSCSCFMINEXPIRETIME ",
                        "Communication Forwarding": "IMSCSCFCFENABLED ",
                        "Forwarding Destination URI": "IMSCSCFCFADDRESS ",
                        "Originating ID": "IMSCSCFORIGID ",
                        "Terminating ID": "IMSCSCFTERMID ",
                        "Precondition": "IMSCSCFPRECONDITION "}

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
        VisaEquipmentBase.__init__(self, name, model, eqt_params, bench_params)

        # Initialize attributes
        self.__bench_params = bench_params
        self.__cells = []
        self._vnid = None

    def init(self):
        """
        Initializes the equipment. Final state: the equipment is ready to use.
        """
        self._visa.connect()
        self._visa.timeout = 10000

    def release(self):
        """
        Releases the connection with the equipment and all resources allocated
        to its use.
        """
        self._visa.disconnect()

    def create_visa_interface(self, transport, kwargs):
        """
        Create visa interface
        :type transport: str
        :param transport: the bench configuration name of the equipment
        :type kwargs: dict
        :param kwargs: various needed parameters depending on the transport
        """
        self._visa = VisaInterfaceAnritsuM8475A(transport, **kwargs)

    def set_cells_technology(self, cells_tech):
        """
        Sets cells technology used by the equipment

        :type cells_tech: List[str]
        :param cells_tech: List of cell technology (2G|GSM, 3G|WCDMA, TDSCDMA, 4G|LTE)
        """
        self.__cells = []
        for (cell_number, cell_tech) in enumerate(cells_tech):
            if cell_tech in ["4G", "LTE"]:
                self.__cells.append(Cell4G(self._visa, cell_number + 1))
            elif cell_tech in ["3G", "WCDMA"]:
                self.__cells.append(Cell3G(self._visa, cell_number + 1))
            elif cell_tech in ["2G", "GSM"]:
                self.__cells.append(Cell2G(self._visa, cell_number + 1))
            else:
                error_msg = "Wrong value for Cell tech {0} (Possible values: 2G|GSM, 3G|WCDMA, 4G|LTE)"
                raise AcsBaseException(AcsBaseException.INVALID_PARAMETER, error_msg)
        self.perform_full_preset()
        self.config_simulation()

    def config_simulation(self):
        """
        Sets the Simulation Model
        :return:
        """
        if len(self.__cells) > 0:
            gpib_command = "SIMMODEL "
            cells_tech = [cell.tech for cell in self.__cells]
            gpib_param = ", ".join(cells_tech)
            gpib_command += gpib_param
            self._visa.send_command(gpib_command)
            gpib_command = "SIMMODELEX RETEUTRAN,ENABLE"
            self._visa.send_command(gpib_command)
        else:
            error_msg = "Trying to set a simulation model with 0 cell"
            AcsBaseException(AcsBaseException.OPERATION_FAILED, error_msg)

    def apply_bench_config(self, bench_config):
        """
        Applies configuration provided in the Bench Config
        :type bench_config: BenchConfigParameters
        :param bench_config: Bench config parameters
        """
        # Simulator IP Config
        # Should be made first as it reset PDN information
        available_params = self._bench_params.get_dict()
        # Subnet_Mask
        if "Subnet_Mask" in available_params:
            subnet_mask = self._bench_params.get_param_value("Subnet_Mask")
            self.set_ip4_subnet_mask(subnet_mask)
        # Default Gateway
        if "Default_Gateway" in available_params:
            default_gateway = self._bench_params.get_param_value("Default_Gateway")
            self.set_ip4_default_gateway(default_gateway)
            default_gateway = self._bench_params.get_param_value("Default_Gateway_IPV6")
            self.set_ip6_default_gateway(default_gateway)

        # Cellular PDN
        cellular_network_params = bench_config.get_parameters("CELLULAR_NETWORK")
        available_params = cellular_network_params.get_dict()

        # PDN APN
        # Set two default PDN because IMS APN can be rejected for test purpose by IMS PDN and take instead one default PDN
        if "APN" in available_params:
            apn_pdn = cellular_network_params.get_param_value("APN")
            self.set_pdn_apn(AnritsuM8475A._CELLULAR_PDN_NUMBER, apn_pdn)
            self.set_pdn_apn(AnritsuM8475A._CELLULAR_PDN_NUMBER_2, apn_pdn)
        # DUT IPV Address
        if "DUT_IP_ADDRESS" in available_params:
            ipv4_address = cellular_network_params.get_param_value("DUT_IP_ADDRESS")
            self.set_pdn_ipv4_address(AnritsuM8475A._CELLULAR_PDN_NUMBER, ipv4_address)
            self.set_pdn_ipv4_address(AnritsuM8475A._CELLULAR_PDN_NUMBER_2, ipv4_address)
        # DUT IPV6 Address
        if "DUT_IPV6_ADDRESS" in available_params:
            ipv6_address = cellular_network_params.get_param_value("DUT_IPV6_ADDRESS")
            self.set_pdn_ipv6_address(AnritsuM8475A._CELLULAR_PDN_NUMBER, ipv6_address)
            self.set_pdn_ipv6_address(AnritsuM8475A._CELLULAR_PDN_NUMBER_2, ipv6_address)

        # IMS PDN
        ims_network_params = bench_config.get_parameters("IMS_NETWORK")
        available_params = ims_network_params.get_dict()
        self.set_pdn_ims(AnritsuM8475A._IMS_PDN_NUMBER, True)
        # PDN APN
        if "APN" in available_params:
            apn_pdn = ims_network_params.get_param_value("APN")
            self.set_pdn_apn(AnritsuM8475A._IMS_PDN_NUMBER, apn_pdn)
        # DUT IPV4 Address
        if "DUT_IP_ADDRESS" in available_params:
            ipv4_address = ims_network_params.get_param_value("DUT_IP_ADDRESS")
            self.set_pdn_ipv4_address(AnritsuM8475A._IMS_PDN_NUMBER, ipv4_address)
        # DUT IPV6 Address
        if "DUT_IPV6_ADDRESS" in available_params:
            ipv6_address = ims_network_params.get_param_value("DUT_IPV6_ADDRESS")
            self.set_pdn_ipv6_address(AnritsuM8475A._IMS_PDN_NUMBER, ipv6_address)
        # IMS VNID
        if "VNID" in available_params:
            self._vnid = ims_network_params.get_param_value("VNID")
            self.set_pdn_vnid(AnritsuM8475A._IMS_PDN_NUMBER, self._vnid)

    def configure_ims(self, bench_config):
        """
        Applies IMS configuration provided in the Bench Config
        :type bench_config: BenchConfigParameters
        :param bench_config: Bench config parameters
        """
        # Check if IMS virtual network is running
        ims_vn_status = self._visa.query("IMSVNSTAT? " + str(self._vnid))
        # If yes stop it to configure it
        if ims_vn_status == "RUNNING":
            self._visa.send_command("IMSSTOPVN " + str(self._vnid))
        # Retrieve IMS configuration for Anritsu MD8475A from bench config
        ims_network_params = bench_config.get_parameters("IMS_NETWORK")
        available_params = ims_network_params.get_dict()

        if "IMS_CONFIGURATION" in available_params:
            ims_config_file = ims_network_params.get_param_value("IMS_CONFIGURATION")
            ims_config_file = os.path.join(os.getcwd(), Paths.EXECUTION_CONFIG, ims_config_file)
            # Parse IMS configuration YAML file to retrieve IMS configuration for Anritsu MD8475A
            ims_generator = ImsConfigGenerator(ims_config_file,
                                               None,
                                               ("Anritsu parameters",),
                                               self._logger)
            ims_generator.load_parameters()
            ims_config_list = ims_generator.document["Anritsu parameters"]
            for key in ims_config_list:
                value = ims_config_list[key]
                # Check if configuration file is correctly filled (it shall have the same field as AnritsuM8475A._IMS_CONFIG_GPIB keys)
                if key not in AnritsuM8475A._IMS_CONFIG_GPIB:
                    AcsBaseException(AcsBaseException.INVALID_PARAMETER, "%s Anritsu MD8475A configuration file is not in the good format, please update it" % ims_config_file)
                # Remove all unneeded user
                if key == "User List":
                    pass
                    # GPIB does not work, if it works one day uncomment it
#                     nb_user = self._visa.query(AnritsuM8475A._IMS_CONFIG_GPIB[key], True)
#                     while nb_user >= 1:
#                         self._visa.send_command("IMSCSCFUSERSLISTDEL " + str(nb_user))
#                         nb_user -= 1
                else:
                    # Apply configuration
                    gpib_cmd = AnritsuM8475A._IMS_CONFIG_GPIB[key] + str(self._vnid) + "," + str(value)
                    self._visa.send_command(gpib_cmd)

    def start_ims(self):
        """
        Starts IMS virtual network
        """
        self._visa.send_command("IMSSTARTVN " + str(self._vnid))

    def get_database_access(self):
        """
        returns the initialized database intarface object
        :rtype: CommandsDatabase
        :return:
        """
        db = CommandsDatabase(self.DB_FILE)
        db.connect()
        return db

    def start_simulation(self):
        """
        Starts the SmartStudio simulation.
        """
        self._visa.send_command("START")

    def stop_simulation(self):
        """
        Stops the SmartStudio simulation
        """
        self._visa.send_command("STOP")

    def get_cell(self, cell_id=1):
        """
        Access to a cellular interface.

        :type cell_id: int
        :param cell_id: Number

        :rtype: CellCommon
        :return: the cell object
        """
        return self.__cells[(cell_id - 1)]

    def get_cell_2g(self, cell_number=1):
        """
        Access to 2G cellular interface.

        :type cell_number: int
        :param cell_number: the number of the cell to retrieve

        :rtype: ICell2G
        :return: the 2G cellular object.
        """
        cell_2g = [cell for cell in self.__cells if isinstance(cell, Cell2G)]
        if cell_number <= len(cell_2g):
            return cell_2g[(cell_number - 1)]
        else:
            error_msg = "Anritsu M8475A have only {0} 2G cells (cell number {1} asked)".format(
                len(cell_2g), cell_number)
            raise AcsBaseException(AcsBaseException.INVALID_PARAMETER, error_msg)

    def get_cell_3g(self, cell_number=1):
        """
        Access to 3G cellular interface.

        :type cell_number: int
        :param cell_number: the number of the cell to retrieve

        :rtype: ICell3G
        :return: the 3G cellular object.
        """
        cell_3g = [cell for cell in self.__cells if isinstance(cell, Cell3G)]
        if cell_number <= len(cell_3g):
            return cell_3g[(cell_number - 1)]
        else:
            error_msg = "Anritsu M8475A have only {0} 3G cells (cell number {1} asked)".format(
                len(cell_3g), cell_number)
            raise AcsBaseException(AcsBaseException.INVALID_PARAMETER, error_msg)

    def get_cell_4g(self, cell_number=1):
        """
        Access to 4G cellular interface.

        :type cell_number: int
        :param cell_number: the number of the cell to retrieve

        :rtype: ICell4G
        :return: the 4G cellular object.
        """
        cell_4g = [cell for cell in self.__cells if isinstance(cell, Cell4G)]
        if cell_number <= len(cell_4g):
            return cell_4g[(cell_number - 1)]
        else:
            error_msg = "Anritsu M8475A have only {0} 4G cells (cell number {1} asked)".format(
                len(cell_4g), cell_number)
            raise AcsBaseException(AcsBaseException.INVALID_PARAMETER, error_msg)

    def perform_full_preset(self):
        """
        Resets all equipment parameters to defaults.
        """
        self.get_logger().info("Perform full preset")
        self.reset()

        status = self._visa.query_command("STAT?")
        if status == "NOTEXIST":
            self._visa.send_command("RUN")
        elif status == "RUNNING":
            self._visa.send_command("STOP")

        self._visa.send_command("*RST")

    def load_configuration(self, config, cell_number=None):
        """
        This method executes the GPIB commands set identified by 'config'
        in configuration file
        :type config: str
        :param config: the name of the configuration to load

        :type cell_number: int
        :param cell_number: the number of the cell to configure

        :rtype list
        :returns for each matching config, the list attributes as dictionaries
        """
        if cell_number is not None:
            cell_name = "BTS" + str(int(cell_number))
        else:
            cell_name = "BTS1"
        VisaEquipmentBase.load_configuration(self, config, cell_name)

    def set_ip4_subnet_mask(self, mask):
        """
        Sets IPv4 subnet mask
        :type mask: str
        :param mask: the subnet mask to set. 15 characters formatted
        as follows: A.B.C.D where A,B,C,D are between = 0 to 255
        (no embedded spaces).
        :raise TestEquipmentException: failed to call SetIp4SubnetMask driver function
        """
        self._logger.info("Set IPV4 Subnet Mask: {0}".format(mask))
        # Default IPV4 Gateway
        gpib_command = "DGSUBNETMASK {0}".format(mask)
        self._visa.send_command(gpib_command)

    def set_ip4_default_gateway(self, gateway):
        """
        Sets default IPv4 gateway address
        :type gateway: str
        :param gateway: the gateway to set.
        """
        self._logger.info("Set IPV4 Default Gateway: {0}".format(gateway))

        if self._visa.query("DEFAULTGATEWAY?") == "DISABLE":
            # Enable Default Gateway
            gpib_command = "DEFAULTGATEWAY ENABLE"
            self._visa.send_command(gpib_command)
        # Default IPV4 Gateway
        gpib_command = "DGIPV4 {0}".format(gateway)
        self._visa.send_command(gpib_command)

    def set_ip6_default_gateway(self, gateway):
        """
        Sets default IPv6 gateway address
        :type gateway: str
        :param gateway: the gateway to set.
        """
        self._logger.info("Set IPV6 Default Gateway: {0}".format(gateway))
        if self._visa.query("DEFAULTGATEWAY?") == "DISABLE":
            # Enable Default Gateway
            gpib_command = "DEFAULTGATEWAY ENABLE"
            self._visa.send_command(gpib_command)
        # Default IPV4 Gateway
        gpib_command = "DGIPV6 {0}".format(gateway)
        self._visa.send_command(gpib_command)

    def set_pdn_apn(self, target, pdn):
        """
        Set the Access Point Name on a specific PDN

        :type target: int
        :param target: PDN number
        :type pdn: str
        :param pdn: Access Point Name
        """
        self._logger.info("Set PDN APN (PDN: {0}): {1}".format(target, pdn))
        gpib_command = "PDNAPN {0},{1}".format(target, pdn)
        self._visa.send_command(gpib_command)

    def set_pdn_reject(self, reject_cause, target):
        """
        Set the APN reject on a specific PDN

        :type reject_cause: int
        :param reject_cause: PDN reject cause

        :type target: int
        :param target: PDN number
        """
        self._logger.info("Set PDN APN to reject (PDN: {0}): cause {1}".format(target, reject_cause))
        gpib_command_reject = "PDNREPLY {0},{1}".format(target, "REJECT")
        gpib_command_cause = "PDNREJECTCAUSE {0},{1}".format(target, reject_cause)
        self._visa.send_command(gpib_command_reject)
        self._visa.send_command(gpib_command_cause)

    def set_pdn_ipv4_address(self, target, ipv4_address):
        """
        Set PDN Parameter UE IPv4 address

        :type target: int
        :param target: PDN number
        :type ipv4_address: str
        :param ipv4_address: IPv4 address
        """
        self._logger.info("Set PDN IPv4 address (PDN: {0}): {1}".format(target, ipv4_address))
        gpib_command = "PDNIPV4 {0},{1}".format(target, ipv4_address)
        self._visa.send_command(gpib_command)

    def set_pdn_ipv6_address(self, target, ipv6_address):
        """
        Set PDN Parameter UE IPv6 address

        :type target: int
        :param target: PDN number
        :type ipv6_address: str
        :param ipv6_address: IPv6 address
        """
        self._logger.info("Set PDN IPv4 address (PDN: {0}): {1}".format(target, ipv6_address))
        gpib_command = "PDNIPV6 {0},{1}".format(target, ipv6_address)
        self._visa.send_command(gpib_command)

    def set_pdn_ims(self, target, enable):
        """
        Enable or disable IMS Services for PDN

        :type target: int
        :param target: PDN number
        :type enable: bool
        :param enable: True or False depending if IMS should enabled or not
        """
        enable_param = "ENABLE" if enable else "DISABLE"
        self._logger.info("Set PDN IMS status (PDN: {0}): {1}".format(target, enable_param))
        gpib_command = "PDNIMS {0},{1}".format(target, enable_param)
        self._visa.send_command(gpib_command)
        gpib_command = "PDNCHECKAPN {0},IMS".format(target)
        self._visa.send_command(gpib_command)

    def set_pdn_vnid(self, target, vnid):
        """
        Sets the Virtual Network ID of PDN.

        :type target: int
        :param target: PDN number
        :type vnid: int
        :param vnid: VNID to set
        """
        self._logger.info("Set PDN VNID (PDN: {0}): {1}".format(target, vnid))
        gpib_command = "PDNVNID {0},{1}".format(target, vnid)
        self._visa.send_command(gpib_command)

    def send_command(self, command):
        """
        Send a GPIB Command.

        :type command: str
        :param command: String representing a well formed GPIB command.

        :return: None
        """
        self._visa.send_command(command)

    def query_command(self, command):
        """
        Send a GPIB Command.

        :type command: str
        :param command: String representing a well formed GPIB command.

        :rtype: String
        :return: Message from equipment giving the value of a dedicated GPIB command
        """
        self._visa.query(command)

    def get_ims_cscf_state(self):
        """
        Gets IMS CSCF state

        :rtype: str
        :return: IMS CSCF state
        """
        gpib_command = "IMSCSCFSTAT? {0}".format(self._vnid)
        output = self._visa.query_command(gpib_command)
        return output


class VisaInterfaceAnritsuM8475A(VisaInterface):
    """
    Add error handling wrapper to Anritsu M8475A visa command
    """
    __CLEAR_STATUS_CMD = "*CLS"
    __GET_LAST_ERR_CMD = "ERROR?"

    def __init__(self, transport, **kwargs):
        VisaInterface.__init__(self, transport, **kwargs)
        self._logger = Factory().create_logger("%s.%s.VISA_ANRITSU_M8475A" % (ACS_LOGGER_NAME, EQT_LOGGER_NAME))

    def send_command(self, command, bts=None):
        """
        Send a GPIB Command.

        :type command: str
        :param command: String representing a well formed GPIB command.

        :type bts: str
        :param bts: the BTS to apply the command
        """
        # Clear Status command. Clears the whole status structure
        self.write(VisaInterfaceAnritsuM8475A.__CLEAR_STATUS_CMD)
        # Send command to the equipment
        self._logger.debug("Send GPIB command: %s", command)
        # Add the BTS number if the GPIB finish with a ,
        if command[-1] == ",":
            command += str(bts)
        self.write(command)

        # Check errors
        self.error_check()

    def query_command(self, command):
        """
        Send a GPIB Command.

        :type command: str
        :param command: String representing a well formed GPIB command.

        :rtype: String
        :return: Message from equipment giving the value of a dedicated GPIB command
        """
        # Clear Status command. Clears the whole status structure
        self.write(VisaInterfaceAnritsuM8475A.__CLEAR_STATUS_CMD)
        # Send command to the equipment
        self._logger.debug("Send GPIB command: %s", command)
        response = self.query(command)

        return response

    def error_check(self):
        """
        Error checking and warning reporting
        :raise TestEquipmentException: if err != SUCCESS
        """
        error_code = self.query(VisaInterfaceAnritsuM8475A.__GET_LAST_ERR_CMD)
        if error_code != "0":
            error_msg = "Anritsu M8475A error during GPIB command (Error code: {0}: {1})".format(error_code, AnritsuM8475A_ErrorCodes[int(error_code)])
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, error_msg)
