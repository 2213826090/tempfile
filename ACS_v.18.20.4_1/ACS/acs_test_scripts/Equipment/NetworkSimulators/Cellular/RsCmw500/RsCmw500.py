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
:summary: implementation of Rohde & Schwarz CMW 500 cellular network simulator
using visa interface
:since: 16/09/2014
:author: jduran4x
.. note:: Based on previous Agilent E6621 implementation
"""
import os
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.TestEquipmentException import TestEquipmentException
from Core.Report.ACSLogging import ACS_LOGGER_NAME, EQT_LOGGER_NAME
from Core.Factory import Factory
from acs_test_scripts.TestStep.Utilities.Visa import VisaInterface, VisaEquipmentBase
from acs_test_scripts.Equipment.EquipmentManager import EquipmentManager as EM
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Interface.ICellNetSim import ICellNetSim
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmw500.Tech4G.Cell4G import Cell4G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmw500.Tech3G.Cell3G import Cell3G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmw500.Tech3G.CellTDSCDMA import CellTDSCDMA
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmw500.Tech2G.Cell2G import Cell2G
from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Database.CommandsDatabase import CommandsDatabase


class RsCmw500(ICellNetSim, VisaEquipmentBase):

    """
    Implementation of CMW 500 cellular network simulator
    It is based on full python PyVisa library.

    Please read following instructions to implement gpib commands for this equipment:
    - if you need to send a query command use following VisaEquipment method 'query'
    You will have to manage error in your method if needed.

    - if you need to send a write command, use method 'send_command' below.
    It will check errors by reading the response from the equipment.
    In case of PXT equipment it will return the message 'no command'
    else it will return the error message (i.e : 'Invalid header', 'Wrong param type' ...)
    """

    CONFIG_FILE_PATH = os.path.join(ICellNetSim.CONFIG_DIR, "Cmw500Configurations.xml")
    DB_FILE = os.path.join(ICellNetSim.DATABASE_DIR, "commands.db")

    GPIB_SUCCESS = '0,"No error"'

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        :type name: str
        :param name: the bench configuration name of the equipment
        :type model: str
        :param model: the model of the equipment
        :type eqt_params: dict
        :param eqt_params: the dictionary containing equipment parameters
        :type bench_params: BenchConfigParameters
        :param bench_params: the dictionary containing equipment bench parameters
        """
        # Initialize class parent
        ICellNetSim.__init__(self)
        VisaEquipmentBase.__init__(self, name, model, eqt_params, bench_params)

        # Initialize attributes
        self.__name = name
        self.__model = model
        self.__eqt_params = eqt_params
        self.__handle = None
        self.__cells = []
        # Get a reference of equipment manager
        self._em = EM()
        self._status = None

    def __del__(self):
        """
        Destructor: releases all allocated resources.
        """
        self.release()
        del self.__cells

    def set_cells_technology(self, cells_tech):
        """
        Sets cells technology used by the equipment

        :type cells_tech: List[str]
        :param cells_tech: List of cell technology (2G|GSM, 3G|WCDMA, TDSCDMA|TD-SCDMA, 4G|LTE)
        """
        self.__cells = []
        for cell_tech in cells_tech:
            if cell_tech in ["4G", "LTE"]:
                self.__cells.append(Cell4G(self._visa))
            elif cell_tech in ["3G", "WCDMA"]:
                self.__cells.append(Cell3G(self._visa))
            elif cell_tech in ["TDSCDMA", "TD-SCDMA"]:
                self.__cells.append(CellTDSCDMA(self._visa))
            elif cell_tech in ["2G", "GSM"]:
                self.__cells.append(Cell2G(self._visa))
            else:
                error_msg = "Wrong value for Cell tech {0} (Possible values: 2G|GSM, 3G|WCDMA, TDSCDMA|TD-SCDMA, 4G|LTE)"
                raise AcsBaseException(AcsBaseException.INVALID_PARAMETER, error_msg)
        self.perform_full_preset()

    def create_visa_interface(self, transport, kwargs):
        """
        Create visa interface
        :type transport: str
        :param transport: the bench configuration name of the equipment
        :type kwargs: dict
        :param kwargs: various needed parameters depending on the transport
        """
        self._visa = VisaInterfaceCMW500(transport, **kwargs)

    def get_database_access(self):
        """
        returns the initialized database intarface object
        :rtype: CommandsDatabase
        :return:
        """
        db = CommandsDatabase(self.DB_FILE)
        db.connect()
        return db

    def init(self):
        """
        Initializes the equipment. Final state: the equipment is ready to use.
        """
        pass

    def release(self):
        """
        Releases the connection with the equipment and all resources allocated
        to its use.
        """
        pass

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
            cell = cell_2g[(cell_number - 1)]
        else:
            # if cell does not exist, create it
            self.__cells.append(Cell2G(self._visa))
            cell = self.get_cell_2g(cell_number)
        return cell

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
            cell = cell_3g[(cell_number - 1)]
        else:
            # if cell does not exist, create it
            self.__cells.append(Cell3G(self._visa))
            cell = self.get_cell_3g(cell_number)
        return cell

    def get_cell_tdscdma(self, cell_number=1):
        """
        Access to 3G cellular interface.

        :type cell_number: int
        :param cell_number: the number of the cell to retrieve

        :rtype: ICell3G
        :return: the 3G cellular object.
        """
        cell_td = [cell for cell in self.__cells if isinstance(cell, CellTDSCDMA)]
        if cell_number <= len(cell_td):
            cell = cell_td[(cell_number - 1)]
        else:
            # if cell does not exist, create it
            self.__cells.append(CellTDSCDMA(self._visa))
            cell = self.get_cell_tdscdma(cell_number)
        return cell

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
            cell = cell_4g[cell_number - 1]
        else:
            # if cell does not exist, create it
            self.__cells.append(Cell4G(self._visa))
            cell = self.get_cell_4g(cell_number)
        return cell

    def load_configuration_file(self, filename):
        """
        Loads equipment parameters from a file.
        :type filename: str
        :param filename: the configuration file to load with his path

        Although the R&S CMW500 also allows the retrieval of configuration parameters from file,
        this was only necessary for PXT in order to load the signaling
        scenarios. With R&S state machine based implementation, this was not necessary and this method is left empty.
        """
        self.get_logger().info("load_configuration_file function is stubbed for RsCmw500")

    def start_scenario(self):
        """
        Start signaling scenario...

        Again, this is really not necessary for R&S CMW500, since the activation of the RF signals cannot be separated from the activation of the
        cell emulation... therefore, all calls related to cell activation have been moved to set_cell_on function in the Cell4G class...
        """
        self.get_logger().info("start_scenario function is stubbed for RsCmw500")

    def stop_scenario(self):
        """
        Stop signaling scenario...

        Again, this is really not necessary for R&S CMW500, since the deactivation of the RF signals cannot be separated from the deactication of the
        cell emulation... therefore, all calls related to cell deactivation have been moved to set_cell_off function in the Cell4G class...
        """
        self.get_logger().info("stop_scenario function is stubbed for RsCmw500")

    def perform_full_preset(self):
        """
        Resets all equipment parameters to defaults.
        """
        self.get_logger().info("Perform full preset")
        self.reset()
        self._visa.send_command("*RST")
        self._visa.send_command("STAT:PRES")

    def apply_bench_config(self, bench_config=None):
        """
        Applies configuration provided in the Bench Config
        :type bench_config: BenchConfigParameters
        :param bench_config: kep for compatibility but not used
        """
        # self._bench_params is inherited from VisaEquipmentBase
        available_params = self._bench_params.get_dict().keys()

        # Reset the pool of DUT IP address to be used
        dut_ipv4_address_pool = []
        dut_ipv6_address_pool = []

        # DAU IPV4 MODE
        name = "IPAttributionMode"
        if name in available_params:
            param = self._bench_params.get_param_value(name)
            if param not in ["", None]:
                self.get_cell_4g().get_data().set_ip_addr_configuration_mode(param, "IPV4")
                # In case IPAttributionMode is STATIC, need to set the DAU IP LAN address
                name = "IP_Lan1"
                if param == "STATIC":
                    param = self._bench_params.get_param_value(name)
                    self.set_ip4_lan_address(param)

        # Subnet_Mask for ipv4
        name = "Subnet_Mask"
        if name in available_params:
            param = self._bench_params.get_param_value(name)
            if param not in ["", None]:
                self.set_ip4_subnet_mask(param)
        # Default_Gateway for ipv4
        name = "Default_Gateway"
        if name in available_params:
            param = self._bench_params.get_param_value(name)
            if param not in ["", None]:
                self.set_ip4_default_gateway(param)
        # DUT_IP_Address for ipv4
        for dut_ip_nb in ["", "2", "3"]:
            name = "DUT_IP_Address%s" % dut_ip_nb
            if name in available_params:
                param = self._bench_params.get_param_value(name)
                if param not in ["", None]:
                    dut_ipv4_address_pool.append(param)
        if len(dut_ipv4_address_pool) > 0:
            for i in range(len(dut_ipv4_address_pool)):
                self.get_cell_4g().get_data().set_dut_ip_address(dut_ipv4_address_pool[i], dut_ipv4_address_pool)
        # DAU IPV6 MODE
        name = "IPv6AttributionMode"
        if name in available_params:
            param = self._bench_params.get_param_value(name)
            if param not in ["", None]:
                if param == "STATIC":
                    # In case IPv6AttributionMode is STATIC, need to set the DAU IP LAN address
                    name = "IPV6_Lan1"
                    param = self._bench_params.get_param_value(name)
                    self.get_cell_4g().get_data().set_ip6_network_parameters(param)
                else:
                    self.get_cell_4g().get_data().set_ip_addr_configuration_mode(param, "IPV6")

        # DUT_IPV6_Address
        for dut_ip_nb in ["", "2", "3"]:
            name = "DUT_IPV6_Address%s" % dut_ip_nb
            if name in available_params:
                param = self._bench_params.get_param_value(name)
                if param not in ["", None]:
                    dut_ipv6_address_pool.append(param)
        if len(dut_ipv6_address_pool) > 0:
            for i in range(len(dut_ipv6_address_pool)):
                self.get_cell_4g().get_data().set_dut_ip_address(dut_ipv6_address_pool[i], dut_ipv6_address_pool)
        # todo: additional option for ipv6 exist :  CONFigure:DATA:CONTrol:IPVSix:MANual:ROUTing:ADD

        # Toggle ON/OFF the signaling unit to stabilize the new DAU configuration
        self._logger.info("Toggle ON/OFF the Signaling Unit to stabilize the new DAU configuration...")
        if self.get_cell_4g().get_cell_status() == "OFF":
            self.get_cell_4g().set_cell_on()
            self.get_cell_4g().set_cell_off()
        else:
            # If the signaling unit was previously ON, then set_cell_on will
            # successively turn it OFF, then ON again.
            self.get_cell_4g().set_cell_on()

    def load_cell_config(self, config_type, eqt_id=1):
        """
        Configure equipment to specific cells via .xml files
        in order to reach specific throughput

        :type config_type: str
        :param config_type: the type of configuration(band, category, etc) to
        load.

        :type eqt_id: int
        :param eqt_id: Equipment number
        """
        # set specific cell parameters
        self._logger.info("Set Specific LTE parameters on Equipment"
                          " Rhode&Schwartz CMW500 : %s" % config_type)
        self._em.configure_equipments("CMW500_LTE_NS%d" % eqt_id, {"type": config_type})

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
        # Retrieve from bench configuration the frequency and offset table if not given
        if frequency_list is None or offset_list is None:
            if "AmplitudeOffsetTable" in self._bench_params.get_dict():
                amplitude_file = ConfigsParser(self._bench_params.get_param_value("AmplitudeOffsetTable"))
                amplitude_table = amplitude_file.parse_amplitude_offset_table()
                frequency_list = amplitude_table.get("FrequencyList")
                offset_list = amplitude_table.get("OffsetList")
                if "SecondAntennaOffsetList" in amplitude_table:
                    second_antenna_offset_list = amplitude_table.get("SecondAntennaOffsetList")
        if offset_list is not None:
            # The argument format for the CMW GPIB is freq1, attenuation1, freq2, attenuation2 ...
            main_antenna_list = zip(str(frequency_list).split(","), str(offset_list).split(","))
            main_antenna_str = ", ".join([item for sublist in main_antenna_list for item in sublist])
            # Load in CMW the compensation table
            self._visa.send_command("CONFigure:BASE:FDCorrection:CTABle:CREate 'ACS_main_antenna', %s" % main_antenna_str)
            # Activate the compensation table
            self._visa.send_command("CONFigure:FDCorrection:ACTivate RF1C, 'ACS_main_antenna'")

        try:
            if second_antenna_offset_list is not None:
                # The argument format for the CMW GPIB is freq1, attenuation1, freq2, attenuation2 ...
                div_antenna_list = zip(str(frequency_list).split(","), str(second_antenna_offset_list).split(","))
                div_antenna_str = ", ".join([item for sublist in div_antenna_list for item in sublist])
                # Load in CMW the compensation table
                self._visa.send_command("CONFigure:BASE:FDCorrection:CTABle:CREate 'ACS_div_antenna', %s" % div_antenna_str)
                # Activate the compensation table
                self._visa.send_command("CONFigure:FDCorrection:ACTivate RF3C, 'ACS_div_antenna'")

        except TestEquipmentException as e:
            self._logger.warning("Cannot activate compensation table for RF3C: %s" % e)

    def switch_app_format(self, app_format):
        """
        [Stub to align with RS CMU200 API] Switches application format of the network simulator.
        :type app_format: str
        :param app_format: the desired application format. Possible values:
            - "1xEV-DO"
            - "AMPS/136"
            - "GSM/GPRS"
            - "IS-2000/IS-95/AMPS"
            - "IS-856"
            - "WCDMA"
        """
        pass

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

    def set_ip4_lan_address(self, ip_addr):
        """
        Sets IPv4 LAN address
        :type ip_addr: str
        :param ip_addr: the IP address to set. 15 characters formatted
        as follows: A.B.C.D. The range of values for A = 0 to 126
        and 128 to 223. The range of values for B,C,D = 0 to 255
        (no embedded spaces).
        :raise TestEquipmentException: failed to call SetIp4LanAddress driver function
        """
        # Use 4G cell by default
        self.get_cell_4g().get_data().dau_set_ip4_lan_address(ip_addr)

    def set_ip4_default_gateway(self, gateway):
        """
        Sets default IPv4 gateway address
        :type gateway: str
        :param gateway: the gateway to set. 15 characters formatted
        as follows: A.B.C.D where A = 0 to 223 B,C,D = 0 to 255
        (no embedded spaces).
        """
        # use 4G cell by default
        self.get_cell_4g().get_data().dau_set_ip4_default_gateway(gateway)

    def set_ip4_subnet_mask(self, mask):
        """
        Sets IPv4 subnet mask
        :type mask: str
        :param mask: the subnet mask to set. 15 characters formatted
        as follows: A.B.C.D where A,B,C,D are between = 0 to 255
        (no embedded spaces).
        """
        # use 4G cell by default
        self.get_cell_4g().get_data().dau_set_ip4_subnet_mask(mask)


class VisaInterfaceCMW500(VisaInterface):
    """
    Add error handling wrapper to Anritsu M8475A visa command
    """
    """
    The string message returned by the equipment when no error occured
    after sending a gpib command
    """

    __GET_LAST_ERR_CMD = "SYST:ERR?"
    """
    GPIB command used to retrieve the last error.
    This command returns an error code (and an error message):
        - equal to zero if no error occurs
        - greater than zero for warning messages
        - lower than zero for error messages
    """
    __CLEAR_STATUS_CMD = "*CLS"
    """
    Clear Status command. Clears the whole status structure
    """
    __FILTERED_ERROR_CODES = []

    def __init__(self, transport, **kwargs):
        VisaInterface.__init__(self, transport, **kwargs)
        self._logger = Factory().create_logger("%s.%s.VISA_R&S_CMW500" % (ACS_LOGGER_NAME, EQT_LOGGER_NAME))

    def send_command(self, command, bts=None):
        """
        Send a GPIB Command.

        :type command: str
        :param command: String representing a well formed GPIB command.

        :type bts: str
        :param bts: the BTS to apply the command
        """
        # Clear Status command. Clears the whole status structure
        self.write(VisaInterfaceCMW500.__CLEAR_STATUS_CMD)
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
        self.write(VisaInterfaceCMW500.__CLEAR_STATUS_CMD)
        # Send command to the equipment
        self._logger.debug("Send GPIB command: %s", command)
        response = self.query(command)

        return response

    def error_check(self):
        """
        Error checking and warning reporting
        :raise TestEquipmentException: if err != SUCCESS
        """
        # query the last error
        return_msg = self.query(VisaInterfaceCMW500.__GET_LAST_ERR_CMD)
        (err_code, msg) = return_msg.split(',', 1)
        msg = msg.replace("\r", "")
        msg = msg.replace("\n", "")
        msg = str(msg).strip('"')
        # we can check error thanks to error code
        if err_code.isdigit() or err_code.startswith("-"):
            err_code = int(err_code)
            if err_code > 0 or err_code in VisaInterfaceCMW500.__FILTERED_ERROR_CODES:
                # Warning
                self.get_logger().warning(msg)
            elif err_code < 0:
                # Error
                self.get_logger().error(msg)
                raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, msg)
            # else command has been executed correctly!
        # if we can't retrieve error code,
        # we check that error message say that no
        # error occurred
        elif msg != VisaInterfaceCMW500.GPIB_SUCCESS:
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, msg)
