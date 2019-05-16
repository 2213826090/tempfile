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
:summary: This file is the base Use Case for external handover
:since: 12/08/2011
:author: ccontreras
"""
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil
from ErrorHandling.DeviceException import DeviceException


class LabMobilityBase(UseCaseBase):

    """
    Usecase base for mobility use cases
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        UseCaseBase.__init__(self, tc_name, global_config)

        # Read registrationTimeout from Device_Catalog.xml
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

        # Read maxDlMultislotConfig from DeviceCatalog.xml
        max_dl_multislot_config = \
            str(self._dut_config.get("maxDlMultislotConfig"))

        # Get Cellular Network configuration from BenchConfig
        self._network = global_config.benchConfig.get_parameters(
            "CELLULAR_NETWORK")
        self._apn = self._network.get_param_value("APN")
        self._ssid = self._network.get_param_value("SSID")

        # Read callSetupTimeout from Device_Catalog.xml
        self._call_setup_time = \
            int(self._dut_config.get("callSetupTimeout"))

        # Read NETWORK_SIMULATOR1 from BenchConfig.xml
        self._ns1_node = \
            global_config.benchConfig.get_parameters("NETWORK_SIMULATOR1")

        # Read NETWORK_SIMULATOR2 from BenchConfig.xml
        self._ns2_node = \
            global_config.benchConfig.get_parameters("NETWORK_SIMULATOR2")

        # Create active cellular network simulator instance and retrieve 3G data interface
        self._ns1 = self._em.get_cellular_network_simulator("NETWORK_SIMULATOR1")
        # Retrieve the model of the equipment
        self._ns1_model = self._ns1_node.get_param_value("Model")

        # Create idle cellular network simulator instance
        self._ns2 = self._em.get_cellular_network_simulator("NETWORK_SIMULATOR2")
        # Retrieve the model of the equipment
        self._ns2_model = self._ns2_node.get_param_value("Model")

        # Read the Multislot Configuration value from UseCase xml Parameter
        multislot_config = \
            str(self._tc_parameters.get_param_value("MULTISLOT_CONFIG"))
        if multislot_config == "" or multislot_config == "None":
            self._multislot = max_dl_multislot_config
        else:
            self._multislot = multislot_config

        # Read IP configuration parameters from NETWORK_SIMULATOR1
        # and from NETWORK_SIMULATOR2
        self.read_ip_configuration_parameters()

        # Instantiate generic UECmd for voiceCall Ucs
        self._modem_api = self._device.get_uecmd("Modem")
        self._voicecall_api = self._device.get_uecmd("VoiceCall")
        self._networking_api = self._device.get_uecmd("Networking")
        self._phone_system_api = self._device.get_uecmd("PhoneSystem")

        # Connect to equipments
        self._ns1.init()
        self._ns2.init()

# ------------------------------------------------------------------------------
    def set_up(self):
        """
        Set up the test configuration
        """
        UseCaseBase.set_up(self)

        # Connect to equipments
        self._ns1.init()
        self._ns2.init()

        # Remove WIFI Configuration
        self._networking_api.wifi_remove_config('all')

        return Global.SUCCESS, "No errors"

# ------------------------------------------------------------------------------
    def read_ip_configuration_parameters(self):
        """
        Read IP configuration parameters from NETWORK_SIMULATOR1
        and from NETWORK_SIMULATOR2
        """
        # Read IP configuration parameters from NETWORK_SIMULATOR1:
        # IP_Lan1
        # IP_Lan2
        # DUT_IP_Address
        # DNS1
        # DNS2
        # Subnet_Mask
        # Default_Gateway
        self._ns1_ip_lan1 = \
            self._ns1_node.get_param_value("IP_Lan1")
        self._ns1_ip_lan2 = \
            self._ns1_node.get_param_value("IP_Lan2")
        self._ns1_ip_dut = \
            self._ns1_node.get_param_value("DUT_IP_Address")
        self._ns1_ip_dns1 = self._ns1_node.get_param_value("DNS1")
        self._ns1_ip_dns2 = self._ns1_node.get_param_value("DNS2")
        self._ns1_ip_subnet_mask = \
            self._ns1_node.get_param_value("Subnet_Mask")
        self._ns1_ip_default_gateway = \
            self._ns1_node.get_param_value("Default_Gateway")

        # Read IP configuration parameters from NETWORK_SIMULATOR2:
        # IP_Lan1
        # IP_Lan2
        # DUT_IP_Address
        # DNS1
        # DNS2
        # Subnet_Mask
        # Default_Gateway
        self._ns2_ip_lan1 = \
            self._ns2_node.get_param_value("IP_Lan1")
        self._ns2_ip_lan2 = \
            self._ns2_node.get_param_value("IP_Lan2")
        self._ns2_ip_dut = \
            self._ns2_node.get_param_value("DUT_IP_Address")
        self._ns2_ip_dns1 = self._ns2_node.get_param_value("DNS1")
        self._ns2_ip_dns2 = self._ns2_node.get_param_value("DNS2")
        self._ns2_ip_subnet_mask = \
            self._ns2_node.get_param_value("Subnet_Mask")
        self._ns2_ip_default_gateway = \
            self._ns2_node.get_param_value("Default_Gateway")

# ------------------------------------------------------------------------------
    def set_external_connection(self,
                                ns1,
                                ns2,
                                ns1_ip_lan2,
                                ns2_ip_lan2):
        """
        This function sets the External Connection

        :type ns1: str
        :param ns1: the Instance of the network simulator 1 to set
        from the bench_config.xml file
        :type ns2: str
        :param ns2: the Instance of the network simulator 2 to set
        from the bench_config.xml file
        :type ns1_ip_lan2: str
        :param ns1_ip_lan2: the IP address to set. 15 characters formatted
        as follows: A.B.C.D. The range of values for A = 0 to 126
        and 128 to 223. The range of values for B,C,D = 0 to 255
        (no embedded spaces).
        :type ns2_ip_lan2: str
        :param ns2_ip_lan2: the IP address to set. 15 characters formatted
        as follows: A.B.C.D. The range of values for A = 0 to 126
        and 128 to 223. The range of values for B,C,D = 0 to 255
        (no embedded spaces).
        """

        # Set external 8960 Ip Address with Agilent 8960 idle
        ns1.set_external_ip_address(ns2_ip_lan2)

        # Set external 8960 Ip Address with Agilent 8960 active
        ns2.set_external_ip_address(ns1_ip_lan2)

        # Connect to external 8960
        ns1.connect_to_external_device()

        # Verify ethernet connection before timeout
        # 5 seconds using the Agilent 8960 idle IP address
        ns1.check_external_device_connection(
            1, self._ns2_ip_lan2, 5)

        # Verify ethernet connection before timeout
        # 5 seconds using the Agilent 8960 active IP address
        ns2.check_external_device_connection(
            1, self._ns1_ip_lan2, 5)

# ------------------------------------------------------------------------------
    def check_2G_measurement_while_compress_mode(self,
                                                 timeout,
                                                 ns_cell,
                                                 neighbour_cell_arfcn,
                                                 target_rssi=-100):
        """
        Check if 2G cell has been measured during compress mode and if its RSSI is correct (>= -100dBm)
        :type timeout: int
        :param timeout: timeout used to get RSSI measurement
        :type ns_cell: str
        :param ns_cell: "Camped" Network Simulator Cell API
        :type cell_arfcn: int
        :param neighbour_cell_arfcn: Neighbour cell to measure ARFCN
        :type target_rssi: float
        :param target_rssi: Neighbour cell to measure target RSSI
        :raise DeviceException if neighbour_cell_arfcn is not measured or if its measured
        RSSI is too low
        """
        elapsed_time = 0
        while elapsed_time < timeout:
            elapsed_time += 1
            time.sleep(1)
            # check if 2G cell is reported by UE
            measure_dict = ns_cell.get_gsm_neighbor_cells_rssi_measurements()

            if neighbour_cell_arfcn in measure_dict.keys():
                self._logger.info("2G cell has been measured during compress mode")
                # Measured RSSI = -110dBm + value returned by 8960 (measure_dict[neighbour_cell_arfcn])
                if -110 + measure_dict[neighbour_cell_arfcn] >= target_rssi:
                    self._logger.info("2G cell has been measured during compress mode and its measured RSSI is more than -100dBm")
                    break
                else:
                    # Log that Cell Re-selection has failed
                    msg = self._logger.error("2G cell has measured during compress mode but its RSSI is less than %ddBm: %d dBm." % (target_rssi,
                                                                                                                                     - 110 - measure_dict[neighbour_cell_arfcn]))

                    # Return message and quit the method
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)
            else:
                # Log that Cell Re-selection has failed
                msg = self._logger.error("2G cell has not been measured during compress mode.")

                # Return message and quit the method
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

# ------------------------------------------------------------------------------
    def decrease_cell_power_while_idle(self,
                                       ns_camped_cell,
                                       ns_camped_cell_power,
                                       ns_neighbour_cell,
                                       ns_neighbour_data,
                                       ns_neighbour_cell_service,
                                       decrementation_step_power,
                                       decrementation_step_timer,
                                       cresel_limit_power,
                                       cresel_power,
                                       ns_neighbour_model):
        """
        Decrease cell power in case of IDLE Cell Reselection

        :type ns_camped_cell: str
        :param ns_camped_cell: "Camped" Network Simulator Cell API
        :type ns_camped_cell_power: float
        :param ns_camped_cell_power: "Camped" Network Simulator cell power
        :type ns_neighbour_cell: str
        :param ns_neighbour_cell: "NEIGHBOUR" Network Simulator Cell API
        :type ns_neighbour_data: str
        :param ns_neighbour_data: "NEIGHBOUR" Network Simulator Data API
        :type ns_neighbour_cell_power: float
        :param ns_neighbour_cell_power: cell power of the neighbour cell
        :type decrementation_step_power: float
        :param decrementation_step_power: Decrementation step for
        NS1 cell power in dBm (must be positive, can be decimal value
        like 0,20 or 2,6)
        :type decrementation_step_timer: float
        :param decrementation_step_timer: Decrementation step timer
        in seconds between 2 steps (must be positive, can be
        decimal value like 0,3 or 3,5)
        :type cresel_power: float
        :param cresel_power: cell power of the neighbour cell
        :type cresel_limit_power: float
        :param cresel_limit_power: Limit power to stop the cell
        reselection power if reached
        """
        self._logger.info("Set the neighbour cell power to %.2f dBm ", cresel_power)

        # Set final Neighbour cell power to NS2_CELL_POWER
        ns_neighbour_cell.set_cell_power(cresel_power)

        # the Connection State to check we are using PXT or AGILENT8960
        if ns_neighbour_model == "AGILENT_8960":
            conn_state = "ATTACHED"
        elif ns_neighbour_model == "AGILENT_E6621A":
            conn_state = "CON"

        # WHILE DUT is registered to the "Camped"
        # cell AND ns_camped_cell_power <= cresel_limit_power
        dut_imsi = self._modem_api.get_imsi(self._registration_timeout)
        while True:
            # Decrement active cell power
            ns_camped_cell_power -= decrementation_step_power
            ns_camped_cell.set_cell_power(ns_camped_cell_power)

            # Wait during DECREMENTATION_STEP_TIMER
            time.sleep(decrementation_step_timer)

            if ns_neighbour_cell_service in ("GSM", "CIRCUIT"):
                # Check phone registration on equipment before given CAMP_TIMEOUT
                # (in loop compare every second the IMSI returned by the simulator
                # and IMSI read on CDK until comparison is true or timeout expired)
                if(RegUtil.check_dut_registration_before_timeout(ns_neighbour_cell,
                                                                 self._networking_api,
                                                                 self._logger,
                                                                 dut_imsi,
                                                                 decrementation_step_timer,
                                                                 flightmode_cycle=False,
                                                                 blocking=False)):

                    # Log the cell reselection success
                    self._logger.info(
                        "Cell reselection succeeded at %.2f dBm " +
                        "Camped cell power.",
                        ns_camped_cell_power)
                    # DUT Camped on neighbour cell, break function to quit the loop
                    break

            else:
                # Check Data Connection State => conn_state before timeout
                if ns_neighbour_data.check_data_connection_state(conn_state, decrementation_step_timer, False):
                    # Log the cell reselection success
                    self._logger.info(
                        "Cell reselection succeeded at %.2f dBm " +
                        "Camped cell power.",
                        ns_camped_cell_power)
                    # DUT Camped on neighbour cell, break function to quit the loop
                    break

            # IF ns_camped_cell_power < cresel_limit_power
            if ns_camped_cell_power < cresel_limit_power:

                # Log the cell reselection failure
                msg = self._logger.error("The Power's limit has been reached %.2f dBm, Cell reselection has failed",
                                         cresel_limit_power)

                # Return message and quit the method
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # IF DUT is registered to the Neighbour cell before registration timeout
        # When in GSM there is no data connection but IMSI have already been checked
        if ns_neighbour_cell_service in "GSM":
            # Log that cell reselection is done
            self._logger.info("Cell reselection success.")
            ns_camped_cell.set_cell_power(cresel_limit_power)
        elif ns_neighbour_data.check_data_connection_state(conn_state, decrementation_step_timer, False):
            # Check that DUT is registered on the good RAT
            self._modem_api.check_network_type_before_timeout(ns_neighbour_data.get_network_type(),
                                                              self._registration_timeout)
            # Log that cell reselection is done
            self._logger.info("Cell reselection success.")
            ns_camped_cell.set_cell_power(cresel_limit_power)
        else:
            # Log that Cell Reselection has failed
            msg = self._logger.error("Cell reselection has failed.")
            # Return message and quit the method
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

# ------------------------------------------------------------------------------
    def decrease_cell_power_while_idle_no_pdp(self,
                                              ns_camped_cell,
                                              ns_camped_cell_power,
                                              ns_neighbour_cell,
                                              ns_neighbour_data,
                                              decrementation_step_power,
                                              decrementation_step_timer,
                                              cresel_limit_power,
                                              cresel_power,
                                              ns_neighbour_model):
        """
        Decrease cell power in case of IDLE Cell Reselection with no PDP context

        :type ns_camped_cell: str
        :param ns_camped_cell: "Camped" Network Simulator Cell API
        :type ns_camped_cell_power: float
        :param ns_camped_cell_power: "Camped" Network Simulator cell power
        :type ns_neighbour_cell: str
        :param ns_neighbour_cell: "NEIGHBOUR" Network Simulator Cell API
        :type ns_neighbour_data: str
        :param ns_neighbour_data: "NEIGHBOUR" Network Simulator Data API
        :type ns_neighbour_cell_power: float
        :param ns_neighbour_cell_power: cell power of the neighbour cell
        :type decrementation_step_power: float
        :param decrementation_step_power: Decrementation step for
        NS1 cell power in dBm (must be positive, can be decimal value
        like 0,20 or 2,6)
        :type decrementation_step_timer: float
        :param decrementation_step_timer: Decrementation step timer
        in seconds between 2 steps (must be positive, can be
        decimal value like 0,3 or 3,5)
        :type cresel_limit_power: float
        :param cresel_limit_power: Limit power to stop the cell
        :type cresel_power: float
        :param cresel_power: cell power of the neighbour cell
        reselection power if reached
        """
        self._logger.info("Set the neighbour cell power to %.2f dBm ", cresel_power)

        # Set final Neighbour cell power to NS2_CELL_POWER
        ns_neighbour_cell.set_cell_power(cresel_power)

        # WHILE DUT is registered to the "Camped"
        # cell AND ns_camped_cell_power <= cresel_limit_power
        dut_imsi = self._modem_api.get_imsi(self._registration_timeout)
        while True:
            # Wake up screen
            self._phone_system_api.wake_screen()

            # Decrement active cell power
            ns_camped_cell_power -= decrementation_step_power
            ns_camped_cell.set_cell_power(ns_camped_cell_power)

            # Wait during DECREMENTATION_STEP_TIMER
            time.sleep(decrementation_step_timer)

            # Check that DUT is registered on the good RAT
            if self._modem_api.check_network_type_before_timeout(ns_neighbour_data.get_network_type(),
                                                                 decrementation_step_timer):
                # Check phone registration on equipment before given CAMP_TIMEOUT
                # (in loop compare every second the IMSI returned by the simulator
                # and IMSI read on CDK until comparison is true or timeout expired)
                if(RegUtil.check_dut_registration_before_timeout(ns_neighbour_cell,
                                                                 self._networking_api,
                                                                 self._logger,
                                                                 dut_imsi,
                                                                 decrementation_step_timer,
                                                                 flightmode_cycle=False,
                                                                 blocking=False)):
                    # Log the cell reselection success
                    self._logger.info(
                        "Cell reselection succeeded at %.2f dBm " +
                        "Camped cell power.",
                        ns_camped_cell_power)
                    # DUT Camped on neighbour cell, break function to quit the loop
                    # Log that cell reselection is done
                    self._logger.info("Cell reselection success.")
                    ns_camped_cell.set_cell_power(cresel_limit_power)
                    break

            # IF ns_camped_cell_power < cresel_limit_power
            if ns_camped_cell_power < cresel_limit_power:

                # Log the cell reselection failure
                msg = self._logger.error("The Power's limit has been reached %.2f dBm, Cell reselection has failed",
                                         cresel_limit_power)

                # Return message and quit the method
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

# ------------------------------------------------------------------------------
    def resel_error(self):
        # Log that Cell Reselection has failed
        msg = "Cell reselection has failed."
        self._logger.error(msg)

        # Return message and quit the method
        raise DeviceException(DeviceException.OPERATION_FAILED,
                              msg)

# ------------------------------------------------------------------------------
    def go_out_of_coverage_and_cresel(self,
                                      ns_camped_cell,
                                      ns_neighbour_cell,
                                      ns_neighbour_data,
                                      ns_neighbour_cell_service,
                                      cresel_power,
                                      ns_neighbour_model,
                                      cresel_timeout,
                                      cresel_nocoverage_time):
        """
        Decrease cell power in case of IDLE Cell Reselection

        :type ns_camped_cell: str
        :param ns_camped_cell: "Camped" Network Simulator Cell API
        :type ns_neighbour_cell: str
        :param ns_neighbour_cell: "NEIGHBOUR" Network Simulator Cell API
        :type ns_neighbour_data: str
        :param ns_neighbour_data: "NEIGHBOUR" Network Simulator Data API
        :type cresel_power: float
        :param cresel_power: cell power of the neighbour cell
        :type cresel_timeout: float
        :param cresel_timeout: time to go from no coverage to attach to a new cell
        :type cresel_nocoverage_time: float
        :param cresel_nocoverage_time: time to stay in no coverage
        """

        # Set camped cell power to -115dBm
        ns_camped_cell.set_cell_power(-115)
        self._logger.info("Going to no coverage zone")
        # stay a while unregistered
        time.sleep(cresel_nocoverage_time)
        # if the DUT is not unregistered test is failed
        if self._modem_api.get_network_registration_status() != "unregistered":
            msg = "DUT is not unregistered"
            self._logger.error(msg)
            # Return message and quit the method
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        # Set final Neighbour cell power to NS2_CELL_POWER
        ns_neighbour_cell.set_cell_power(cresel_power)

        # the Connection State to check we are using PXT or AGILENT8960
        if ns_neighbour_model == "AGILENT_8960":
            conn_state = "ATTACHED"
        elif ns_neighbour_model == "AGILENT_E6621A":
            conn_state = "CON"

        dut_imsi = self._modem_api.get_imsi(self._registration_timeout)

        if ns_neighbour_cell_service in ("GSM",
                                         "WCDMA",
                                         "PACKET"):
            # Check phone registration on equipment before given CAMP_TIMEOUT
            # (in loop compare every second the IMSI returned by the simulator
            # and IMSI read on CDK until comparison is true or timeout expired)
            if(RegUtil.check_dut_registration_before_timeout(ns_neighbour_cell,
                                                             self._networking_api,
                                                             self._logger,
                                                             dut_imsi,
                                                             cresel_timeout,
                                                             False,
                                                             False)):

                # Log the cell reselection success
                self._logger.info("Cell reselection succeeded")
                # DUT Camped on neighbour cell, break function to quit the loop
            else:
                self.resel_error()

        else:
            # Check Data Connection State => conn_state before timeout
            if(ns_neighbour_data.check_data_connection_state(conn_state,
                                                             cresel_timeout,
                                                             False)):
                # Log the cell reselection success
                self._logger.info("Cell reselection succeeded ")
                # DUT Camped on neighbour cell, break function to quit the loop
            else:
                # Log that Cell Reselection has failed
                self.resel_error()

# ------------------------------------------------------------------------------
    def decrease_cell_power_while_ftp(self,
                                      ns_camped_cell,
                                      ns_camped_cell_power,
                                      decrementation_step_power,
                                      decrementation_step_timer,
                                      cell_resel_camped_limit_power):
        """
        Decrease / Increase cell power in case of Cell Reselection
        while FTP transfer is on-going

        :type ns_camped_cell: str
        :param ns_camped_cell: "Camped" Network Simulator Cell API
        :type ns_camped_cell_power: float
        :param ns_camped_cell_power: "Camped" Network Simulator cell power

        :type decrementation_step_power: float
        :param decrementation_step_power: Decrementation step for
        NS1 cell power in dBm (must be positive, can be decimal value
        like 0,20 or 2,6)
        :type decrementation_step_timer: float
        :param decrementation_step_timer: Decrementation step timer
        in seconds between 2 steps (must be positive, can be
        decimal value like 0,3 or 3,5)
        :type cell_resel_camped_limit_power: float
        :param cell_resel_camped_limit_power: Limit power on Camped cell to stop the cell
        reselection power if reached
        """

        msg = "Begin to decrease registered cell power from %.2f dBm "
        msg += "to %.2f dBm each %d seconds by step of %.2f dBm before "
        msg += "cell reselection is performed."
        self._logger.info(
            msg,
            ns_camped_cell_power,
            cell_resel_camped_limit_power,
            decrementation_step_timer,
            decrementation_step_power)

        # WHILE DUT is registered to the camped cell
        # AND ns_camped_cell_power < self._cell_resel_camped_limit_power
        # AND ns_neighbour_cell_power > self._cell_resel_neighbour_limit_power
        while ns_camped_cell_power > cell_resel_camped_limit_power:
            # Decrement active cell power
            ns_camped_cell_power -= decrementation_step_power
            ns_camped_cell.set_cell_power(ns_camped_cell_power)

            # Wait during DECREMENTATION_STEP_TIMER
            time.sleep(self._decrementation_step_timer)

# ------------------------------------------------------------------------------
    def decrease_and_increase_cell_power_while_ftp(self,
                                                   ns_camped_cell,
                                                   ns_camped_cell_power,
                                                   ns_neighbour_cell,
                                                   ns_neighbour_data,
                                                   ns_neighbour_cell_power,
                                                   decrementation_step_power,
                                                   decrementation_step_timer,
                                                   cell_resel_camped_limit_power,
                                                   incrementation_step_power,
                                                   incrementation_step_timer,
                                                   cell_resel_neighbour_limit_power,
                                                   camped_cell_tech="2G",
                                                   ns_camped_data=None):
        """
        Decrease / Increase cell power in case of Cell Reselection
        while FTP transfer is on-going

        :type ns_camped_cell: str
        :param ns_camped_cell: "Camped" Network Simulator Cell API
        :type ns_camped_cell_power: float
        :param ns_camped_cell_power: "Camped" Network Simulator cell power

        :type ns_neighbour_cell: str
        :param ns_neighbour_cell: "NEIGHBOUR" Network Simulator Cell API
        :type ns_neighbour_data: str
        :param ns_neighbour_data: "NEIGHBOUR" Network Simulator Data API
        :type ns_neighbour_cell_power: float
        :param ns_neighbour_cell_power: "NEIGHBOUR" Network Simulator cell power

        :type decrementation_step_power: float
        :param decrementation_step_power: Decrementation step for
        NS1 cell power in dBm (must be positive, can be decimal value
        like 0,20 or 2,6)
        :type decrementation_step_timer: float
        :param decrementation_step_timer: Decrementation step timer
        in seconds between 2 steps (must be positive, can be
        decimal value like 0,3 or 3,5)
        :type cell_resel_camped_limit_power: float
        :param cell_resel_camped_limit_power: Limit power on Camped cell to stop the cell
        reselection power if reached

        :type incrementation_step_power: float
        :param incrementation_step_power: Incrementation step for
        Neighbour cell power in dBm (must be positive, can be decimal value
        like 0,20 or 2,6)
        :type incrementation_step_timer: float
        :param incrementation_step_timer: Incrementation step timer
        in seconds between 2 steps (must be positive, can be
        decimal value like 0,3 or 3,5)
        :type cell_resel_neighbour_limit_power: float
        :param cell_resel_neighbour_limit_power: Limit power on Neighbor cell to stop the cell
        reselection power if reached
        :type camped_cell_tech: str
        :param camped_cell_tech: Camped cell technology 2G or 3G
        :type ns_camped_data: str
        :param ns_camped_data: Camped cell technology 2G or 3G
                                                   ns_camped_data=None
        :raise DeviceException:
            - if DUT lost connection, raise DeviceException.CONNECTION_LOST
            - if DUT reselection failed, raise DeviceException.OPERATION_FAILED
        """
        check_data_transfer_state_timeout = 5
        # Set the limits reached flag to false for camped and neighbour cells
        camped_cell_power_limit_reached = False
        neighbour_cell_power_limit_reached = False

        msg = "Begin to decrease registered cell power from %.2f dBm "
        msg += "to %.2f dBm each %d seconds by step of %.2f dBm before "
        msg += "cell reselection is performed."
        self._logger.info(
            msg,
            ns_camped_cell_power,
            cell_resel_camped_limit_power,
            decrementation_step_timer,
            decrementation_step_power)

        msg = "Begin to increase neighbour cell power from %.2f dBm "
        msg += "to %.2f dBm each %d seconds by step of %.2f dBm before "
        msg += "cell reselection is performed."
        self._logger.info(
            msg,
            ns_neighbour_cell_power,
            cell_resel_neighbour_limit_power,
            incrementation_step_timer,
            incrementation_step_power)

        # WHILE DUT is registered to the camped cell
        # AND ns_camped_cell_power < self._cell_resel_camped_limit_power
        # AND ns_neighbour_cell_power > self._cell_resel_neighbour_limit_power
        while True:

            if(ns_neighbour_data.check_data_connection_state("PDP_ACTIVE",
                                                             check_data_transfer_state_timeout,
                                                             blocking=False)):

                # Wait to avoid checking ftp transfer status on PDP activation event...
                # Let's ftp transfer restarting after PDP activation
                time.sleep(check_data_transfer_state_timeout)

                if self._ftp_api.get_ftp_status(self._ftp_task_id).lower() == "transferring":

                    # Log the cell reselection success
                    self._logger.info(
                        "Cell reselection succeeded at %.2f dBm " +
                        "Camped cell power.",
                        ns_camped_cell_power)

                    # Decrease power on originate cell to avoid to quit destination cell
                    ns_camped_cell_power = cell_resel_camped_limit_power
                    ns_camped_cell.set_cell_power(ns_camped_cell_power)

                    # Finish to increase power on destination cell
                    ns_neighbour_cell_power = cell_resel_neighbour_limit_power
                    ns_neighbour_cell.set_cell_power(ns_neighbour_cell_power)

                    # Get RAT from Equipment
                    network_type = ns_neighbour_data.get_network_type()

                    # Check that DUT is registered on the good RAT
                    self._modem_api.check_network_type_before_timeout(network_type,
                                                                      self._registration_timeout)

                    # DUT Camped on neighbour cell, break function to quit the loop
                    break

                else:
                    msg = self._logger.info("Cell reselection has failed. FTP transfer has been lost !")

                    raise DeviceException(DeviceException.CONNECTION_LOST, msg)

            # No more cell power decrease or increase to perform
            if camped_cell_power_limit_reached and neighbour_cell_power_limit_reached:

                # Log the appropriate error and quit the Use Case
                msg = self._logger.info("Cell Reselection not done within cell power range")

                # Return message and quit the UC
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            # Power decrease
            if ns_camped_cell_power > cell_resel_camped_limit_power:

                # Decrement active cell power
                ns_camped_cell_power -= decrementation_step_power
                ns_camped_cell.set_cell_power(ns_camped_cell_power)

                # Wait during DECREMENTATION_STEP_TIMER
                time.sleep(self._decrementation_step_timer)

            else:
                camped_cell_power_limit_reached = True

            # Power increase
            if ns_neighbour_cell_power < cell_resel_neighbour_limit_power:

                # Increment idle cell power
                ns_neighbour_cell_power += incrementation_step_power
                ns_neighbour_cell.set_cell_power(ns_neighbour_cell_power)

                # Wait during INCREMENTATION_STEP_TIMER
                time.sleep(incrementation_step_timer)

            else:
                neighbour_cell_power_limit_reached = True
            # Force CELL FACH to perform 3G to 2G reselection
            if camped_cell_tech == "3G":
                ns_camped_data.set_rrc_transition("FACH")

# ------------------------------------------------------------------------------
    def decrease_and_increase_cell_power_while_vc(self,
                                                  ns_camped_cell,
                                                  ns_camped_vc,
                                                  ns_camped_cell_power,
                                                  ns_neighbour_cell,
                                                  ns_neighbour_data,
                                                  ns_neighbour_cell_power,
                                                  decrementation_step_power,
                                                  decrementation_step_timer,
                                                  cell_resel_camped_limit_power,
                                                  incrementation_step_power,
                                                  incrementation_step_timer,
                                                  cell_resel_neighbour_limit_power):
        """
        Decrease / Increase cell power in case of Cell Reselection
        while Voice Call is on-going

        :type ns_camped_cell: str
        :param ns_camped_cell: "Camped" Network Simulator Cell API
        :type ns_camped_vc: str
        :param ns_camped_vc: "Camped" Network Simulator VoiceCall API
        :type ns_camped_cell_power: float
        :param ns_camped_cell_power: "Camped" Network Simulator cell power

        :type ns_neighbour_cell: str
        :param ns_neighbour_cell: "NEIGHBOUR" Network Simulator Cell API
        :type ns_neighbour_data: str
        :param ns_neighbour_data: "NEIGHBOUR" Network Simulator Data API
        :type ns_neighbour_cell_power: float
        :param ns_neighbour_cell_power: "NEIGHBOUR" Network Simulator cell power

        :type decrementation_step_power: float
        :param decrementation_step_power: Decrementation step for
        NS1 cell power in dBm (must be positive, can be decimal value
        like 0,20 or 2,6)
        :type decrementation_step_timer: float
        :param decrementation_step_timer: Decrementation step timer
        in seconds between 2 steps (must be positive, can be
        decimal value like 0,3 or 3,5)
        :type cell_resel_camped_limit_power: float
        :param cell_resel_camped_limit_power: Limit power on Camped cell to stop the cell
        reselection power if reached

        :type incrementation_step_power: float
        :param incrementation_step_power: Incrementation step for
        Neighbour cell power in dBm (must be positive, can be decimal value
        like 0,20 or 2,6)
        :type incrementation_step_timer: float
        :param incrementation_step_timer: Incrementation step timer
        in seconds between 2 steps (must be positive, can be
        decimal value like 0,3 or 3,5)
        :type cell_resel_neighbour_limit_power: float
        :param cell_resel_neighbour_limit_power: Limit power on Neighbour cell to stop the cell
        reselection power if reached
        """
        # Set the limits reached flag to false for camped and neighbour cells
        camped_cell_power_limit_reached = False
        neighbour_cell_power_limit_reached = False

        msg = "Begin to decrease registered cell power from %.2f dBm "
        msg += "to %.2f dBm each %d seconds by step of %.2f dBm before "
        msg += "cell reselection is performed."
        self._logger.info(
            msg,
            ns_camped_cell_power,
            cell_resel_camped_limit_power,
            decrementation_step_timer,
            decrementation_step_power)

        msg = "Begin to increase neighbour cell power from %.2f dBm "
        msg += "to %.2f dBm each %d seconds by step of %.2f dBm before "
        msg += "cell reselection is performed."
        self._logger.info(
            msg,
            ns_neighbour_cell_power,
            cell_resel_neighbour_limit_power,
            incrementation_step_timer,
            incrementation_step_power)

        # WHILE DUT is registered to the camped cell
        # AND ns_camped_cell_power < self._cell_resel_camped_limit_power
        # AND ns_neighbour_cell_power > self._cell_resel_neighbour_limit_power
        while True:

            # Check call state "CONNECTED" before callSetupTimeout seconds
            ns_camped_vc.check_call_connected(self._call_setup_time,
                                              blocking=False)

            if camped_cell_power_limit_reached and neighbour_cell_power_limit_reached:
                # Check call state "CONNECTED" before callSetupTimeout seconds
                ns_camped_vc.check_call_connected(self._call_setup_time,
                                                  blocking=False)
                break

            if ns_camped_cell_power > cell_resel_camped_limit_power:

                # Decrement active cell power
                ns_camped_cell_power -= decrementation_step_power
                ns_camped_cell.set_cell_power(ns_camped_cell_power)

                # Wait during DECREMENTATION_STEP_TIMER
                time.sleep(self._decrementation_step_timer)

            else:
                camped_cell_power_limit_reached = True

            if ns_neighbour_cell_power < cell_resel_neighbour_limit_power:

                # Increment idle cell power
                ns_neighbour_cell_power += incrementation_step_power
                ns_neighbour_cell.set_cell_power(ns_neighbour_cell_power)

                # Wait during INCREMENTATION_STEP_TIMER
                time.sleep(incrementation_step_timer)

            else:
                neighbour_cell_power_limit_reached = True

# ------------------------------------------------------------------------------
    def _setup_high_categories(self, eqt_id=1):
        """
        Configure equipment to specific categories via .xml files
        in order to reach specific throughput
        :type eqt_id: int
        :param eqt_id: Equipment number
        """

        # Booleans to know if specific categories must be configured
        hsupa_spec_cat = False
        hsdpa_spec_cat = False

        if self._hsdpa_cat is not None and self._hsdpa_cat > 8:
            hsdpa_spec_cat = True

        if self._hsupa_cat is not None and self._hsupa_cat > 5:
            hsupa_spec_cat = True

        # If the HSDPA or HSUPA categories needs to tune equipment
        if hsdpa_spec_cat or hsupa_spec_cat:

            # Sets PS Data HS-DSCH MAC-d PDU Size to 656
            self._mac_d_pdu_size = "BITS656"
            self._logger.warning(
                'Override MAC-d PDU size to "BITS656" '
                'in order to reach throughput target')

            # If the HSDPA category needs to tune equipment
            if hsdpa_spec_cat:

                # set cqi scheme to "USER_DEFINED"
                self._cqi_scheme = "USER_DEFINED"
                self._logger.warning(
                    'Override CQI scheme to "USER_DEFINED" '
                    'in order to reach throughput target')

                # Configure network simulator with
                # HSDPA_Categories file
                self._em.configure_equipments("HSDPACategories%d" % eqt_id,
                                              {"category": self._hsdpa_cat})

                self._logger.info("HSDPACategories %s has been set on the equipment" % self._hsdpa_cat)
            # End if hsdpa_spec_cat

            # If the HSUPA category needs to tune equipment
            if hsupa_spec_cat:

                # Configure network simulator with
                # HSUPA_Categories file
                self._em.configure_equipments("HSUPACategories%d" % eqt_id,
                                              {"category": self._hsupa_cat})

                self._logger.info("HSUPACategories %s has been set on the equipment" % self._hsupa_cat)
            # End if hsupa_spec_cat
