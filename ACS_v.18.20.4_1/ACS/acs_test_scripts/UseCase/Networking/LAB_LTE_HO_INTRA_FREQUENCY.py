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
:summary:  This file implements usecase that do FTP over LTE network
:since: 17/04/2013
:author: hbianx
"""
from Queue import Queue
from threading import Thread

import time
import re
from LAB_LTE_BASE import LabLteBase
from UtilitiesFWK.Utilities import Global, run_local_command
from thread import start_new_thread
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException


class LabLteHoIntraFrequency(LabLteBase):

    """
    LAB_LTE_HO_INTRA_FREQUENCY
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LAB_LTE_BASE Init function
        # All the base config will be set for CELL A
        LabLteBase.__init__(self, tc_name, global_config)
        if self._cell_id == "A":
            self._alt_cell_id = "B"
        else:
            self._alt_cell_id = "A"
        # Read PHYSICAL_CELL_B_ID from test case xml file
        self._physical_cell_b_id = \
            str(self._tc_parameters.get_param_value("PHYSICAL_CELL_B_ID"))

        # Read SCENARIO_PATH for cell B from test case xml file
        self._scenario_path_b = \
            str(self._tc_parameters.get_param_value("SCENARIO_PATH_B"))

        # Read the data size of a packet
        self._packet_size = self._tc_parameters.get_param_value("PACKET_SIZE")

        self._rrc_state = self._tc_parameters.get_param_value("RRC_STATE", "RRC_CONNECTED")
        if self._rrc_state == "RRC_CONNECTED":
            self._connection_state = "CON"
        else:
            self._connection_state = "IDLE"
        self._alt_cell_4g = self._ns.get_alt_cell_4g()
        self._alt_data_4g = self._alt_cell_4g.get_data()
#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call UseCaseBase set_up function because LabLteBaseBase
        # is not relevant according multiple cell config and scenario starting
        UseCaseBase.set_up(self)

        # Ensure flight mode off for above check
        self._networking_api.set_flight_mode("off")

        time.sleep(self._wait_btwn_cmd)

        # Determinates the kind of registration state to wait
        # by comparing test case MCC/MNC parameters to sim MCC/MNC
        sim_info = self._modem_api.get_sim_operator_info()
        if sim_info["MCC"] != self._mcc or\
                sim_info["MNC"] != self._mnc:
            self._wanted_reg_state = "roaming"
            self._networking_api.set_roaming_mode("ON")
        else:
            self._wanted_reg_state = "registered"

        message = "Expected network registration state to be %s while " \
            "HPMLN coverage is %s, MCC is %s and MNC is %s" % (
                str(self._wanted_reg_state),
                str(self._hplmn_coverage),
                str(self._mcc),
                str(self._mnc))
        self._logger.info(message)

        # Clear all data connections
        self._networking_api.clean_all_data_connections()
        time.sleep(self._wait_btwn_cmd)

        # Flight mode activation
        self._networking_api.set_flight_mode("on")

        # Connect to equipment
        self._ns.init()

        # Perform a full preset
        self._ns.perform_full_preset()

        # Wait a while for the full preset
        time.sleep(self._wait_btwn_cmd)

        # Set EPC off
        self._ns_data_4g.set_epc_off()

        # Set cell-A off
        self._ns_cell_4g.set_cell_off()

        # Set cell-B off
        self._alt_cell_4g.set_cell_off()

        # load the scenario for cell-A
        self._ns.load_configuration_file(self._scenario_path)

        # Configure the Cell-B used for the test
        # select the cell-B
        self._ns_cell_4g.set_cell_setup(self._alt_cell_id)
        # load the scenario for cell-B
        self._ns.load_configuration_file(self._scenario_path_b)

        # end of cell-B configuration; reset cell-A
        self._ns_cell_4g.set_cell_setup(self._cell_id)

        # ensure cell-A is 100%
        self._ns_cell_4g.set_cell_ratio(100)

        self._ns_data_4g.set_ip_address_type(self._ip_version)
        time.sleep(5)
        if self._ip_version in ("IPV4", "IPV6"):
            self._ns_data_4g.set_dut_ip_address(self._ns_dut_ip_Address)
            self._alt_data_4g.set_dut_ip_address(self._ns_dut_ip_Address)
        else:
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG)

        # Set APN on Cell-A
        self._ns_data_4g.set_apn(self._apn)
        # Set APN on Cell-B
        self._alt_data_4g.set_apn(self._apn)

        self._ns_data_4g.set_epc_on()

        # Start the scenario
        self._ns.start_scenario()

        # Set Cell on
        self._ns_cell_4g.set_cell_on(self._mimo)

        # Flight mode deactivation
        self._networking_api.set_flight_mode("off")

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml (Non blocking
        # for this test if function isn't implemented on CDK)
        self._modem_api.check_cdk_registration_bfor_timeout(self._registration_timeout)

        # Set the APN
        self._networking_api.set_apn(self._ssid, self._apn)

        self._networking_api.activate_pdp_context(check=False)

        # Check Data Connection State => CON before timeout
        self._check_data_connection_state("CON")

        # Get RAT from Equipment
        network_type = self._ns_data_4g.get_network_type()
        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(network_type,
                                                          self._registration_timeout)

        if self._rrc_state == "RRC_IDLE":
            self._networking_api.disable_output_traffic()
            self._ns_data_4g.ue_detach()
            self._networking_api.enable_output_traffic()

        current_rrc_state = self._ns_cell_4g.get_rrc_state()
        self._logger.info("The ping will start from %s state" % current_rrc_state)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        Test steps:
            Initiate continuous MO 1000 byte ping for 30 sec -> successful on cell-A
            Initiate continuous MO 1000 byte ping for 1 min -> ping start successfully
            (during ping) Align RSRP cells to same level
            (during ping) <PS handover from Cell-A to Cell-B is triggered from network>
            Initiate continuous MO 1000 byte ping for 1 min -> ping start successfully
            (during ping) <PS handover from Cell-A to Cell-B is triggered from network>
        """
        self._logger.info("Start ping on Cell-A for 30s")
        packet_loss = self._networking_api.ping(self._server_ip_address, self._packet_size, 30)
        if packet_loss.value > 0:
            msg = "First ping on Cell-A failed - Value : %0.2f %% (Expected: < 0%%)" % packet_loss.value
            self._logger.error(msg)
            return Global.FAILURE, msg
        else:
            msg = "First ping on Cell-A succeed - Value : %0.2f %% (Expected: < 0%%)" % packet_loss.value
            self._logger.info(msg)

        self._logger.info("Start ping on Cell-A for 1min long")
        threaded_ping = Threaded(self._networking_api.ping, (self._server_ip_address, self._packet_size, 60))
        threaded_ping.start()

        self._logger.info("Change cell ratio to 50%")
        self._ns_cell_4g.set_cell_ratio(50)
        time.sleep(self._wait_btwn_cmd)
        # check that cell-A is still connected with 50%
        self._check_data_connection_state(self._connection_state)
        time.sleep(self._wait_btwn_cmd)

        self._logger.info("Sending custom command 'Measurement Report'")
        self._ns_cell_4g.send_custom_message("Measurement Report")
        time.sleep(10)
        self._logger.info("Sending handover message 'B1 RRC Handover %s to %s'" % (self._cell_id, self._alt_cell_id))
        self._ns_cell_4g.send_handover_message("B1 RRC Handover %s to %s" % (self._cell_id, self._alt_cell_id))

        # check if cell-B is connected
        self._logger.info("Check data connection state")
        self._ns_cell_4g.set_cell_setup(self._alt_cell_id)
        is_conn_established = self._ns_data_4g.check_data_connection_state(self._connection_state,
                                          self._registration_timeout,
                                          blocking=False,
                                          cell_id=self._alt_cell_id)
        if not is_conn_established:
            msg = "Connection on %s not established" % self._alt_cell_id
            return Global.FAILURE, msg

        packet_loss = threaded_ping.get_return()
        if packet_loss.value > 0:
            msg = "Second ping on Cell-A failed - Value : %0.2f %% (Expected: < 5%%)" % packet_loss.value
            self._logger.error(msg)
            return Global.FAILURE, msg
        else:
            msg = "Second ping on Cell-A succeed - Value : %0.2f %% (Expected: < 5%%)" % packet_loss.value
            self._logger.info(msg)

        self._logger.info("Start ping on Cell-B for 1min long")
        threaded_ping = Threaded(self._networking_api.ping, (self._server_ip_address, self._packet_size, 60))
        threaded_ping.start()
        self._ns_cell_4g.send_custom_message("Measurement Report")
        time.sleep(10)
        self._logger.info("Sending handover message 'B1 RRC Handover %s to %s'" % (self._alt_cell_id, self._cell_id))
        self._ns_cell_4g.send_handover_message("B1 RRC Handover %s to %s" % (self._alt_cell_id, self._cell_id))

        packet_loss = threaded_ping.get_return()
        if packet_loss.value > 0:
            msg = "Third ping on Cell-B failed - Value : %0.2f %% (Expected: < 5%%)" % packet_loss.value
            self._logger.error(msg)
            return Global.FAILURE, msg
        else:
            msg = "Third ping on Cell-B succeed - Value : %0.2f %% (Expected: < 5%%)" % packet_loss.value
            self._logger.info(msg)

        return Global.SUCCESS, "No errors"


class Threaded(Thread):
    def __init__(self, target, args=()):
        Thread.__init__(self)
        self._target = target
        self._args = args
        self._ret = None

    def run(self):
        self._ret = self._target(*self._args)

    def get_return(self):
        self.join()
        return self._ret