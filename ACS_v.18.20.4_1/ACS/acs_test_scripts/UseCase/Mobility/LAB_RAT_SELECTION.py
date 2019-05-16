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
:summary: Use Case RAT SELECTION (Single mode)
:since: 15/05/2013
:author: sjamaoui
:since: 19/02/2014
:author: galagnox
"""
import time

from acs_test_scripts.UseCase.Mobility.LAB_RAT_SELECTION_BASE import LabRatSelectionBase
from UtilitiesFWK.Utilities import Global


class LabRatSelection(LabRatSelectionBase):
    """
    Lab Rat Selection class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LabRatSelectionBase Init function
        LabRatSelectionBase.__init__(self, tc_name, global_config)

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Set up the test configuration
        """
        LabRatSelectionBase.set_up(self)

        # Set first cell on
        self._ns1_cell.set_cell_on()

        # Set second cell on if requested
        if self._ns2_delay_cell_activation == 0:
            self._ns2_cell.set_cell_on()

        # Disable flight mode
        self._networking_api.set_flight_mode("off")

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml
        self._modem_api.check_cdk_registration_bfor_timeout(self._registration_timeout)

        # Attempt to register on first cell (DUT should register with default RAT)
        self.check_registration_on_ns("NS1")

        # Check Connected Network state is seen on DUT
        self.check_networking_status_bfor_timeout(1, False)

        # Setting of the initial network type
        if self._default_rat is not None:
            self._logger.debug("Setting of the initial default preferred network type (%s)." % self._default_rat)
            self.configure_preferred_rat(self._default_rat, True)
        else:
            self._logger.warning("No default rat parameter found on device catalog, unable to reset rat")

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test:
        """
        # Init test
        # Unknown RAT is 12 value
        rat_after_reboot = 12
        output_msg = ""
        # Call LabRatSelectionBase run_test function
        LabRatSelectionBase.run_test(self)

        # ***** REBOOT DEVICE *****
        self._logger.info("")
        self._logger.info("*** Restart the DUT...")
        self._device.reboot(wait_settledown_duration=True)

        try:
            # ***** CHECK DUT REGISTRATION ON NS1 *****
            self._logger.info("")
            self._logger.info("*** Check DUT register to Network 1 on boot...")
            self.check_registration_on_ns("NS1")
            # Check Connected Network state is seen on DUT
            self.check_networking_status_bfor_timeout(1, False)

            # ***** Ensure mobile move to IDLE mode
            self._logger.info("")
            self._logger.info("*** Ensure DUT move to IDLE mode on Network 1...")
            self._networking_api.disable_output_traffic()
            (verdict, output) = self._go_to_idle_state("NS1")
            output_msg += output
            if verdict == Global.FAILURE:
                return verdict, output_msg

            # ***** WAIT 10 seconds *****
            self._logger.info("")
            self._logger.info("*** Waiting for 10 seconds ...")
            time.sleep(10)

            # ***** SET THE WANTED RAT 2 *****
            self._logger.info("")
            self._logger.info("*** Setting of the second preferred network type (%s)." % self._wanted_network2)
            self.configure_preferred_rat(self._wanted_network2, False)

            # ***** ACTIVATE CELL 2 WITH DELAY or ALREADY ON *****
            if self._ns2_delay_cell_activation > 0:
                # Check no service on Network 2 cell
                self._logger.info("")
                self._logger.info("*** Check DUT goes to no service before %ss." % str(self._registration_timeout))
                self._modem_api.check_cdk_no_registration_bfor_timeout(self._registration_timeout)
                # wait the NS2 activation delay
                self._logger.info("")
                self._logger.info("*** Wait %ss before Network 2 cell activation." % str(self._ns2_delay_cell_activation))
                time.sleep(self._ns2_delay_cell_activation)
                # verify the DUT status
                status_dut = self._modem_api.get_network_registration_status()
                self._logger.info("Current DUT registration status is: %s", status_dut)
                # verify the DUT connection on NS
                if status_dut != "unregistered":
                    verdict = Global.FAILURE
                    output = "DUT still registered on Network 1"
                    self._logger.error(output)
                    output_msg += output
                    return verdict, output_msg
                else:
                    self._logger.info("DUT is unregistered from Network 1, Enable Network 2 cell.")
                    # activate the Second cell
                    self._ns2_cell.set_cell_on()

            # ***** CHECK DUT REGISTRATION ON NS2 *****
            self._logger.info("")
            self._logger.info("*** Check DUT register to Network 2 before %ss..." % str(self._ns2_registration_timeout))
            self.check_registration_on_ns("NS2", self._ns2_registration_timeout)
            # Check Connected Network state is seen on DUT
            self.check_networking_status_bfor_timeout(1, False)

            if self._ns2_delay_cell_activation > 0:
                # ***** Ensure mobile move to IDLE mode
                self._logger.info("")
                self._logger.info("*** Ensure DUT move to IDLE mode on Network 2...")
                self._networking_api.disable_output_traffic()
                (verdict, output) = self._go_to_idle_state("NS2")
                output_msg += output
                if verdict == Global.FAILURE:
                    return verdict, output_msg

            # ***** MAIN TEST *****
            (verdict, output) = self.do_ratsel_main_test("NS2")
            output_msg += output
            if verdict == Global.FAILURE:
                return verdict, output_msg

            # ***** REBOOT DEVICE *****
            self._logger.info("")
            self._logger.info("*** Restart the DUT...")
            self._device.reboot(wait_settledown_duration=True)

            # ***** CHECK RAT REMAIN THE SAME *****
            # Check Connected Network state is seen on DUT
            if self.check_networking_status_bfor_timeout(1, True):
                rat_after_reboot = self._networking_api.get_preferred_network_type()
            if rat_after_reboot != self._wanted_network2:
                verdict = Global.FAILURE
                output = "The preferred network after reboot is not same (expected: %s, found: %s)"\
                         % (self._wanted_network2, str(rat_after_reboot))
                self._logger.error(output)
                output_msg += output
                return verdict, output_msg
            else:
                self._logger.info("")
                self._logger.info("*** The preferred network after reboot is the same (expected: %s, found: %s)"
                                  % (self._wanted_network2, str(rat_after_reboot)))

            # ***** CHECK DUT REGISTRATION ON NS2 *****
            self._logger.info("")
            self._logger.info("*** Check DUT register to Network 2 on boot...")
            self.check_registration_on_ns("NS2")
            # Check Connected Network state is seen on DUT
            self.check_networking_status_bfor_timeout(1, False)

            if self._ns2_delay_cell_activation > 0:
                # ***** Ensure mobile move to IDLE mode
                self._logger.info("")
                self._logger.info("*** Ensure DUT move to IDLE mode on Network 2...")
                self._networking_api.disable_output_traffic()
                (verdict, output) = self._go_to_idle_state("NS2")
                output_msg += output
                if verdict == Global.FAILURE:
                    return verdict, output_msg

        except Exception as exception:
            msg = "Error: %s. " % str(exception)
            self._logger.error(msg)
            verdict = Global.FAILURE
            output_msg += msg

            # Set ns2 initial state
            if self._ns2_delay_cell_activation > 0:
                self._ns2_cell.set_cell_off()

        finally:
            # Reset RAT
            self._logger.info("")
            self._logger.info("*** Reset DUT RAT to default preferred network type...")

            # Check Connected Network state is seen on DUT
            self.check_networking_status_bfor_timeout(1, False)

            # Reset Default Preferred RAT
            if self._default_rat is not None:
                self._logger.debug("Setting of the initial default preferred network type (%s)." % self._default_rat)
                self.configure_preferred_rat(self._default_rat)
            else:
                self._logger.warning("No default rat parameter found on device catalog, unable to reset rat")

            if self._ns2_delay_cell_activation == 0:
                # ***** CHECK DUT REGISTRATION ON NS1 *****
                self._logger.info("")
                self._logger.info("*** Check DUT register to Network 1 before %ss..." % str(self._ns1_registration_timeout))
                self.check_registration_on_ns("NS1", self._ns1_registration_timeout)
                # Check Connected Network state is seen on DUT
                self.check_networking_status_bfor_timeout(1, False)
            else:
                # ***** CHECK DUT STAY REGISTERED ON NS2 *****
                self._logger.info("")
                self._logger.info("*** Check DUT stay register on Network 2...")
                # Check that DUT is attached on NS => ATTACHED before timeout
                self._ns2_data.check_data_connection_state("ATTACHED",
                                                           self._ns2_registration_timeout,
                                                           blocking=True)
                # Check that DUT is registered on the good RAT
                self._modem_api.check_network_type_before_timeout(self._ns2_data.get_network_type(), self._registration_timeout)
                        # Check Connected Network state is seen on DUT
                self.check_networking_status_bfor_timeout(1, False)

        return verdict, output_msg
