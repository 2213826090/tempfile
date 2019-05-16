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
:summary: Use Case RAT SELECTION (Dual mode)
:since: 12/05/2013
:since: 19/02/2014
:author: galagnox
"""
import time

from acs_test_scripts.UseCase.Mobility.LAB_RAT_SELECTION_BASE import LabRatSelectionBase
from UtilitiesFWK.Utilities import Global


class LabRatSelectionDualMode(LabRatSelectionBase):
    """
    Lab Rat Selection Dual Mode class.
    The dual mode will operate a dual check on NWs then a cell recellection
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

        # Set second cell always on for DUAL MODE test
        self._ns2_cell.set_cell_on()

        # Enable Data output traffic
        self._networking_api.enable_output_traffic()

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
            # ***** CHECK DUT REGISTRATION ON NS1 (ex: 3G) *****
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

            # ***** SET THE WANTED RAT 1 (ex: GSM_ONLY) *****
            self._logger.info("")
            self._logger.info("*** Setting of the first preferred network type (%s)." % self._wanted_network1)
            self.configure_preferred_rat(self._wanted_network1, False)

            # ***** CHECK DUT REGISTRATION ON NS2 (ex: 2G) *****
            # Check that DUT is attached on NS2 => ATTACHED before timeout
            self._logger.info("")
            self._logger.info("*** Check DUT register to Network 2 before %ss..." % str(self._ns2_registration_timeout))
            self.check_registration_on_ns("NS2", self._ns2_registration_timeout)
            # Check Connected Network state is seen on DUT
            self.check_networking_status_bfor_timeout(1, False)

            # Perform a reselection or go to NS1, then go back to NS2
            inc = 0
            resel_counts = self._retry_count
            while resel_counts:
                # resel counter: inc, for end logs
                inc += 1
                self._logger.info("")
                self._logger.info("*** Start of Reselection loop: %d." % inc)

                # ***** WAIT 10 seconds *****
                self._logger.info("")
                self._logger.info("*** Waiting for 10 seconds ...")
                time.sleep(10)

                # ***** SET THE WANTED RAT 2 (ex: GSM_WCDMA) *****
                self._logger.info("")
                self._logger.info("*** Setting of the second preferred network type (%s)." % self._wanted_network2)
                self.configure_preferred_rat(self._wanted_network2, False)

                # ***** CHECK DUT REGISTRATION ON NS1 (ex: 3G) *****
                self._logger.info("")
                self._logger.info("*** Check DUT register to Network 1 before %ss..." % str(self._ns1_registration_timeout))
                self.check_registration_on_ns("NS1", self._ns1_registration_timeout)
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

                # ***** MAIN TEST *****
                (verdict, output) = self.do_ratsel_main_test("NS1")
                output_msg += output
                if verdict == Global.FAILURE:
                    return verdict, output_msg

                if "TRUE" in self._resel:
                    # wait for 5 second
                    self._logger.info("")
                    self._logger.info("*** Waiting for 5 seconds ...")
                    time.sleep(5)

                    # ***** Ensure mobile move to IDLE mode before reselection
                    self._logger.info("")
                    self._logger.info("*** Ensure DUT move to IDLE mode on Network 1...")
                    (verdict, output) = self._go_to_idle_state("NS1")
                    output_msg += output
                    if verdict == Global.FAILURE:
                        return verdict, output_msg

                    # ***** Perform reselection from NS1 to NS2 (ex: 3G -> 2G) without changing wanted rat 2
                    # (rat 2 should include rat 1)
                    msg = "*** Begin to decrease Camped cell power from %.2f dBm "
                    msg += "to %.2f dBm each %d seconds by step of %.2f dBm while "
                    msg += "cell reselection isn't performed."
                    self._logger.info("")
                    self._logger.info(msg,
                                      self._ns2_cell_power,
                                      self._cresel_limit_power,
                                      self._decrementation_step_timer,
                                      self._decrementation_step_power)
                    # Decrease cell power on of NS1 and wait for DUT to be camped to NS2 (ex: Decrease 3G to 2G)
                    # without pdp context (idle mode)
                    self.decrease_cell_power_while_idle_no_pdp(self._ns1_cell,
                                                              self._ns1_cell_power,
                                                              self._ns2_cell,
                                                              self._ns2_data,
                                                              self._decrementation_step_power,
                                                              self._decrementation_step_timer,
                                                              self._cresel_limit_power,
                                                              self._cresel_power,
                                                              self._ns2_model)

                # ***** Go to NS2 by changing rat (ex: 3G -> 2G)
                else:
                    # ***** WAIT 10 seconds *****
                    self._logger.info("")
                    self._logger.info("*** Waiting for 10 seconds ...")
                    time.sleep(10)

                    # ***** SET THE WANTED RAT 1 (ex: GSM_ONLY) *****
                    self._logger.info("")
                    self._logger.info("*** Setting of the first preferred network type (%s)." % self._wanted_network1)
                    self.configure_preferred_rat(self._wanted_network1, False)

                    # ***** CHECK DUT REGISTRATION ON NS2 (ex: 2G) *****
                    # Check that DUT is attached on NS2 => ATTACHED before timeout
                    self._logger.info("")
                    self._logger.info("*** Check DUT register to Network 2 before %ss..." % str(self._ns2_registration_timeout))
                    self.check_registration_on_ns("NS2", self._ns2_registration_timeout)
                    # Check Connected Network state is seen on DUT
                    self.check_networking_status_bfor_timeout(1, False)

                # ***** MAIN TEST 2 *****
                (verdict, output) = self.do_ratsel_main_test("NS2")
                output_msg += output
                if verdict == Global.FAILURE:
                    return verdict, output_msg

                # decrement reselection counter
                resel_counts -= 1
                self._logger.info("")
                self._logger.info("*** End of Reselection loop: %d." % inc)

            if self._retry_count == 1:
                # ***** REBOOT DEVICE *****
                self._logger.info("")
                self._logger.info("*** Restart the DUT...")
                self._device.reboot(wait_settledown_duration=True)

                # ***** CHECK DUT REGISTRATION STILL ON NS2 *****
                self._logger.info("")
                self._logger.info("*** Check DUT register to Network 2 on boot...")
                self.check_registration_on_ns("NS2")
                # Check Connected Network state is seen on DUT
                self.check_networking_status_bfor_timeout(1, False)

                # ***** Ensure mobile move to IDLE mode
                self._logger.info("")
                self._logger.info("*** Ensure DUT move to IDLE mode on Network 2...")
                self._networking_api.disable_output_traffic()
                (verdict, output) = self._go_to_idle_state("NS2")
                output_msg += output
                if verdict == Global.FAILURE:
                    return verdict, output_msg

                # ***** CHECK RAT REMAIN SAME *****
                # Check Connected Network state is seen on DUT
                if self.check_networking_status_bfor_timeout(1, True):
                    rat_after_reboot = self._networking_api.get_preferred_network_type()
                if "TRUE" in self._resel:
                    wanted_network_to_compare = self._wanted_network2
                else:
                    wanted_network_to_compare = self._wanted_network1
                if rat_after_reboot != wanted_network_to_compare:
                    verdict = Global.FAILURE
                    output = "The preferred network after reboot is not same  (expected: %s, found: %s)"\
                             % (str(wanted_network_to_compare), str(rat_after_reboot))
                    self._logger.error(output)
                    output_msg += output
                    return verdict, output_msg
                else:
                    self._logger.info("")
                    self._logger.info("*** The preferred network after reboot is the same (expected: %s, found: %s)"
                                          % (str(wanted_network_to_compare), str(rat_after_reboot)))
            else:
                # ***** WAIT 10 seconds *****
                self._logger.info("")
                self._logger.info("*** Waiting for 10 seconds ...")
                time.sleep(10)

            # Reset Default Preferred RAT
            if self._default_rat is not None:
                self._logger.debug("Setting of the initial default preferred network type (%s)." % self._default_rat)
                self.configure_preferred_rat(self._default_rat)
            else:
                self._logger.warning("No default rat parameter found on device catalog, unable to reset rat")

            if self._retry_count == 1:
                # ***** CHECK DUT STAY REGISTERED ON NS2 *****
                self._logger.info("")
                self._logger.info("*** Check DUT stay register on Network 2...")
                self.check_registration_on_ns("NS2")
                # Check Connected Network state is seen on DUT
                self.check_networking_status_bfor_timeout(1, False)
            else:
                # ***** CHECK DUT REGISTRATION ON NS1 *****
                self._logger.info("")
                self._logger.info("*** Check DUT register to Network 1 before %ss..." % str(self._ns1_registration_timeout))
                self.check_registration_on_ns("NS1", self._ns1_registration_timeout)
                # Check Connected Network state is seen on DUT
                self.check_networking_status_bfor_timeout(1, False)

        except Exception as exception:
            msg = "Error: %s. " % str(exception)
            self._logger.error(msg)
            verdict = Global.FAILURE
            output_msg += msg

            # Set initial cell power
            self._ns1_cell.set_cell_power(self._ns1_cell_power)
            self._ns2_cell.set_cell_power(self._ns2_cell_power)

        return verdict, output_msg
