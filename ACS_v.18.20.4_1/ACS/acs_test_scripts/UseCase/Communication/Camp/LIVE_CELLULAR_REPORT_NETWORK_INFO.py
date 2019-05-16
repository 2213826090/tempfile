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
:summary: Use Case Cellular report information about serving cell and
    neighboring cells.
:since: 05/07/2013
:author: asebbanx
"""
# Module name follows ACS conventions
# pylint: disable=C0103

import time

from acs_test_scripts.UseCase.Networking.LIVE_CELLULAR_BASE import LiveCellularBase
from UtilitiesFWK.Utilities import Global, str_to_bool


class LiveCellularReportNetworkInfo(LiveCellularBase):

    """
    Live Cellular report information about serving cell and
    neighboring cells.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call UseCase base Init function
        LiveCellularBase.__init__(self, tc_name, global_config)

        # Initialize attributes
        self._check_serving_cell_failed = False
        self._found_one_non_empty_neighbor = False

        # Remember whether we have to check the serving cell or not
        check_serving_cell = self._tc_parameters.get_param_value(
            "CHECK_SERVING_CELL")
        self._check_serving_cell = str_to_bool(check_serving_cell)

        # Remember whether we have to check the neighboring cells or not
        check_neighboring_cells = self._tc_parameters.get_param_value(
            "CHECK_NEIGHBORING_CELLS")
        self._check_neighboring_cells = str_to_bool(check_neighboring_cells)

        # Instantiate Networking UE Command category
        self._networking_api = self._device.get_uecmd("Networking")

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test.
        """
        # Call inherited method
        LiveCellularBase.run_test(self)

        # Initialize some variables for error messages
        serving_cell_error = "No error."

        # Reset error statuses for back-to-back iterations
        self._check_serving_cell_failed = False
        self._found_one_non_empty_neighbor = False

        #
        # Check the serving cell if requested
        #

        if self._check_serving_cell:
            # Retrieve information about the serving cell
            # from Android
            serving_cell = self._networking_api.get_serving_cell()

            # Check whether the cell is empty or not
            if serving_cell.is_empty():
                # If that is the case, raise an exception
                serving_cell_error = "Got empty cell: %s" % str(serving_cell)
                self._logger.error(serving_cell_error)
                self._check_serving_cell_failed = True
            else:
                self._logger.info("Found serving cell: %s" % str(serving_cell))

        # Wait 15 sec before reading the neighboring cells
        time.sleep(15)

        #
        # Check the neighboring cells if requested
        #
        if self._check_neighboring_cells:
            # Retrieve the neighboring cell list from Android
            neighboring_cells = self._networking_api.get_neighboring_cells()

            # Check whether the neighboring cell list contains
            # at least one non empty cell
            for cell in neighboring_cells:
                # If there is one empty cell, log it
                if cell.is_empty():
                    self._logger.debug("Empty cell: %s" % str(cell))
                # If the cell is not empty, update the
                # boolean used to compute the verdict
                else:
                    self._logger.debug("Non empty cell: %s" % str(cell))
                    self._found_one_non_empty_neighbor = True

        #
        # Compute the verdict
        #

        # If the boolean values used to compute the verdict
        # indicates an error
        test_message = []
        got_error = False
        verdict = Global.SUCCESS
        # Build a nice error message
        if self._check_serving_cell_failed:
            # Build an error message
            test_message.append(
                "Check of serving cell failed: %s." % serving_cell_error)
            got_error = True
        if self._check_neighboring_cells and not self._found_one_non_empty_neighbor:
            got_error = True
            # If we already had an error, append information to it
            test_message.append("All neighboring cells were empty.")
        # Now that we have a meaningful error message, raise an
        # exception if we have to.
        if got_error:
            verdict = Global.FAILURE
            test_message = "\n".join(test_message)
        else:
            test_message = "No error."

        # Return the verdict
        return verdict, test_message
