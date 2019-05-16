#!/usr/bin/env python
"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related to the source code ("Material") are owned by
Intel Corporation or its suppliers or licensors. Title to the Material remains with Intel Corporation or its suppliers
and licensors. The Material contains trade secrets and proprietary and confidential information of Intel or its
suppliers and licensors.

The Material is protected by worldwide copyright and trade secret laws and treaty provisions. No part of the Material
may be used, copied, reproduced, modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual property right is granted to or conferred
upon you by disclosure or delivery of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express and approved by Intel in writing.

:organization: INTEL MCG
:summary: This file implements the scenario runner for the RF Attenuator.
:since: 2014-08-13
:author: emarchan

"""

from os import path as os_path
from optparse import OptionParser
from ScenarioManager import ScenarioManager
try:
    from acs_test_scripts.Equipment.RFAttenuator.McRCDAT.McRCDAT import McRCDAT
except:
    from sys import path as sys_path
    script_path = os_path.dirname(os_path.realpath(__file__))
    sys_path.append(os_path.join(script_path, '..'))
    from McRCDAT.McRCDAT import McRCDAT

DEFAULT_IP_BASE_ADDR = "192.168.1.30"
class ScenarioRun(object):
    def __init__(self):
        """
        Creator
        """
        self._verbose = False
        self._ip_base_addr = DEFAULT_IP_BASE_ADDR

        # Patch classes to allow them to run in standalone
        McRCDAT._log_info = self.__my_info
        McRCDAT._log_error = self.__my_error
        McRCDAT._log_debug = self.__my_debug
        ScenarioManager._log_info = self.__my_info
        ScenarioManager._log_error = self.__my_error
        ScenarioManager._log_debug = self.__my_debug

    def set_verbosity(self, value):
        """
        Enables or disables debug messages

        :type value: boolean
        :param value: True to enable debug messages, False else.
        """
        self._verbose = value

    def set_dry_mode(self, value):
        """
        Enables or disables dry mode. If dry mode is set, no command will be sent to the device

        :type value: boolean
        :param value: True to enable dry run, False else.
        """
        if value is True:
            self.__my_info("Dry mode enable, no command will be sent to the device.")
            McRCDAT._open_uri = self.__my_urlopen
        else:
            McRCDAT._open_uri = McRCDAT._open_uri

    def set_ip_base(self, value):
        """
        Sets the address of the 1st attenuator (assuming, the others will follow in range).

        :type value: string
        :param value: Base IP address
        """
        self._ip_base_addr = value

    def run_scenario(self, scenario_uri, time_between_set):
        """
        Executes the scenario.

        :type scenario_uri: string
        :param scenario_uri: location of the scenario. It can be a local file or an URI
        :type time_between_set: int
        :param time_between_set: Number of second between each attenuation change.

        """
        # Instantiate the scenario manager
        scenario_mgr = ScenarioManager(options.scenario_file, int(options.time_between_set))
        scenario_mgr.set_fixed_time_between_set(time_between_set)
        # Parse the scenario
        scenario_mgr.create_attn_list()

        # Instantiate the equipments needed to run the scenario
        scenario_mgr.instantiate_attenuators(McRCDAT, self._ip_base_addr)

        # Execute the scenario
        scenario_mgr.run_scenario()

    """
    Stubs to allow standalone execution
    """
    def __my_urlopen(self, url):
        """
        Simulates an URI opening result

        :type uri: string
        :param uri: URI to open.
        """
        if url.endswith("MN?"):
            attn_response = "RCDAT-6000-60"
        elif url.endswith("ATT?"):
            attn_response = "10"
        else:
            attn_response = "1"
        return attn_response

    def __my_info(self, msg):
        """
        Logs an info

        :type msg: string
        :param msg: Message for the logging.
        """
        print("INFO: " + msg)

    def __my_error(self, msg):
        """
        Logs an error

        :type msg: string
        :param msg: Message for the logging.
        """
        print("ERROR: " + msg)

    def __my_debug(self, msg):
        """
        Logs a debug info

        :type msg: string
        :param msg: Message for the logging.
        """
        if self._verbose is True:
            print("DBG: " + msg)

"""
Section to run the script in standalone
"""
if __name__ == "__main__":

    script_path = os_path.dirname(os_path.realpath(__file__))

    usage = "usage: %prog [OPTIONS]"
    parser = OptionParser(usage=usage)
    parser.add_option("-t", "--time",
                               help="Time between the attenuation set [default: %default].",
                               metavar="<TIME IN SECs>",
                               default="1",
                               dest="time_between_set")
#
    parser.add_option("-i", "--input",
                          help="CSV file containing the scenario.  [default: %default]",
                          metavar="<FILE>",
                          default='scenario.csv',
                          dest="scenario_file")

    parser.add_option("-a", "--address_base",
                               help="Address of the 1st attenuator (assuming, the others will follow in range). [default: %default].",
                               metavar="<IP ADDRESS>",
                               default="192.168.1.30",
                               dest="address_base")

    parser.add_option("-v", "--verbose",
                          help="Prints debug messages during execution [default: %default]",
                          action="store_true", dest="verbose", default=False)

    parser.add_option("-d", "--dry_run",
                          help="Do not execute action on the device, just display them. [default: %default]",
                          action="store_true", dest="dry_run", default=False)
    (options, args) = parser.parse_args()

    # Instantiate the scenario runner
    scenario_runner = ScenarioRun()

    # Set its parameters.
    scenario_runner.set_verbosity(options.verbose)
    scenario_runner.set_dry_mode(options.dry_run)

    # Launch execution.
    scenario_runner.set_ip_base(options.address_base)
    scenario_runner.run_scenario(options.scenario_file, int(options.time_between_set))
