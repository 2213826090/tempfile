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
:summary: implementation of Cisco1260 family configurable AP
:since:22/03/2012
:author: apairex
.. note:: BZ2969
"""

from Core.FileParsingManager import FileParsingManager
from acs_test_scripts.Equipment.ConfigurableAP.Cisco1250.Cisco1250 import Cisco1250
import UtilitiesFWK.Utilities as Util


class Cisco1260(Cisco1250):

    """
    Implementation of Cisco1260 configurable AP
    """

    # Power values
    POWER_VALUES_2G = ["-1", "2", "5", "8", "11", "14", "max"]
    POWER_VALUES_5G = ["-1", "2", "5", "8", "11", "14", "17", "max"]

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        """
        # Initialize class parent
        Cisco1250.__init__(self, name, model, eqt_params, bench_params)

    def __del__(self):
        """
        Destructor: releases all allocated resources.
        """
        Cisco1250.__del__(self)


# Logger class for unit tests
class UnitTestLogger():

    def __init__(self):
        pass

    def debug(self, msg):
        print msg

    def info(self, msg):
        print msg

    def error(self, msg):
        print msg

# Unit tests
if __name__ == "__main__":
    bench_config = "bench_config_CTP"
    equipment_catalog = "Equipment_Catalog"
    global_config = Util.Dictionary(attribute1='benchConfig',
                                    attribute2='deviceConfig',
                                    attribute3='campaignConfig',
                                    attribute4='equipmentCatalog',
                                    attribute5='campaignReportTree')
    global_config.benchConfig = {}
    global_config.deviceConfig = {}
    global_config.campaignConfig = {}
    global_config.equipmentCatalog = {}
    global_config.campaignReportTree = None
    flash_file_path = None

    file_parsing_manager = FileParsingManager(
        bench_config,
        equipment_catalog,
        global_config)
    file_parsing_manager.parse_bench_config()

    bench_name = "CONFIGURABLE_AP1"
    equipment_model = "CISCO_1260"
    equipment_dict = {u'CISCO_WRVS4400N': {}, u'CISCO_AP541N': {},
                      u'DLINK_DAP2553': {}, u'CISCO_WAP4410N': {},
                      u'CISCO_1260': {}, u'CISCO_1250': {}}

    ap = Cisco1260(bench_name, equipment_model, equipment_dict,
                   global_config['benchConfig'].get_parameters("CONFIGURABLE_AP1"))

    ap._logger = UnitTestLogger()  # pylint: disable=W0212

    ap.init()

    # Change following method parameters to test different configuration
    ap.set_wifi_config(ssid="UNIT_TEST",
                       hidden=False,
                       standard_type="n5G",
                       authentication_type="WPA2-PSK-AES",
                       passphrase="1234567890123",
                       channel=48,
                       dtim=None,
                       beacon=None,
                       wmm=None,
                       bandwidth=None,
                       mimo=True,
                       radiusip="192.168.0.150",
                       radiusport="1812",
                       radiussecret="RadiusPass")
    ap.release()
