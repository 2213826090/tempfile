"""
@summary: This test step runs iperf server on a second DUT or computer, and iperf client on the main DUT.

@since 22 May 2014
@author: Val Peterson
@organization: INTEL PEG-SVE-DSV

@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
otherwise. Any license under such intellectual property rights must be expressed
and approved by Intel in writing.
"""

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from Device.DeviceManager import DeviceManager
from acs_test_scripts.Equipment.EquipmentManager import EquipmentManager
from acs_test_scripts.Utilities.IPerfUtilities import IperfExecutionHandler
from acs_test_scripts.Utilities.NetworkingUtilities import get_dut_ipv4_address
import time


class RunIperf(DeviceTestStepBase):
    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self.server_object = None
        self.networking_api = self._device.get_uecmd("Networking")
        self.svr_networking_api = None
        self.equip_mgr = EquipmentManager()
        self.device_mgr = DeviceManager()
        self._iperf_settings = None

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        runtime_seconds = self._pars.duration * 60
        server_address = "none"
        if self._pars.iperf_protocol == "tcp":
            port_num = 5001
        else:
            port_num = 5009

        # prepare setup the IPERF test.
        self._iperf_settings = {"duration": runtime_seconds,
                                "protocol": self._pars.iperf_protocol,
                                "port_number": port_num,
                                "direction": self._pars.direction,
                                "parallel_thread": self._pars.nr_of_threads}

        bench_config = self._global_conf.benchConfig.get_dict()
        if self._pars.server_name not in bench_config.keys():
            # the indicated equipment or phone is NOT declared in the current bench config
            raise Exception("")

        # check in the bench config what is the kind of given server
        eqt_param = self._global_conf.benchConfig.get_parameters(self._pars.server_name)
        eqt_param_dict = eqt_param.get_dict()
        if "Model" in eqt_param_dict.keys():
            # server is an equipment
            eqt_param_dict = eqt_param.get_param_value("Model")
            if "REMOTE_COMPUTER" in eqt_param_dict:
                # remote computer, get the IP parameter
                server_address = eqt_param.get_param_value("IP")
                self._iperf_settings["computer"] = self.equip_mgr.get_computer(self._pars.server_name)
            elif "COMPUTER" in eqt_param_dict:
                # local computer, nothing to do
                pass
            elif "RS_CMW500" in eqt_param_dict:
                # equipment with Iperf embedded, get the IP_Lan1 parameter
                server_address = eqt_param.get_param_value("IP_Lan1")
                self._iperf_settings["equipment_api"] = \
                    self.equip_mgr.get_cellular_network_simulator().get_cell_4g().get_data()
        else:
            # server is a phone
            phone = self.device_mgr.get_device(self._pars.server_name)
            networking_api2 = phone.get_uecmd("Networking")
            self._iperf_settings["networking_api2"] = networking_api2
            if self._pars.server_net_interface is not None:
                server_address = networking_api2.get_interface_ipv4_address(self._pars.server_net_interface)
            else:
                self._logger.warning("No Server net interface provided. Use the DUT net interface")
                server_address = networking_api2.get_interface_ipv4_address(self._pars.net_interface)

        time.sleep(5)
        dut_ip_address = get_dut_ipv4_address(self.networking_api, self._pars.net_interface)
        if self._pars.direction in ("down", "both"):
            self._iperf_settings["server_ip_address"] = dut_ip_address
            self._iperf_settings["bind_host"] = server_address
        else:
            self._iperf_settings["server_ip_address"] = server_address
            self._iperf_settings["bind_host"] = dut_ip_address

        self._logger.info("RunIperf: starting")

        self._logger.info("IPERF parameters:")
        self._logger.info("  server addr: " + server_address)
        self._logger.info("  duration: %d sec" % runtime_seconds)
        self._logger.info("  direction: " + self._pars.direction)
        self._logger.info("  port: %d" % port_num)

        if self._pars.window_size.lower() != "compute":
            self._iperf_settings["server_window_size"] = self._pars.window_size
            self._iperf_settings["client_window_size"] = self._pars.window_size
        if self._pars.no_delay:
            self._iperf_settings["no_delay"] = ''
        if self._pars.iperf_options.lower() != "none":
            self._iperf_settings["extra_options"] = self._pars.iperf_options

        # Run Iperf command
        IperfHandler = IperfExecutionHandler(self._iperf_settings, self._device.get_uecmd("Networking"))
        throughput = IperfHandler.iperf()

        context.set_nested_info([self._pars.measured_throughput, "UL_VALUE"], throughput.ul_throughput.value)
        context.set_nested_info([self._pars.measured_throughput, "DL_VALUE"], throughput.dl_throughput.value)
        context.set_nested_info([self._pars.measured_throughput, "UL_UNITS"], throughput.ul_throughput.unit)
        context.set_nested_info([self._pars.measured_throughput, "DL_UNITS"], throughput.dl_throughput.unit)

        self._logger.info("Start iperf client: finished")
