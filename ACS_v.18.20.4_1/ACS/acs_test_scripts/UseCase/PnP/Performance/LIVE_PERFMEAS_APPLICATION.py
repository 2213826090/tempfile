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
:summary: Use Case base for applications based power measurement
:since: 20/02/2013
:author: pbluniex
"""

import os
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.Utilities.PnPUtilities import PnPResults
from Core.Report.SecondaryTestReport import SecondaryTestReport


class LivePerfMeasApplication(UseCaseBase):

    """
    Class live to run benchmarks application
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor

        :type tc_name: BaseConf
        :param tc_name: Configuration of the usecase

        :type global_config: Dictionnary
        :param global_config: Global configuration of the campaign
        """
        # Call power measurement base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # Call verdict management instance
        self.__failure_file = os.path.join(self._execution_config_path,
                                           self._device.get_config("FailureFile"))
        self.__target_file = os.path.join(self._execution_config_path,
                                          self._device.get_config("TargetFile"))
        self.__application = None
        self._global_config = global_config
        self.__adbConnectionTimeout = self._device.get_config("adbConnectTimeout", 30, float)

        # If the device was disconnected before due to an error
        # we must reconnect it explicitly at the beginning of the test
        # else the commands will fail and the test will be blocked
        self._system_api = self._device.get_uecmd("System")
        return_code = self._system_api.wait_for_device(self.__adbConnectionTimeout)
        if not return_code:
            time.sleep(30)

        if not self._device.is_available():
            self._device.connect_board()

        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self._networking_api = self._device.get_uecmd("Networking")
        self._localconnectivity_api = self._device.get_uecmd("LocalConnectivity")
        self._localization_api = self._device.get_uecmd("Location")
        self._sensor_api = self._device.get_uecmd("Sensor")
        self._sysdebug_apis = self._device.get_uecmd("SysDebug")
        self._system_apis = self._device.get_uecmd("System")

        self._report_tree = global_config.campaignConfig.get("campaignReportTree")
        self._tc_name = os.path.basename(self.get_name())
        self._tc_date = ""
        self._secondary_report = SecondaryTestReport(
            self._device.get_report_tree().get_report_path())

        attributes = {"id": self._tc_name,
                      "date": self._tc_date,
                      "verdict": "NOT EXECUTED"}

        self.__results = PnPResults(self._report_tree,
                                    self._dut_config.get("Name"),
                                    self.__failure_file,
                                    self.__target_file,
                                    attributes)

    def _verdict(self):
        """
        Get verdict
        """
        stat_type = self._tc_parameters.get_param_value("STAT_TYPE")
        scores = self.__application.get_score(stat_type)
        sysreport = self._sysdebug_apis.report()

        self.__results.append(scores)
        self.__results.append(sysreport)

        try:
            ilb = self.__application.is_lower_better
            verdict, output = self.__results.get_performance_verdict(scores, ilb,
                                                                     self._secondary_report,
                                                                     self.tc_order)
            self._logger.info(output)
        finally:
            self.__results.write()

        return verdict, output

    def set_up(self):
        """
        Set up the test configuration

        :rtype: tuple
        :return: tuple of Verdict and comment
        """
        # Call power measurement base setup function
        UseCaseBase.set_up(self)

        sysdbg_modules_config = self._tc_parameters.get_param_value("SYSDEBUG_MODULES")
        self._sysdebug_apis.init(sysdbg_modules_config)

        self._networking_api.set_wifi_power("off")
        self._localconnectivity_api.set_bt_power("off")
        self._localization_api.set_gps_power("off")
        self._networking_api.set_flight_mode("on")

        application_name = self._tc_parameters.get_param_value("APPLICATION_NAME")
        # appuri contains the app name to retrieve in pnp module config
        app_key = self._tc_parameters.get_param_value("APP_NAME")
        main_app_full_name = self.__retrieve_value_from_module_config(app_key)
        if not main_app_full_name:
            # specified key is not in config, it could be a real app name
            main_app_full_name = app_key
        # additional contains additional app name to install to retrieve in pnp module config
        additionnals = self._tc_parameters.get_param_value("ADDITIONALS")
        additional_app_full_name = self.__retrieve_value_from_module_config(additionnals)
        if not additional_app_full_name:
            # specified key is not in config, it could be a real app name
            additional_app_full_name = additionnals

        arguments = self._tc_parameters.get_param_value("ARGUMENTS")
        arguments_full_name = self.__retrieve_value_from_module_config(arguments)
        if not arguments_full_name:
            arguments_full_name = arguments
        self.__loop_mode = self._tc_parameters.get_param_value("LOOP_MODE")
        url = self._tc_parameters.get_param_value("URL")
        self.__application = self._device.get_application_instance(application_name)

        self.__application.pre_install(self._execution_config_path,
                                       self._global_config,
                                       self._dut_config,
                                       self._sysdebug_apis)
        self.__application.install(main_app_full_name, additional_app_full_name, arguments_full_name, url)
        self.__application.post_install()

        self._system_apis.clear_cache()
        self._phonesystem_api.display_on()
        stabilization_time = 5
        self._logger.debug("Wait for %s seconds after display_on." % str(stabilization_time))
        time.sleep(stabilization_time)

        return Global.SUCCESS, "No errors"

    def __retrieve_value_from_module_config(self, app_key):
        """
        Retrieve the apk to install from PnpModule configuration
        """
        if app_key is None:
            return None
        # get all configuration for pnp module
        apk_full_name = None
        module_name = "PnpModule"
        module_configs = [module.configuration for module in self._device.get_device_modules(module_name)]
        for module_config in module_configs:
            if app_key.strip() in module_config:
                apk_full_name = module_config.get(app_key.strip())
        return apk_full_name

    def run_test(self):
        """
        Run the test

        :rtype: tuple
        :return: tuple of Verdict and comment
        """
        UseCaseBase.run_test(self)

        self._tc_date = time.strftime("%Y-%m-%d %H:%M:%S")
        self.__results.update({"date": self._tc_date})

        self._sysdebug_apis.reset()
        self._sysdebug_apis.start()
        self.__application.run(self._tc_parameters)
        self._sysdebug_apis.fetch()
        self._sysdebug_apis.stop()

        return self._verdict()

    def tear_down(self):
        """
        Tear down

        :rtype: tuple
        :return: tuple of Verdict and comment
        """
        UseCaseBase.tear_down(self)

        if self.__application is not None:
            self.__application.uninstall()
        #some application may have change the brightness value
        brightness = int(self._dut_config.get("displayBrightness"))
        if brightness is not None:
            new_brightness = int(self._phonesystem_api.set_display_brightness(brightness))
            if abs(new_brightness - brightness) > 1:
                self._logger.warning("Requested brightness was %s, brightness has been set to %s" %
                                     (brightness, new_brightness))
        self._phonesystem_api.display_off()

        return Global.SUCCESS, "No errors"
