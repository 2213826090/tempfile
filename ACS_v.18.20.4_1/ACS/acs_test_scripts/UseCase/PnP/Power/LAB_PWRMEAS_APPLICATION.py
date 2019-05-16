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

from UtilitiesFWK.Utilities import Global
from LAB_PWRMEAS_BASE import LabPwrMeasBase


class LabPwrMeasApplication(LabPwrMeasBase):

    """
    Class Lab Idle Power Measurement CDK in Flight Mode.
    """
    __application = None

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call power measurement base Init function
        LabPwrMeasBase.__init__(self, tc_name, global_config)

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Set up the test configuration
        """
        # Call power measurement base setup function
        LabPwrMeasBase.set_up(self)

        # Wakes up the phone
        self._phonesystem_api.display_on()
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
                                       self.global_config,
                                       self._dut_config)
        self.__application.install(main_app_full_name, additional_app_full_name, arguments_full_name, url)
        self.__application.post_install()

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
#------------------------------------------------------------------------------

    def _run_test_begin(self):
        """
        Run the test
        """
        self.__application.run_begin(self._tc_parameters)

#------------------------------------------------------------------------------

    def _run_test_end(self):
        """
        Run the test
        """
        self.__application.run_end(self._tc_parameters)

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        Tear down
        """
        LabPwrMeasBase.tear_down(self)

        self.__application.uninstall()

        return Global.SUCCESS, "No errors"
