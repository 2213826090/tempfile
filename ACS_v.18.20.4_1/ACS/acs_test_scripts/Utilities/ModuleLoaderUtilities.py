"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

@organization: INTEL MCG PSI
@summary: Module loader utilities
@since: 19/07/2013
@author: pbluniex
"""
import os
import imp
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT
from acs_test_scripts.Utilities.ParseConfigurationUtilities import ParseConfiguration


class ModuleLoader:
    """
    Load modules from path and name
    """
    def __init__(self, device, dirname):
        """
        Constructor

        @type device: DeviceBase
        @param device: The DUT

        @type dirname: str
        @param dirname: Path where to find modules
        """
        self._logger = LOGGER_TEST_SCRIPT
        self.__module_dirname = dirname
        self._device = device

    def __get_module(self, name, config):
        """
        Get the module class defined by name

        @type name: str
        @param name: The name of the module

        @type config: dict
        @param config: The configuration of the module.
                       The keys of the dictionary are the parameters of the module.
                       Values are the parameter value if any, None otherwise

                       e.g. :
                       The configuration value :

                           "param1=val1, param2=val2, param3"

                       will be represented by the dictionary:

                           config = {"param1": val1,
                                     "param2": val2,
                                     "param3": None}

        @rtype: dict
        @return: The configuration compiled in a dictionary
        """
        sourcefile = None
        module_file = os.path.join(self.__module_dirname, name)
        if os.path.isfile(module_file + ".py"):
            sourcefile = imp.load_source(name, module_file + ".py")
        elif os.path.isfile(module_file + ".pyc"):
            sourcefile = imp.load_compiled(name, module_file + ".pyc")

        module = None
        if sourcefile:
            module = getattr(sourcefile, name)(self._device, config)

        if module is not None:
            self._logger.info("Module '%s' successfuly loaded" % name)
        else:
            self._logger.error("Module '%s' not loaded" % name)

        return module

    def load(self, config):
        """
        Load modules with configuration

        @type config: str
        @param config: Configuration of modules to load

        @rtype: dict
        @return: dictionary of modules. keys are the name of the module,
                 values are the instance of the modules
        """
        config = ParseConfiguration().get(config)

        modules = {}
        for mod_name, mod_conf in config.items():
            module_instance = self.__get_module(mod_name, mod_conf)
            if module_instance is not None:
                modules[mod_name] = module_instance

        return modules
