"""
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
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

@organization: INTEL MCG PSI
@summary: Module loader utilities
@since: 03/01/2014
@author: pbluniex
"""
import re
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT


class ParseConfiguration:
    """
    Parse configuration stream in test case configuration
    """
    def __init__(self):
        """
        Constructor

        @type dirname: str
        @param dirname: Path where to find modules
        """
        self._logger = LOGGER_TEST_SCRIPT
        self._config = {}

    def __parse_configuration(self, module):
        """
        Configuration of one module. This will enabled the module

        @type module: str
        @param module: The configuration item.
                       Each module can be activated in the test case parameter.
                       The configuration must follows the syntax (regular expression):
                           (?P<ModuleName>[^:]+)[\s*][:[\s*]config];

                       e.g.:
                           * Activate a module without configuration
                           ModuleName;

                           * Activate a module with configuration
                           ModuleName: config;
        """
        if not module:
            return

        conf = module.split(":", 1)
        mod_name = conf[0].strip()
        self._config[mod_name] = {}
        if len(conf) == 2:
            parameters = conf[1].strip()
            self._config[mod_name] = self.__get_parameters(parameters)
        elif len(conf) > 2:
            self._logger.error("Can not configure module : "
                               "invalid syntax (%s)" % module)
            return

    def __get_parameters(self, parameters):
        """
        Get parameters of configuration

        @type parameters: str
        @param parameters: string representation of the parameters of the module

                           The configuration must follow the syntax (regular expression) for litterals
                           (?P<param1>\w+)[\s*=\s*(?P<val1>\w+)]?[\s,\s*(?P<paramX>\w+[=(?P<valX>\w+)]]*

                            or use python style list or dictionaries:

                           (?P<param1>\w+)\s*=\s*[data1, data1, ...]
                           (?P<param1>\w+)\s*=\s*{key1: value1, key2: value2, ...}

                           parameters are coma separated and end of module configuration is marked with
                           semicolon.

                           e.g. :
                               * Parameter with empty value
                               param1

                               * Parameter with value
                               param1 = val1

                               * Parameters
                               param1, param2 = val2, param3

                               * Dictionary
                               param1, {"param2": "value1", ...}, ...

                               * list
                               param1, param2 = [val2_1: val2_2, ...], ...

        @rtype: dictionary
        @return: The parameters of modules compiled in a dictionary
        """
        config = {}
        pattern = "(?P<name>([a-zA-Z0-9_-]+))(=(?P<value>({[^}]*})|([a-zA-Z0-9_-]*)))?(,|$)"
        prmiter = re.finditer(pattern, parameters.replace(" ", ""))

        for param in prmiter:
            prmdict = param.groupdict()
            config[prmdict["name"]] = prmdict["value"]

        return config

    def get(self, config):
        """
        Load modules with configuration

        @type config: str
        @param config: Configuration of modules to load

        @rtype: dict
        @return: dictionary of modules. keys are the name of the module,
                 values are the instance of the modules
        """

        if config:
            modules = ("".join(line.strip()
                               for line in config.strip(" ;\n").splitlines())).split(";")
            for module in modules:
                self.__parse_configuration(module.strip())

        return self._config
