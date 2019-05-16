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
:summary: This script implements the interface to unitary actions for Ui features.
:since: 25/03/2011
:author: fhu2
"""

import lxml.etree as et
import os
import UtilitiesFWK.Utilities as Utils
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT
from ErrorHandling.AcsConfigException import AcsConfigException
from Core.PathManager import Paths

class DictionaryParser():

    """
    classdocs
    """
    DEFAULT_OP_DICTIONARY_FOLDER = os.path.join("..", "..", "acs_test_scripts", "Device", "UECmd", "Imp",
                                                "Android", "Common", "Ui", "OpDictionary")
    DEFAULT_OP_DICTIONARY_EXTENSION = ".xml"

    def __init__(self, device_model):
        """
        Constructor
        """

        # Initialize the op dictionary variables
        self.__op_dictionary = {}
        opdict_filename = \
            device_model.get_ui_dictionary_name() + \
            DictionaryParser.DEFAULT_OP_DICTIONARY_EXTENSION

        # Initialize default value of opDictionary file
        self._dictionary_file = None

        _execution_config_path = Paths.EXECUTION_CONFIG
        execconfig_opdict_filename = \
            os.path.join(_execution_config_path, opdict_filename)

        # Check if the op dictionary is in _ExecutionConfig folder
        if os.path.isfile(execconfig_opdict_filename):
            LOGGER_TEST_SCRIPT.info("Loading OpDictionary %s" % (execconfig_opdict_filename))
            self._dictionary_file = execconfig_opdict_filename

        # By default load OpDictionary from Device UI api folder
        else:
            LOGGER_TEST_SCRIPT.info("Loading default OpDictionary %s" % str(opdict_filename))
            op_path = os.path.join(DictionaryParser.DEFAULT_OP_DICTIONARY_FOLDER, opdict_filename)
            op_path = os.path.abspath(op_path)
            self._dictionary_file = op_path


    def parse_operation_config(self):
        """
        Parse the operation configure file. Each type of DUT has a dedicated configure file for holding
        UI operation commands
        The operation configures will be stored in a dictionary like this
        {
            sms_send: [[command, param], []]
            sms_delete: []
            sms_receive: []
        }
        """

        # Check if op dictionary exist
        if not os.path.isfile(self._dictionary_file):
            error_msg = \
                "OpDictionary file '%s' not found !" % \
                os.path.basename(str(self._dictionary_file))

            raise AcsConfigException(AcsConfigException.FILE_NOT_FOUND, error_msg)

        try:
            operation_config_doc = et.parse(self._dictionary_file)
        except et.XMLSyntaxError:
            _, error_msg, _ = Utils.get_exception_info()
            raise AcsConfigException(AcsConfigException.XML_PARSING_ERROR, error_msg)
        operation_sets = operation_config_doc.xpath("//operationsets/operationset")

        # Check if found operation sets
        if not operation_sets:
            error_msg = \
                "No operations sets found from OpDictionary file '%s' !" % \
                os.path.basename(str(self._dictionary_file))

            raise AcsConfigException(AcsConfigException.XML_PARSING_ERROR, error_msg)

        for operation_set in operation_sets:
            operation_set_name = operation_set.get('name', "")
            if not operation_set_name:
                error_msg = "one operationset declared in UI dictionary has no name defined!"
                LOGGER_TEST_SCRIPT.warning(error_msg)
                continue

            # one operation set contains several operations
            operations = operation_set.xpath("./operation")
            op_triglog = operation_set.findtext("./triglog")
            op_timeout = operation_set.findtext("./timeout")
            operation_list = []

            # each operation is combined by command and parameter
            for operation in operations:
                op = {}
                for key in ["command", "triglog", "timeout", "log"]:
                    op[key] = operation.findtext("./%s" % (key,))
                parameters = operation.xpath("./parameter")

                param_array = []
                if parameters:
                    for parameter in parameters:
                        param_array.append(parameter.text)
                op["parameters"] = param_array

                # add this operation to the operation list
                operation_list.append(op)

            operation_set_parameters = {}
            operation_set_parameters["operations"] = operation_list
            operation_set_parameters["triglog"] = op_triglog
            operation_set_parameters["timeout"] = op_timeout
            # add the operation set into operation dictionary
            self.__op_dictionary[operation_set_name] = operation_set_parameters

    def retrieve_operation_set(self, operation_set_name):
        """
        retrieve operation set from the dictionary by its name
        :param operation_set_name:  the name of operation set

        :return: the list containing operations in sequence
        """

        # Check if operation set exist in the Op dictionary
        if operation_set_name not in self.__op_dictionary.keys():
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Operation set '%s' not found !" % str(operation_set_name))

        return self.__op_dictionary[operation_set_name]
