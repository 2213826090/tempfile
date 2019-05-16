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
:summary: Registration Utilities classes and function for Data
:since: 05/04/2012
:author: cbresoli
"""
import os
import shutil
import time
import yaml

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Equipment.EquipmentManager import EquipmentManager as EM
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.TestEquipmentException import TestEquipmentException
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT


class CellInfo(object):

    """
    A base class for GsmCellInfo and CdmaCellInfo.
    """

    EMPTY_CELL = "empty"
    """
    The key value indicating an empty cell.
    """

    NEIGHBORING_CELL_CLASS_NAME = "NeighboringCellInfo"
    """
    The Java class name corresponding to a I{neighboring cell}
    description.
    """

    CDMA_CLASS_NAME = "CdmaCellLocation"
    """
    The Java class name corresponding to a I{CDMA} cell.
    """

    GSM_CLASS_NAME = "GsmCellLocation"
    """
    The Java class name corresponding to a I{GSM} cell.
    """

    KEY_CELL_TYPE = "type"
    """
    The key corresponding to the cell type information.
    """

    def is_empty(self):
        """
        Returns a boolean indicating whether this instance represents
        an empty cell or not.

        This method is intended for overriding.

        :rtype: bool
        :return:
            - C{True} if this cell is empty
            - C{False} otherwise
        """
        return True

    @classmethod
    def is_empty_value(cls, value):
        """
        Returns a boolean value indicating whether the given
        value shall be considered as an empty field for this
        CellInfo instance or not.
        :rtype: bool
        :return:
            - C{True} if this field shall be considered as empty
            - C{False} otherwise
        """
        # By default we consider that the value is empty
        empty = True
        # Check the value
        if value not in (None, -1, "-1", ""):
            empty = False
        # Return the empty status
        return empty

    @classmethod
    def from_dict(cls, values):
        """
        Returns a new subclass instance built from the given values.

        :param values: the values to use in order to build the
            C{CellInfo} subclass instance
        :type values: dict

        :rtype: CellInfo
        :return: a CellInfo subclass instance (or C{None}).
        """
        # Initialize the return value
        instance = None
        # Consider whether we have to create a GSM cell info
        # or something else.
        if values[CellInfo.KEY_CELL_TYPE] == CellInfo.GSM_CLASS_NAME:
            # Build a GSM cell
            instance = GsmCellInfo.from_dict(values)
        elif values[CellInfo.KEY_CELL_TYPE] == CellInfo.CDMA_CLASS_NAME:
            # Build a CDMA cell
            instance = CdmaCellInfo.from_dict(values)
        elif values[CellInfo.KEY_CELL_TYPE] == CellInfo.NEIGHBORING_CELL_CLASS_NAME:
            # Build a neighboring cell
            instance = NeighboringCellInfo.from_dict(values)
        elif values[CellInfo.KEY_CELL_TYPE] == CellInfo.EMPTY_CELL:
            # Build an empty cell
            instance = CellInfo()
        else:
            # Force the return value to None anyway
            instance = None
        # Return the instance
        return instance

    def __str__(self):
        """
        Returns a string representation of this object.

        :rtype: str
        :return: the string representation of this object
        """
        return "CellInfo[]"


class NeighboringCellInfo(CellInfo):

    """
    This class holds all information representing a
    I{neighboring cell}.
    """

    KEY_CELL_ID = "cid"
    """
    The key corresponding to the I{cell id} information.
    """

    KEY_LAC = "lac"
    """
    The key corresponding to the I{LAC} information.
    """

    KEY_RSSI = "rssi"
    """
    The key corresponding to the I{RSSI} information.
    """

    KEY_NETWORK_TYPE = "network_type"
    """
    The key corresponding to the I{network type} information.
    """

    def __init__(self):
        """
        Initializes this instance.
        """
        CellInfo.__init__(self)
        self.__cid = None
        self.__lac = None
        self.__rssi = None
        self.__network_type = None

    def is_empty(self):
        """
        Returns a boolean indicating whether this instance represents
        an empty cell or not.

        This method is intended for overriding.

        :rtype: bool
        :return:
            - C{True} if this cell is empty
            - C{False} otherwise
        """
        # Initialize the return value
        empty = True
        # Check whether relevant attributes are not empty
        if not CellInfo.is_empty_value(self.__rssi) \
                and not CellInfo.is_empty_value(self.__network_type):
            # We do not consider the other attributes as relevant
            # (for which sometimes Android leaves the default value).
            empty = False
        # Return the value
        return empty

    @classmethod
    def from_dict(cls, values):
        """
        Returns an new instance built from the given values.

        :param values: the values to use in order to build the
            C{GsmCellInfo} instance
        :type values: dict

        :rtype: GsmCellInfo
        :return: a GsmCellInfo instance.
        """
        # Initialize the return value
        instance = NeighboringCellInfo()
        # Consider whether we have to create a GSM cell info
        # or something else.
        if NeighboringCellInfo.KEY_LAC in values and \
            NeighboringCellInfo.KEY_RSSI in values and \
            NeighboringCellInfo.KEY_NETWORK_TYPE in values and \
                NeighboringCellInfo.KEY_CELL_ID in values:
            instance.__lac = values[NeighboringCellInfo.KEY_LAC]
            instance.__cid = values[NeighboringCellInfo.KEY_CELL_ID]
            instance.__network_type = values[NeighboringCellInfo.KEY_NETWORK_TYPE]
            instance.__rssi = values[NeighboringCellInfo.KEY_RSSI]
        # Return the instance
        return instance

    def __str__(self):
        """
        Returns a string representation of this object.

        :rtype: str
        :return: the string representation of this object
        """
        string_representation = "%s[lac:%s, cid: %s, network_type: %s, rssi: %s]" % (
            str(self.__class__),
            str(self.__lac),
            str(self.__cid),
            str(self.__network_type),
            str(self.__rssi))
        return string_representation


class GsmCellInfo(CellInfo):

    """
    This class represents a I{GSM} cell info.
    """

    KEY_LAC = "lac"
    """
    The key corresponding to the I{LAC} information.
    """

    KEY_CELL_ID = "cid"
    """
    The key corresponding to the I{cell id} information.
    """

    KEY_PSC = "psc"
    """
    The key corresponding to the I{Primary Scrambling Code} information.
    """

    def __init__(self):
        """
        Initializes this instance.
        """
        CellInfo.__init__(self)
        self.__cid = None
        self.__lac = None
        self.__psc = None

    def is_empty(self):
        """
        Returns a boolean indicating whether this instance represents
        an empty cell or not.

        This method is intended for overriding.

        :rtype: bool
        :return:
            - C{True} if this cell is empty
            - C{False} otherwise
        """
        # Initialize the return value
        empty = True
        # Check whether relevant attributes are not empty
        if not CellInfo.is_empty_value(self.__cid) and \
                not CellInfo.is_empty_value(self.__lac):
            # We do not consider the psc attribute as relevant
            # (for which sometimes Android leaves the default value).
            empty = False
        # Return the value
        return empty

    @classmethod
    def from_dict(cls, values):
        """
        Returns an new instance built from the given values.

        :param values: the values to use in order to build the
            C{GsmCellInfo} instance
        :type values: dict

        :rtype: GsmCellInfo
        :return: a GsmCellInfo instance.
        """
        # Initialize the return value
        instance = GsmCellInfo()
        # Consider whether we have to create a GSM cell info
        # or something else.
        if GsmCellInfo.KEY_LAC in values and \
            GsmCellInfo.KEY_PSC in values and \
                GsmCellInfo.KEY_CELL_ID in values:
            instance.__lac = values[GsmCellInfo.KEY_LAC]
            instance.__cid = values[GsmCellInfo.KEY_CELL_ID]
            instance.__psc = values[GsmCellInfo.KEY_PSC]
        # Return the instance
        return instance

    def __str__(self):
        """
        Returns a string representation of this object.

        :rtype: str
        :return: the string representation of this object
        """
        string_representation = "%s[lac:%s, cid: %s, psc: %s]" % (
            str(self.__class__),
            str(self.__lac),
            str(self.__cid),
            str(self.__psc))
        return string_representation


class CdmaCellInfo(CellInfo):

    """
    This class represents a I{CDMA} cell info.
    """

    KEY_BASE_STATION_ID = "bsid"
    """
    The key corresponding to the I{Base Station ID} information.
    """

    KEY_LATITUDE = "lat"
    """
    The key corresponding to the I{latitude} information.
    """

    KEY_LONGITUDE = "long"
    """
    The key corresponding to the I{longitude} information.
    """

    KEY_SYSTEM_ID = "sid"
    """
    The key corresponding to the I{System ID} information.
    """

    KEY_NETWORK_ID = "nwid"
    """
    The key corresponding to the I{Network ID} information.
    """

    def __init__(self):
        """
        Initializes this instance.
        """
        CellInfo.__init__(self)
        self.__base_station_id = None
        self.__latitude = None
        self.__longitude = None
        self.__system_id = None
        self.__network_id = None

    def is_empty(self):
        """
        Returns a boolean indicating whether this instance represents
        an empty cell or not.

        This method is intended for overriding.

        :rtype: bool
        :return:
            - C{True} if this cell is empty
            - C{False} otherwise
        """
        # Initialize the return value
        empty = True
        # Check whether relevant attributes are not empty
        if not CellInfo.is_empty_value(self.__base_station_id) and \
                not CellInfo.is_empty_value(self.__latitude) and \
                not CellInfo.is_empty_value(self.__longitude):
            # We do not consider the other attributes as relevant
            # (for which sometimes Android leaves the default value).
            empty = False
        # Return the value
        return empty

    @classmethod
    def from_dict(cls, values):
        """
        Returns an new instance built from the given values.

        :param values: the values to use in order to build the
            C{CdmaCellInfo} instance
        :type values: dict

        :rtype: CdmaCellInfo
        :return: a CdmaCellInfo instance.
        """
        # Initialize the return value
        instance = CdmaCellInfo()
        # Consider whether we have to create a GSM cell info
        # or something else.
        if CdmaCellInfo.KEY_BASE_STATION_ID in values and \
            CdmaCellInfo.KEY_LATITUDE in values and \
            CdmaCellInfo.KEY_LONGITUDE in values and \
            CdmaCellInfo.KEY_SYSTEM_ID in values and \
                CdmaCellInfo.KEY_NETWORK_ID in values:
            instance.__base_station_id = values[CdmaCellInfo.KEY_BASE_STATION_ID]
            instance.__latitude = values[CdmaCellInfo.KEY_LATITUDE]
            instance.__longitude = values[CdmaCellInfo.KEY_LONGITUDE]
            instance.__system_id = values[CdmaCellInfo.KEY_SYSTEM_ID]
            instance.__network_id = values[CdmaCellInfo.KEY_NETWORK_ID]
        # Return the instance
        return instance

    def __str__(self):
        """
        Returns a string representation of this object.

        :rtype: str
        :return: the string representation of this object
        """
        string_representation = "%s[base_station_id:%s, lat: %s, long:%s, " \
            "system_id: %s, network_id:%s]" % (
                str(self.__class__),
                str(self.__base_station_id),
                str(self.__latitude),
                str(self.__longitude),
                str(self.__system_id),
                str(self.__network_id))
        return string_representation


class SipLogParser(object):
    """
    A Sip log parser that can be used to test IMS registration
    (especially in LIVE conditions).

    This class is minimal by design. It can be improved further
    to cover more use cases.
    """

    START_INCOMING = "<<<<<"
    """
    The tag that indicates the beginning of an incoming
    message from the network.
    """

    START_OUTGOING = ">>>>>"
    """
    The tag that indicates the beginning of an outgoing
    message sent to the network.
    """

    CSEQ_TAG = "CSeq"
    """
    The tag corresponding to a sequence in a log message.
    """

    def __init__(self, file_path, logger=None):
        """
        Initializes this instance.
        :param file_path: the path of the file to parse
        :type file_path: str

        :param logger: the logger instance to use.
            Defaults to None.
        """
        self.__file_path = file_path
        self.__messages = []
        self.__logger = logger

    def ignore(self, line):
        """
        Implements the behavior of this class when a line has
        to be ignored from parsing.

        :param line: the line to process
        :type line: str
        """
        if self.__logger:
            self.__logger.debug("Ignoring line: '%s'" % line)

    def process_line(self, line):
        """
        Processes the given line and update parsed data accordingly.

        :param line: the line to process
        :type line: str
        """
        # Ensure the line is valid
        if not line:
            return
        # If the line corresponds to a message header start a new element
        if line.startswith(SipLogParser.START_INCOMING) or \
                line.startswith(SipLogParser.START_OUTGOING):
            new_message = {"header": line, SipLogParser.CSEQ_TAG: None}
            self.__messages.append(new_message)
        # Otherwise update the last element in the list
        elif line.startswith(SipLogParser.CSEQ_TAG):
            if len(self.__messages):
                latest_message = self.__messages[-1]
                latest_message[SipLogParser.CSEQ_TAG] = line
        else:
            self.ignore(line)

    def run(self):
        """
        Executes this parser.
        """
        if not self.__file_path:
            return
        with open(self.__file_path) as sip_log_file:
            # We assume that the file is not too big otherwise
            # we need to change the file reading process
            for current_line in sip_log_file:
                current_line = current_line.rstrip("\n")
                if current_line != "":
                    self.process_line(current_line)

    def get_messages(self):
        """
        Returns all the messages that have been detected during parsing.

        Each message is represented by a dict instance.

        :return: the parsed messages
        :rtype: list
        """
        return self.__messages

    def filter_cseq(self, tag):
        """
        Filters the messages containing the given tag in their
        CSeq attribute value.

        Each message is represented by a dict instance.

        :param tag: the tag to look for
        :type tag: str

        :return: the list of all messages that match the given tag
        :rtype: list
        """
        # Ensure that the tag is valid
        if not tag:
            return self.__messages
        # Filter the message list
        filtered = []
        # Return empty result if no messages have been parsed
        if not self.__messages:
            return filtered
        # Interate on messages
        for message in self.__messages:
            # Skip null messages
            if not message:
                continue
            # Store the cseq messages for later:
            cseq_messages = message[SipLogParser.CSEQ_TAG]
            # Check the content of the cseq messages
            if not cseq_messages:
                continue
            # A message is a dict, check that the message
            # has the given key and the expected tag.
            if SipLogParser.CSEQ_TAG in message and \
                    tag in cseq_messages:
                # In which case we add it to the filtered message list
                filtered.append(message)
        # Return the result of the filter
        return filtered


class ImsRegistrationStatus(object):
    """
    A class that allows to work with IMS registration statuses
    including integer to string conversion.
    """

    REGISTRATION_STATES = {
        - 1: "INVALID_VALUE",
        0: "STATE_IN_SERVICE",
        1: "STATE_OUT_OF_SERVICE"}
    """
    A dictionary indicating the possible values for IMS
    registration statuses and their string descriptions.
    """

    def __init__(self, registration_status):
        """
        Initializes this instance.
        :param registration_status: the integer value of the
            registration status
        :type registration_status: int
        """
        # Check the input parameter
        if registration_status not in \
                ImsRegistrationStatus.REGISTRATION_STATES:
            registration_status = -1
        # Initialize the object's attributes
        self.__integer_value = registration_status
        self.__string_value = \
            ImsRegistrationStatus.REGISTRATION_STATES[registration_status]

    def as_int(self):
        """
        Returns the integer value of this IMS registration status instance.
        :return: this object's integer value.
        :rtype: int
        """
        return self.__integer_value

    def __str__(self):
        """
        Returns a string representation of this object.
        :return: the registration status as string.
        :rtype: str
        """
        return self.__string_value

    def as_str(self):
        """
        Returns a string representation of this object.
        :return: the registration status as string.
        :rtype: str
        """
        return str(self)

    def __eq__(self, other):
        """
        Redefines the equality operator.
        :return: True of both object are equals and False otherwise
        :rtype: bool
        """
        if other is None or not isinstance(other, ImsRegistrationStatus):
            return False
        return self.as_int() == other.as_int()

    def __ne__(self, other):
        """
        Redefines the inequality operator.
        :return: False of both object are equals and True otherwise
        :rtype: bool
        """
        return not self == other

    @classmethod
    def in_service(cls):
        """
        Returns an ImsRegistrationStatus instance corresponding
        to the state 'STATE_IN_SERVICE'.
        :return: the registration state 'IN_SERVICE'
        :rtype: ImsRegistrationStatus
        """
        return ImsRegistrationStatus(0)

    @classmethod
    def out_of_service(cls):
        """
        Returns an ImsRegistrationStatus instance corresponding
        to the state 'STATE_OUT_OF_SERVICE'.
        :return: the registration state 'STATE_OUT_OF_SERVICE'
        :rtype: ImsRegistrationStatus
        """
        return ImsRegistrationStatus(1)


class ImsConfigGenerator(object):
    """
    A class that is able to re-generate an IMS_Config.ini file
    based on input parameters described in YAML format.
    """

    IMS_CONFIG_PARAMETERS = {
        "e_Mmb_Configuration_IETFModeEnabled": 4,
        "e_Mmb_Configuration_AutoLogin": 69,
        "e_Mmb_Configuration_PDPAddress_Type": 248,
        "e_Mmb_Configuration_NwDedicatedSignallingBearer": 253}

    """
    A dictionary indicating the possible values for the
    IMS parameters for modem configuration
    """

    def __init__(self, parameters_file_name, input_file_name, configurations, logger=None):
        """
        Initializes this instance.
        """
        self.__parameters_file_name = parameters_file_name
        self.__input_file_name = input_file_name
        self.__logger = logger
        if isinstance(configurations, (list, tuple)):
            self.__configurations = configurations
        else:
            self.__configurations = (configurations,)
        self.__document = None
        self.__last_error = ""

    @property
    def configurations(self):
        """
        The list of configurations.
        """
        return self.__configurations

    @configurations.setter
    def configurations(self, new_configurations):
        """
        Sets the list of configurations.
        """
        if not isinstance(new_configurations, (list, tuple)):
            new_configurations = (new_configurations,)
        self.__configurations = new_configurations

    @property
    def logger(self):
        """
        The logger instance to use as a property.
        :rtype: logging.logger
        """
        if not self.__logger:
            import logging
            self.__logger = logging.getLogger(__name__)
            self.__logger.setLevel(logging.DEBUG)
            formatter = logging.Formatter('%(name)s - %(levelname)s - %(message)s')
            handler = logging.StreamHandler()
            handler.setFormatter(formatter)
            self.__logger.addHandler(handler)
        return self.__logger

    @property
    def last_error(self):
        """
        Returns this generator's last error as string.
        :rtype: str
        :return: the last error message
        """
        return self.__last_error

    @property
    def document(self):
        """
        This generator's configuration as dictionary.
        :rtype: dict
        """
        return self.__document

    def get_parameters(self):
        """
        Returns a dictionary of all the parameters to apply.
        :rtype: dict
        """
        parameters = {}
        for configuration_name in self.__configurations:
            if configuration_name in self.document:
                new_parameters = self.document[configuration_name]
                self.logger.debug("Adding elements: %s" % str(new_parameters))
                for parameter in new_parameters:
                    if parameter:
                        parameters.update(parameter)
            else:
                self.logger.warning("Skipping unknown configuration: %s" % str(configuration_name))
        return parameters

    def save_source(self):
        """
        Saves the input file under a different name and
        return the save operation status.
        :return: the save operation status (True|False)
        :rtype: bool
        """
        # Initialize some local variables
        backup_name = "%s.save" % self.__input_file_name
        save_status = False
        # Save the file
        try:
            shutil.copy(self.__input_file_name, backup_name)
            save_status = True
        except IOError as io_error:
            message = "Could not copy file: %s (%s)" % (
                self.__input_file_name,
                str(io_error))
            self.logger.error(message)
            save_status = False
            self.__last_error = message
        # Return the status
        return save_status

    def restore_source(self):
        """
        Restores the original input file and returns
        the restore operation status.
        :return: the restore operation status (True|False)
        :rtype: bool
        """
        # Initialize some local variables
        backup_name = "%s.save" % self.__input_file_name
        restore_status = False
        # Rename the saved input file with its original name
        try:
            shutil.move(backup_name, self.__input_file_name)
            restore_status = True
        except IOError as io_error:
            message = "Could not move file: %s (%s)" % (
                backup_name,
                str(io_error))
            self.logger.error(message)
            self.__last_error = message
        # Return the status
        return restore_status

    def load_parameters(self):
        """
        Loads the parameters for this generator and returns
        the load operation status.
        :return: the load operation status (True|False)
        :rtype: bool
        """
        # Initialize some local variables
        load_status = False
        # Parse the parameter file
        self.logger.debug("Parsing IMS generator configuration file: %s" % self.__parameters_file_name)
        try:
            with open(self.__parameters_file_name, 'r') as stream:
                self.__document = yaml.load(stream)
                load_status = True
        except IOError as io_error:
            message = "Could not load parameters: %s" % str(io_error)
            self.logger.error(message)
            load_status = False
            self.__last_error = message
        # Return the status
        return load_status

    def get_at_commands(self):
        """
        Returns the set of AT commands to send to the modem for
        this generator configurations.
        """
        # Initialize local variables
        load_status = False
        # Initialize a tuple with all AT commands
        at_commands = ()
        # Check the configuration list
        if not self.__configurations:
            self.logger.warning("No IMS configuration found, no AT commands to run.")
            return at_commands
        # Parse the input document
        try:
            with open(self.__parameters_file_name, 'r') as stream:
                self.__document = yaml.load(stream)
                load_status = True
        except IOError as io_error:
            message = "Could not load parameters: %s" % str(io_error)
            self.logger.error(message)
            load_status = False
            self.__last_error = message
            return load_status
        # Test the input document
        if not self.document:
            self.logger.warning("No input configuration was found.")
            return at_commands
        # Look for the requested configurations
        for configuration_name in self.__configurations:
            # If the configuration exists in the YAML document
            if configuration_name in self.document:
                # Retrieve the configuration content
                new_at_commands = self.document[configuration_name]
                self.logger.debug("Adding elements: %s" % str(at_commands))
                # Check the at command list
                if not new_at_commands:
                    continue
                # Add each command in the result tuple seperatedly
                for command in new_at_commands:
                    if command:
                        at_commands += (command,)
            # Ignore unknown configurations
            else:
                self.logger.warning("Skipping unknown configuration: %s" % str(configuration_name))
        # Return the tuple with all AT commands
        return at_commands


    def get_ims_config_parameters(self):
        """
        Returns the set of IMS parameters to be configured in modem side
        this generator configurations.

        :rtype: tuple, tuple
        :return: The keys and the values for the IMS parameters which needs to be set
        """

        # Initialize local variables
        load_status = False
        # Initialize the tuples with all the keys and values for the IMS parameters
        ims_params = ()
        params_values = ()
        # Check the configuration list
        if not self.__configurations:
            self.logger.warning("No IMS configuration found, no parameter to set on modem side.")
            return (ims_params, params_values)
        # Parse the input document
        try:
            with open(self.__parameters_file_name, 'r') as stream:
                self.__document = yaml.load(stream)
                self.logger.info("load the document successfully")
        except IOError as io_error:
            message = "Could not load parameters: %s" % str(io_error)
            self.logger.error(message)
            self.__last_error = message
            return (ims_params, params_values)
        # Test the input document
        if not self.document:
            self.logger.warning("No input configuration was found.")
            return (ims_params, params_values)
        # Look for the requested configurations
        for configuration_name in self.__configurations:
            # If the configuration exists in the YAML document
            if configuration_name in self.document:
                # Retrieve the configuration content
                new_ims_config_list = self.document[configuration_name]
                self.logger.debug("Adding elements: %s" % str(new_ims_config_list))
                # Check the at command list
                if not new_ims_config_list:
                    continue
                # Add each command in the result tuple
                for config, value in new_ims_config_list.iteritems():
                    if config not in self.IMS_CONFIG_PARAMETERS:
                        msg = "Parameter: " + config + " is not in the dictionary defined"
                        raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
                    else:
                        ims_params += (self.IMS_CONFIG_PARAMETERS[config],)
                        params_values += (value, )

            # Ignore unknown configurations
            else:
                self.logger.warning("Skipping unknown configuration: %s" % str(configuration_name))

        # Return the IMS key parameters and the values
        return (ims_params, params_values)


    def replace_tokens(self, params_values, template_values):
        """
        Replaces all the tokens that can be found in the given set
        of AT commands/parameters with the provided template values.

        Token syntax is %%<token_name>%%.

        :param params_values: the set of AT commands/values for the parameters
            that we want to update.
        :type params_values: tuple

        :param template_values: the dictionary containing the token
            names and their corresponding replace value.
        :type template_values: dict

        :rtype: tuple
        :return: the updated set of AT commands/Values for the parameters
        """

        # Update the dictionary for the tokens
        tokens = self.__update_token_dict(template_values)

        self.__logger.debug("Original set = " + str(params_values))
        # Initialize the result tuple
        updated_param_values = ()
        # Iterate on the provided AT command/Values set
        for v_param in params_values:
            # Replace any occurrence of any token with its corresponding value.
            updated_value = str(v_param)
            for token in tokens:
                value = tokens[token]
                updated_value = updated_value.replace(token, value)
            # Add the command to the final result
            updated_param_values += (updated_value,)

        self.__logger.debug("Result after replacing the tokens: " + str(updated_param_values))

        # Return the result tuple
        return updated_param_values

    def __update_token_dict(self, template_values):
        """
        Updates the dictionary for the token used

        :param template_values: the dictionary containing the token
            names and their corresponding replace value.
        :type template_values: dict

        :rtype: dict
        :return: the updated dictionary - Token syntax is %%<token_name>%%.
        """
        # Create a new dictionary with the actual token names/values
        tokens = {}
        # Log the input values
        self.__logger.debug("Processing values: %s" % str(template_values))
        for key in template_values:
            # Log the key that we are processing
            self.__logger.debug("Processing key: %s" % str(key))
            # Build the token value without using the % operator
            token = "%%" + str(key) + "%%"
            # Retrieve the value
            value = template_values[key]
            # Update the tokens dictionary
            tokens[token] = value

        return tokens

    def revert_config(self, serial_handler):
        """
        Reverts the modem configuration to its default state.

        This method will apply the set of AT commands linked
        to the 'reset' configuration in the YAML file.

        :type serial_handler: SerialHandler
        :param serial_handler: the SerialHandler instance that will
            actually send AT commands to the modem

        """
        # Initialize some local variables
        configuration_status = False
        # Save the initial configuration for later
        initial_configurations = self.configurations
        # Force the new configuration
        self.configurations = ("reset", "flush")
        # Retrieve and update the AT commands
        at_commands = self.get_at_commands()
        # Restore the initial configuration
        self.configurations = initial_configurations
        # Iterate on AT commands
        for command in at_commands:
            # Execute the current AT commands
            self.logger.debug("Sending AT command: %s " % str(command))
            (status, message) = serial_handler.write_and_analyse(command)
            # Check the command execution status
            if status == Global.SUCCESS:
                configuration_status = True
            else:
                self.logger.error("Error on AT command execution: %s " % str(message))
                self.__last_error = message
                configuration_status = False
                break
        # Return the generation status
        return configuration_status

    def generate_config(self):
        """
        Generates the IMS_Config.ini file and returns a boolean status
        for the generation.
        :return: the generation status (True|False)
        :rtype: bool
        """
        # Initialize local variables
        tmp_file_name = "%s.tmp" % self.__input_file_name
        generation_status = False
        parameters = self.get_parameters()
        # Save the input file under a different name
        try:
            shutil.move(self.__input_file_name, tmp_file_name)
        except IOError as io_error:
            message = "Could not move file: %s (%s)" % (
                self.__input_file_name,
                str(io_error))
            self.logger.error(message)
            self.__last_error = message
            # Return the verdict of the generation
            return generation_status
        # Avoid nested contexts
        try:
            # Open input and output files
            with open(self.__input_file_name, 'w') as output_file:
                with open(tmp_file_name, 'r') as input_file:
                    for line in input_file:
                        # Check if we have a matching parameter
                        for parameter in parameters:
                            if parameter in line:
                                line = self.__process_line(line, parameters)
                        # Write the line in the destination
                        output_file.write(line)
            # If everything went fine, update the generation status
            generation_status = True
        except IOError as io_error:
            message = "Could not read or write file: %s" % str(io_error)
            self.logger.error(message)
            self.__last_error = message
            generation_status = False
        finally:
            os.unlink(tmp_file_name)
        # Return the status
        return generation_status

    def __process_line(self, line, parameters):
        """
        Updates the given line with the given parameters if any match is found.
        Return the resulting line.
        :return: the updated line
        :rtype: str
        """
        # Check if we have a matching parameter
        for parameter in parameters:
            if parameter in line:
                replace_value = parameters[parameter]
                # Update the line
                replace = replace_value["replace"]
                value = replace_value["value"]
                self.logger.debug(
                    "Applying parameter %s ('%s' => '%s') to line: '%s'" % (
                        parameter,
                        replace,
                        value, line))
                line = line.replace(replace, value)
        # Return the result
        return line


def check_dut_registration_before_timeout(ns=None,
                                          networking_api=None,
                                          logger=None,
                                          dut_imsi=None,
                                          timeout=0,
                                          flightmode_cycle=True,
                                          blocking=True):
    """
    Check whether or not the DUT is registering onto network simulator.
    Optionally, it can flight mode cycle the modem or skip imsi match check
    :type ns: object instance
    :param ns: Network Simulator object instance
    :type networking_api: object instance
    :param networking_api: Networking UECmd object instance
    :type logger: object instance
    :param logger: Logger instance
    :type dut_imsi: str
    :param dut_imsi: imsi of the DUT
    :type timeout: integer
    :param timeout: maximum authorized time for DUT registration
    :type flightmode_cycle: Boolean
    :param flightmode_cycle: Described whether a flight mode cycle is required to force registration of modem
    :rtype: bool
    :return: if the DUT is registered
    """

    # Turn Off then On the modem by using Flight mode cycle if required
    if ((ns is None) or
       (networking_api is None) or
       (logger is None)):
        msg = "Cannot act on flight mode or Network simulator as no interface provided!"
        raise AcsConfigException(AcsConfigException.PROHIBITIVE_BEHAVIOR, msg)

    if flightmode_cycle:
        networking_api.set_flight_mode("on")
        time.sleep(5)
        networking_api.set_flight_mode("off")

    # No IMSI check, nothing to be done
    if dut_imsi is None:
        return True

    logger.info("Network simulator: Check DUT registration before %d seconds", int(timeout))
    elapsed_time = 0
    is_registered = False

    while (elapsed_time < timeout) and (is_registered is False):
        is_registered = ns.is_dut_registered(dut_imsi)
        time.sleep(1)
        elapsed_time += 1

    if is_registered:  # Registration success
        logger.info("Network simulator: Registration success!")
        return True
    else:  # Registration failure
        msg = "Network simulator: Registration failure after %d seconds!" % elapsed_time
        if blocking:
            logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.TIMEOUT_REACHED, msg)
        else:
            logger.info(msg)
        return False


def configure_ims_for_8960(ns_data, networking_api, logger):
    """
    Configure 8960 to reject IMS PDP context
    :type ns: object instance
    :param ns: Network Simulator data interface object instance
    :type networking_api: object instance
    :param networking_api: Networking UECmd object instance
    :type logger: object instance
    :param logger: Logger instance
    :attention: This function is only for 3G 8960
    """
    logger.info("DUT support IMS, configure 8960 to support it ")
    # Set IMS APN on phone

    logger.info("Setting APN for IMS on protocol IPV4 ")
    # APN for IMS
    networking_api.set_apn("ims", "ims", None, None, "IP", None, "ims", False, False)

    ns_data.configure_ims_reject("ims")


def check_dut_data_connection_state_before_timeout(state,
                                                   ns=None,
                                                   networking_api=None,
                                                   logger=None,
                                                   timeout=0,
                                                   flightmode_cycle=True,
                                                   blocking=True,
                                                   cell_id=None,
                                                   mimo=None):
    """
    Check whether or not the DUT is data connected onto network simulator.
    Optionally, it can flight mode cycle the modem
    :type state: str
    :param state: the expected state. Possible values:
            - "ATTACHED"
            - "PDP_ACTIVE"
            - "TRANSFERRING"
            - "SUSPENDED"
    :type ns: object instance
    :param ns: Network Simulator object instance
    :type networking_api: object instance
    :param networking_api: Networking UECmd object instance
    :type logger: object instance
    :param logger: Logger instance
    :type timeout: integer
    :param timeout: maximum authorized time for DUT registration
    :type flightmode_cycle: Boolean
    :param flightmode_cycle: Described whether a flight mode cycle is required to force registration of modem
    :type blocking: boolean
    :param blocking: boolean to know if the function raises an error
            or simply return true or false if the status is reached or not
    :type cell_id : str
    :param cell_id: cell used for the test. Possible values:
            - "A"
            - "B"
    :attention: This parameter is only used in 4G (LTE)
    :type mimo: boolean
    : param mimo: If mimo is not used, only RFO1 / MOD1 are on
                  If mimo is used, RFO1 / MOD1 and RF02 / MOD2 are on
    """

    # Turn Off then On the modem by using Flight mode cycle if required
    if ((ns is None) or
       (networking_api is None) or
       (logger is None)):
        msg = "Cannot act on flight mode or Network simulator as no interface provided!"
        raise AcsConfigException(AcsConfigException.PROHIBITIVE_BEHAVIOR, msg)

    if flightmode_cycle:
        networking_api.set_flight_mode("on")
        time.sleep(5)
        networking_api.set_flight_mode("off")

    ns.get_data().check_data_connection_state(state,
                                              timeout,
                                              blocking,
                                              cell_id)


def get_dict_key_from_value(dictio, value):
    """
    Returns the dictionnary key corresponding to a given value
    :type dictio: dict
    :param dictio: the dictionnary to retrieve the key

    :type value: object instance
    :param value: value from which we will get the key

    :rtype: object
    :return: the corresponding key

    """
    return dictio.keys()[dictio.values().index(value)]


def check_mnc_mcc(
        mnc,
        mcc,
        expected_mnc,
        expected_mcc,
        sim_name=None,
        logger=None):
    """
    Checks that the provided I{MCC} and I{MNC} values match the
    expected given values.

    The optional parameter C{sim_name} should only be used for
    I{DSDS} phones. It is needed to indicate the I{SIM} name
    on which the test is done in case of an error.

    :type mnc: str
    :param mnc: the I{MNC} value to check.
    :param mcc: str
    :param mcc: the I{MCC} value to check.

    :type expected_mnc: str
    :param expected_mnc: the expected I{MNC} value.
    :type expected_mcc: str
    :param expected_mcc: the expected I{MCC} value.

    :type sim_name: str
    :param sim_name: [optional] the name of the SIM on which
        the test is done (for DSDS phones only).

    :type logger: object
    :param logger: [optional] Logger instance

    :raise DeviceException: if either C{mnc} or C{mcc} do not have
        the expected value.
    """
    # If MCC or MNC is empty, there is an issue with registration
    # status as check_cdk_state_bfor_timeout() was successfull
    if (mcc == "") or (mnc == ""):
        message = "MCC and/or MNC are not reported " \
            "despite phone is registered"
        if sim_name is not None:
            message += " [%s]." % str(sim_name)
        else:
            message += "."
        if logger is not None:
            logger.error(message)
        raise DeviceException(DeviceException.PROHIBITIVE_BEHAVIOR, message)
    else:
        # Compare user values of MCC and MNC with network values
        # Raise an error message if MCC and MNC values from
        # operator are not equal to Networks' values
        if (mcc != expected_mcc) or (mnc != expected_mnc):

            message = "Expected MCC (%s) - MNC (%s) do " \
                "not match network returned MCC (%s) - MNC (%s)" % (
                    expected_mcc,
                    expected_mnc,
                    mcc,
                    mnc)
            if sim_name is not None:
                message += " [%s]." % str(sim_name)
            else:
                message += "."

            if logger is not None:
                logger.error(message)
            raise DeviceException(DeviceException.PROHIBITIVE_BEHAVIOR, message)
        else:
            # need to name the mcc and mnc as int not str
            message = "Expected MCC (%s) - MNC (%s) " \
                "match network returned MCC (%s) - MNC (%s)" % (
                    expected_mcc,
                    expected_mnc,
                    mcc,
                    mnc)
            if sim_name is not None:
                message += " [%s]." % str(sim_name)
            else:
                message += "."

            if logger is not None:
                logger.info(message)


def check_mnc_mcc_range(mnc, mcc, logger=None):
    """
    Checks whether the given C{mnc} and C{mcc} values
    are in a valid range.
    :type mnc: str (or int)
    :param mnc: the I{MNC} value to check.
    :type mcc: str (or int)
    :param mcc: the I{MCC} value to check.
    :type logger: object
    :param logger: [optional] Logger instance
    :raise AcsConfigException: if either C{mnc} or C{mcc} are not valid.
    """
    # Try a cast MNC to an int value
    if isinstance(mnc, str) and mnc.isdigit():
        mnc = int(mnc)
    # Check the type
    if not isinstance(mnc, int):
        message = "Invalid type '%s' for MNC (expected 'str' or 'int')" % \
            str(type(mcc))
        if logger is not None:
            logger.error(message)
        raise AcsConfigException(AcsConfigException.PROHIBITIVE_BEHAVIOR, message)

    # Try a cast MCC to an int value
    if isinstance(mcc, str) and mcc.isdigit():
        mcc = int(mcc)
    # Check the type
    if not isinstance(mcc, int):
        message = "Invalid type '%s' for MCC (expected 'str' or 'int')" % \
            str(type(mcc))
        if logger is not None:
            logger.error(message)
        raise AcsConfigException(AcsConfigException.PROHIBITIVE_BEHAVIOR, message)
    # Check the ranges for MNC / MCC
    if not ((0 <= mcc <= 999) and (0 <= mnc <= 999)):
        message = "Expected MCC (%s) - MNC (%s) have no valid values" % \
            (str(mcc), str(mnc))
        if logger is not None:
            logger.error(message)
        raise AcsConfigException(AcsConfigException.PROHIBITIVE_BEHAVIOR, message)


def setup_cell(ns_number,
               ns_model,
               ns_cell_tech,
               ns_cell_band,
               ns_cell_rel,
               logger=None):
    """
    Configure equipment to specific cells via .xml files
    in order to reach specific throughput
    :type ns_number: str
    :param ns_number: Number of the Network Simulator
    :type ns: str
    :param ns: Instance of the Network Simulator
    :type ns_model: str
    :param ns_model: Model of the Network Simulator
    :type ns_cell_tech: str
    :param ns_cell_tech: Cell Technology of the Network Simulator
    :type ns_cell_band: str
    :param ns_cell_band: Cell Band of the Network Simulator
    :type ns_cell_rel: int
    :param ns_cell_rel: Network Cell Release
    :type logger: object
    :param logger: [optional] Logger instance
    """
    if ns_cell_band is not None:
        # Configure active cell
        # set COMMON parameters
        if ns_model == "AGILENT_8960":
            if ns_cell_tech == "2G":
                EM().configure_equipments("CellConfigurationNS%s" % ns_number,
                                          {"type": "COMMON_2G"})
            elif ns_cell_tech == "3G":
                EM().configure_equipments("CellConfigurationNS%s" % ns_number,
                                          {"type": "COMMON_3G"})
            # set specific cell parameters for cell band
            EM().configure_equipments("CellConfigurationNS%s" % ns_number,
                                      {"type": ns_cell_band})
            if ns_cell_rel == 8:
                logger.info("Applying RAN Release 8 settings")
                # set specific cell parameters for release 8
                EM().configure_equipments("CellConfigurationNS%s" % ns_number,
                                          {"type": "%s_R%s" % (ns_cell_band, ns_cell_rel)})

        elif ns_model == "RS_CMW500":
            if ns_cell_tech == "4G":
                EM().configure_equipments("CMW500_LTE_NS%d" % ns_number,
                                          {"type": "COMMON_4G"})
                # set specific cell parameters
                EM().configure_equipments("CMW500_LTE_NS%d" % ns_number,
                                          {"type": ns_cell_band})

        msg = "ACTIVE CELL : The band cell configuration has been set to %s for %s Technology." \
            % (ns_cell_band,
               ns_cell_tech)

        logger.info(msg)

    else:
        msg = "Cannot act on Network Simulator Configuration as no Cell Band provided!"
        raise AcsConfigException(AcsConfigException.PROHIBITIVE_BEHAVIOR, msg)


def setup_cell_lte(ns,
                   mccode,
                   mncode,
                   ip_dut,
                   signal_mode,
                   cell_id,
                   physical_cell_id,
                   mimo,
                   cell_power_rf1,
                   cell_power_rf2,
                   scenario_path,
                   cell_band,
                   lte_dl_earfcn,
                   apn):
    """
    Set the Cell LTE

    :type ns: EquipmentManager
    :param ns: Network Simulator
    (Object of EquipmentManager Class)
    :type mccode: str
    :param mccode: The Mobile Country Code to set.
    An integer from 0 to 999.
    :type mncode: str
    :param mncode: The Mobile Network Code to set.
    An integer from 0 to 99 for other bands.
    :type ip_dut: str
    :param ip_dut: The DUT IP address to set.
    :type signal_mode: str
    :param signal_mode: Operation mode to set BSE or SA
    :type cell_id: str
    :param cell_id:  str ("A" or "B" are expected values)
    :type physical_cell_id: str
    :param physical_cell_id: Cell ID to be set
    :type mimo: boolean
    : param mimo: If mimo is not used, only RFO1 / MOD1 are on
    If mimo is used, RFO1 / MOD1 and RF02 / MOD2 are on
    :type cell_power_rf1: double
    :param cell_power_rf1: Cell Power to set on RF1
    :type cell_power_rf2: double
    :param cell_power_rf2: Cell Power to set on RF2
    :type scenario_path: str
    :param scenario_path: The configuration file to load with his path
    :type cell_band: int
    :param cell_band: Cell Band to be set
    :type lte_dl_earfcn: str
    :param lte_dl_earfcn: Downlink Channel to be set
    :type apn: str
    :param apn: Access point name to be set on the Network simulator
    """
    # Set cell off
    ns.get_cell_4g().set_cell_off()

    # Load the scenario for the test
    ns.load_configuration_file(scenario_path)

    # apply configuration from benchconfig
    ns.apply_bench_config(None)

    # For RS CMW500, set_dut_ip_address is already called in apply_bench_config
    if "RS_CMW500" not in ns.get_model():
        # Set the DUT IP address
        ns.get_cell_4g().get_data().set_dut_ip_address(ip_dut)

    # Set the Signal Mode
    ns.get_cell_4g().set_signal_mode(signal_mode)

    # Enable the Cell used for the test, Cell-A or Cell-B
    ns.get_cell_4g().set_cell_setup(cell_id)

    # Set the Physical Cell ID
    ns.get_cell_4g().set_cell_id(physical_cell_id)

    # Configure Basic cell parameters
    ns.get_cell_4g().configure_basic_lte_cell_parameters(cell_band,
                                                         lte_dl_earfcn,
                                                         mimo,
                                                         cell_power_rf1,
                                                         cell_power_rf2,
                                                         mccode,
                                                         mncode)
    # Set the APN
    ns.get_cell_4g().get_data().set_apn(apn)
    # Start the scenario
    ns.start_scenario()


def check_rat_and_data_connection_state(ns_data,
                                        expected_data_connection_state,
                                        modem_api,
                                        timeout):
    """
    Check RAT and data connection state

    :type ns_data: str
    :param ns_data: Network Simulator Data Api
    :type expected_data_connection_state: str
    :param expected_data_connection_state:
    Expected data connection state ("PDP_ACTIVE" as example)
    :type modem_api: str
    :param modem_api: DUT Modem Api
    :type timeout: integer
    :param timeout: maximum authorized time for DUT registration
    """
    # Check Data Connection State on cell => Expected Data Connection State before timeout
    ns_data.check_data_connection_state(expected_data_connection_state,
                                        timeout,
                                        blocking=False)

    # Check that DUT is registered on the good RAT
    modem_api.check_network_type_before_timeout(ns_data.get_network_type(),
                                                timeout)


def check_and_set_minimal_cell_power(ns_cell, modem_api, dut_config, initial_power):
    """
    Check if DUT see serving cell in good signal condition
    If not update cell power accordingly with bench config phone section parameter ServingCellDesiredPower

    :type ns_cell: str
    :param ns_cell: Network Simulator Cell Api
    :type modem_api: str
    :param modem_api: DUT Modem Api
    :type dut_config: dict
    :param dut_config: DUT parameters
    :type initial_power: integer
    :param initial_power: power value set in simulator cell
    """
    # get serving cell power
    if "ServingCellDesiredPower" in dut_config:
        try:
            cell_power = modem_api.get_cell_power()

            desired_power = int(dut_config.get("ServingCellDesiredPower"))
            delta_power = cell_power - desired_power
            if delta_power < 0:
                LOGGER_TEST_SCRIPT.info("Signal is too weak on DUT, Increase equipment cell power to %.2f" % (float(initial_power) - float(delta_power)))
                LOGGER_TEST_SCRIPT.info("Measured cell power %.2f - Initial cell power %.2f" % (float(cell_power), float(initial_power)))
                ns_cell.set_cell_power(float(initial_power) - float(delta_power))
                try:
                    ns_cell.set_secondary_carrier_cell_power(float(initial_power) - float(delta_power))
                except:
                    pass
        except:
            LOGGER_TEST_SCRIPT.info("Cannot check if DUT see serving cell correctly")
