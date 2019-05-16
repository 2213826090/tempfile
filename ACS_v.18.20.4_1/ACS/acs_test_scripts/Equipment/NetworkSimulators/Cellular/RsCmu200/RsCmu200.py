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
:summary: implementation of RS CMU200 cellular network simulator
:since: 08/03/2011
:author: ymorel
"""

import weakref
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.IEquipment import DllLoader
from Lib.PyVisa import visa as Visa
from Lib.PyVisa import visa_exceptions as VisaException
from acs_test_scripts.Equipment.VisaEquipment import VisaEquipment
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Interface.ICellNetSim import ICellNetSim
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmu200.Tech2G.Cell2G import Cell2G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmu200.Tech3G.Cell3G import Cell3G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmu200.Wrapper import WBase as W


class RsCmu200(ICellNetSim, DllLoader):

    """
    Implementation of RS CMU200 equipment
    """
    GPIB_SUCCESS = '0,"No error"'

    # Default Board IDentifier
    DEFAULT_BOARD_ID = 0

    # Default GPIB Address
    DEFAULT_GPIB_ADDRESS = 20

    # Indice of BASE handle Identifier
    HANDLE_BASE_ID = 0

    # Indice of GSM 400 handle Identifier
    HANDLE_GSM_400_ID = 1

    # Indice of GSM 850 handle Identifier
    HANDLE_GSM_850_ID = 2

    # Indice of GSM 900 handle Identifier
    HANDLE_GSM_900_ID = 3

    # Indice of GSM 1800 handle Identifier
    HANDLE_GSM_1800_ID = 4

    # Indice of GSM 1900 handle Identifier
    HANDLE_GSM_1900_ID = 5

    # Indice of WCDMA handle Identifier
    HANDLE_WCDMA_ID = 6

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        :type name: str
        :param name: the bench configuration name of the equipment
        :type model: str
        :param model: the model of the equipment
        :type eqt_params: dict
        :param eqt_params: the dictionary containing equipment parameters
        :type bench_params: dict
        :param bench_params: the dictionary containing equipment bench parameters
        """
        ICellNetSim.__init__(self)
        # Construct DllLoader object
        DllLoader.__init__(self, name, model, eqt_params)
        # Initialize attributes
        self.__name = name
        self.__model = model
        self.__eqt_params = eqt_params
        self.__bench_params = bench_params
        self.__bench_params = bench_params
        self.__cell2g = None
        self.__cell3g = None
        self.__handle = None
        # pyvisa interface connection
        self.__visa = None
        # Initialize features
        self.__init_features()

    def __del__(self):
        """
        Destructor
        """
        self.release()
        del self.__cell2g
        del self.__cell3g

    def __init_features(self):
        """
        Initialize features according to equipment catalog parameters
        """
        features = self.get_eqt_dict()[self.get_model()]["Features"]
        features_name = features.keys()
        if "2G" in features_name:
            if features["2G"] == "enable":
                self.__cell2g = Cell2G(weakref.proxy(self))
        if "3G" in features_name:
            if features["3G"] == "enable":
                self.__cell3g = Cell3G(weakref.proxy(self))

    def __error_check(self, err, msg):
        """
        Error checking and warning reporting
        :raise TestEquipmentException: if err < 0
        """
        if err < 0:
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, msg)
        elif err > 0:
            self.get_logger().warning(msg)

    def __connect_via_GPIB(self):
        """
        Connect to equipment via GPIB
        """
        board_id = int(self.__bench_params.get_param_value("GPIBBoardId"))
        gpib_addr = int(self.__bench_params.get_param_value("GPIBAddress"))
        (err, handle, msg) = W.Connect(self, board_id, gpib_addr)
        self.__error_check(err, msg)
        # Update handle value
        self._set_handle(handle)

    def get_handle(self):
        """
        Gets the connection handle
        :rtype: unsigned long
        :return: the handle of connection with the equipment, None if no
        equipment is connected
        """
        return self.__handle

    def _set_handle(self, handle):
        """
        Sets the connection handle
        :type handle: unsigned integer
        :param handle: the new connection handle
        """
        self.__handle = handle

    def init(self, additional_handle=None):
        """
        Initializes the equipment
        """

        if additional_handle is not None:
            if self.__visa.get_handle() is not None:
                return

        else:
            self.get_logger().info("Initialization")
            if self.get_handle() is not None:
                return

            # Load the equipment driver
            self.load_driver()

        # Get transport mode and try to connect to equipment
        transport = str(self.__bench_params.get_param_value("Transport"))

        # Check if transport is supported
        transport_catalog = self.get_eqt_dict()[self.get_model()]["Transports"]
        if transport not in transport_catalog:
            msg = "Unsupported transport %s" % (str(transport))
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.TRANSPORT_ERROR, msg)

        # Check if selected transport is enabled
        if transport_catalog[transport] == "enable":
            if additional_handle is not None:
                # Trying to connect to the equipment using VisaEquipment
                self.__visa_connect_via_GPIB(additional_handle)
            else:
                # Trying to connect to the equipment using dll
                connect = getattr(self, "_" + self.__class__.__name__ +
                                  "__connect_via_" + transport)
                connect()

        else:
            msg = "%s transport is disabled" % (str(transport))
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.TRANSPORT_ERROR, msg)

    def release(self):
        """
        Release the equipment and all associated resources
        """
        if self.get_handle() is not None:
            (err, msg) = W.Disconnect(self)
            self.unload_driver()
            self.__error_check(err, msg)
            # Update handle value
            self._set_handle(None)

    def get_cell_2g(self):
        """
        Access to 2G cellular interface.
        :rtype: ICell2G
        :return: the 2G cellular object.
        """
        if self.__cell2g is not None:
            return self.__cell2g
        msg = "2G is not available"
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, msg)

    def get_cell_3g(self):
        """
        Access to 3G cellular interface.
        :rtype: ICell3G
        :return: the 3G cellular object.
        """
        if self.__cell3g is not None:
            return self.__cell3g
        msg = "3G is not available"
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, msg)

    def perform_full_preset(self):
        """
        Resets all equipment parameters to defaults.
        """
        (err, msg) = W.PerformFullPreset(self)
        self.__error_check(err, msg)

    def load_configuration_file(self, source, filename):  # pylint: disable=W0221
        """
        Loads equipment parameters from a file.
        :type source: str
        :param source: the source from which to load the configuration.
        Possible values:
            - "INTERNAL"
            - "EXTERNAL"
        :type filename: str
        :param filename: the configuration file to load
        """
        (err, msg) = W.LoadConfigurationFile(self, source, filename)
        self.__error_check(err, msg)

    def switch_app_format(self, app_format):
        """
        Switches application format of the network simulator.
        :type app_format: str
        :param app_format: the desired application format. Possible values:
            - "1xEV-DO"
            - "AMPS/136"
            - "GSM/GPRS"
            - "IS-2000/IS-95/AMPS"
            - "IS-856"
            - "WCDMA"
        """
        (err, msg) = W.SetAppFormat(self, app_format)
        self.__error_check(err, msg)

    def configure_amplitude_offset_table(self, frequency_list, offset_list):
        """
        Configures the amplitude offset table to compensate cable loss.
        :type frequency_list: str
        :param frequency_list: the frequency list.
        :type offset_list: str
        :param offset_list: the offset list corresponding to the frequency
        listed above.
        """
        # TO DO: to complete
        pass

    def set_ul_dl_attenuation(self, ul_att, dl_att):
        """
        Sets uplink and downlink attenuations
        :type ul_att: double
        :param ul_att: uplink attenuation to set:
            - 2G: -50 dB to +90 dB
            - 3G: -50 dB to +50 dB
        :type dl_att: double
        :param dl_att: downlink attenuation to set:
            - 2G: -50 dB to +90 dB
            - 3G: -50 dB to +50 dB
        """
        self.get_logger().info(
            "Setting UL and DL attenuation to %f and %f...",
            ul_att,
            dl_att)
        (err, msg) = W.SetUlDlAttenuation(self, ul_att, dl_att)
        self.__error_check(err, msg)

    def _get_visa_equipment(self, handle_id):
        """
        Get agilent equipment with pyvisa interface
        It is used only for query command
        """

        # release DllLoader object
        self.release()

        self.get_logger().debug("Init RS visa equipment")

        # Init agilent equipment with pyvisa interface
        self.__visa = VisaEquipment(self.__name,
                                    self.__model,
                                    self.__eqt_params,
                                    self.__bench_params)

        self.init(handle_id)

    def send_command(self, command, instrument):
        """
        sends a GPIB Command.

        :type command: str
        :param command: String representing a well formed GPIB command.

        :return: None
        """

        # Get CMU200 equipment with pyvisa interface
        if instrument in "GSM/GPRS":
            # The band GSM900 is used by default when connecting to the 2G cell via CMU200 dll
            handle_id = RsCmu200.HANDLE_GSM_900_ID
        elif instrument in "WCDMA":
            handle_id = RsCmu200.HANDLE_WCDMA_ID
        else:
            msg = "Wrong CMU200 Instrument name"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        self._get_visa_equipment(handle_id)

        # Send the command
        self.get_logger().info("Send GPIB command : %s" % command)
        response = self.__visa.write(command)

        # Check errors in the System Error buffer
        return_msg = self.__visa.query("SYST:ERR?")

        if return_msg != RsCmu200.GPIB_SUCCESS:
            self.get_logger().error(return_msg)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, return_msg)

        # Release CMU200 equipment with pyvisa interface
        self.__visa.disconnect()
        self.__visa = None

        # reinit DllLoader object
        self.init()

        self.switch_app_format(instrument)

        if instrument in "GSM/GPRS":
            # Set cell off
            self.get_cell_2g().set_cell_off()
            # Set voice call output
            self.get_cell_2g().get_voice_call().set_speech_configuration("SPEECH_OUTPUT")
            # Set cell on
            self.get_cell_2g().set_cell_on()
        elif instrument in "WCDMA":
            # Set cell off
            self.get_cell_3g().set_cell_off()
            # Set voice call output
            self.get_cell_3g().get_voice_call().set_speech_configuration("SPEECH_OUTPUT")
            # Set cell on
            self.get_cell_3g().set_cell_on()

        return response

    def query_command(self, command, instrument):
        """
        query a GPIB Command.

        :type command: str
        :param command: String representing a well formed GPIB command.

        :return: The response from CMU200 for the command
        """

        # Get CMU200 equipment with pyvisa interface
        if instrument in "GSM/GPRS":
            handle_id = RsCmu200.HANDLE_GSM_900_ID
        elif instrument in "WCDMA":
            handle_id = RsCmu200.HANDLE_WCDMA_ID
        else:
            msg = "Wrong CMU200 Instrument name"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        self._get_visa_equipment(handle_id)

        # Query the command
        self.get_logger().info("Query GPIB command : %s" % command)
        response = self.__visa.query(command)

        # Check errors in the System Error buffer
        return_msg = self.__visa.query("SYST:ERR?")

        if return_msg != RsCmu200.GPIB_SUCCESS:
            self.get_logger().error(return_msg)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, return_msg)

        # Release CMU200 equipment with pyvisa interface
        self.__visa.disconnect()
        self.__visa = None

        # reinit DllLoader object
        self.init()

        self.switch_app_format(instrument)

        if instrument in "GSM/GPRS":
            # Set cell off
            self.get_cell_2g().set_cell_off()
            # Set voice call output
            self.get_cell_2g().get_voice_call().set_speech_configuration("SPEECH_OUTPUT")
            # Set cell on
            self.get_cell_2g().set_cell_on()
        elif instrument in "WCDMA":
            # Set cell off
            self.get_cell_3g().set_cell_off()
            # Set voice call output
            self.get_cell_3g().get_voice_call().set_speech_configuration("SPEECH_OUTPUT")
            # Set cell on
            self.get_cell_3g().set_cell_on()

        return response

    def __visa_connect_via_GPIB(self, additional_handle):
        """
        Connect to equipment via GPIB
        """
        board_id = int(self.__bench_params.get_param_value("GPIBBoardId"))
        gpib_addr = int(self.__bench_params.get_param_value("GPIBAddress"))
        gpib_addr = "GPIB%d::%d::%d" % (board_id, gpib_addr, additional_handle)

        try:
            # Initialize Gpib connection to the equipment
            handle = Visa.GpibInstrument(gpib_addr, board_id)
            # Update handle value
            self.__visa._set_handle(handle)
        except (VisaException.VisaIOError,
                VisaException.VisaIOWarning,
                VisaException.VisaTypeError) as ex:
            msg = "Connection via Gpib failed. "
            msg += str(ex)
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.VISA_LIBRARY_ERROR, msg)
