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
:summary:
:since: 20/10/2010
:author: dgonzalez
"""

from Lib.ConversionToolBox.FileManager import ConversionFileManager

class ConversionToolBox(object):

    """
    Conversion toolbox class, used to convert
    ARFCN channel to uplink/downlink frequency.
    """

    def __init__(self,
                 wcdma_filename="wcdma_conversion.xml",
                 gsm_filename="gsm_conversion.xml",
                 wifi_filename="wifi_conversion.xml",
                 directory='./'):
        """
        Constructor of the class.

        :type wcdma_filename: str
        :param wcdma_filename: Name of the wcdma conversion's xml file.

        :type gsm_filename: str
        :param gsm_filename: Name of the gsm conversion's xml file.

        :type wifi_filename: str
        :param wifi_filename: Name of the wifi conversion's xml file.

        :type directory: str
        :param directory: Files' path.
        """
        file_manager = ConversionFileManager(
            wcdma_filename,
            gsm_filename,
            wifi_filename,
            directory)
        self.__wcdma_conversion = file_manager.\
            parse_wcdma_conversion()
        self.__gsm_conversion = file_manager.\
            parse_gsm_conversion()
        self.__wifi_conversion = file_manager.\
            parse_wifi_conversion()

    def convert_wcdma_channelscript_to_array(self, band, expr):
        """
        Returns a list of wcdma channel from an expression.
        The expression define a set of arfcn channel.
        An exception is raised if arfcn values aren't
        defined in the band given in parameter.
        You can choose between:
            - a value (ex: '453')
            - a set of values (ex: '453, 457,465')
            - a range of values given a start, end and scale value
                (ex: 'range(456, 490, 3)')
            - the keyword 'all'
                (means all possible channels for band given in parameter)

        :type band: str
        :param band: Name of the wcdma band.

        :type expr: str
        :param expr: expression representing a set of ARFCN values.

        :rtype: object
        :return: Array of arfcn values.

        :raise Exception: Exception raised if there is no
        relation between band and retrieved arfcn values.
        """
        expr = str(expr).lower()
        if str(expr) == "all":
            values = self.__retrieve_all_channels(band, self.__wcdma_conversion)
        else:
            values = eval(expr)
        array = []
        # Add retrieved value(s) into array.
        if not hasattr(values, '__iter__'):
            array.append(values)
        else:
            array.extend(values)
        self.__assert_parameters_matches(band, array, self.__wcdma_conversion)
        return array

    def convert_wcdma_channel_to_freq(self, band, channel_array):
        """
        Returns frequency's values converted from a set of wcdma channel.

        :type band: str
        :param band: Name of the wcdma's band.

        :type channel_array: object
        :param channel_array: list containing arfcn values.

        :rtype: object
        :return: Tuple containing the list of uplink frequencies and a list of downlink frequencies.
        """
        frequencies_UL_array = []
        frequencies_DL_array = []
        frequencies_mgmt = self.__wcdma_conversion[str(band)]
        for channel in channel_array:
            for freq_mgmt in frequencies_mgmt:
                if freq_mgmt.contains(channel):
                    frequencies_UL_array.append(freq_mgmt.apply_formula(channel, 'UL'))
                    frequencies_DL_array.append(freq_mgmt.apply_formula(channel, 'DL'))
        return frequencies_UL_array, frequencies_DL_array

    def convert_gsm_channelscript_to_array(self, band, expr):
        """
        Returns a list of gsm channel from an expression.
        The expression define a set of arfcn channel.
        An exception is raised if arfcn values aren't
        defined in the band given in parameter.
        You can choose between:
            - a value (ex: '453')
            - a set of values (ex: '453, 457,465')
            - a range of values given a start, end and scale value
                (ex: 'range(456, 490, 3)')
            - the keyword 'all'
                (means all possible channels for band given in parameter)

        :type band: str
        :param band: Name of the gsm band.

        :type expr: str
        :param expr: expression representing a set of ARFCN values.

        :rtype: object
        :return: Array of arfcn values.

        :raise Exception: Exception raised if there is no
        relation between band and retrieved arfcn values.
        """
        expr = str(expr).lower()
        if str(expr) == "all":
            values = self.__retrieve_all_channels(band, self.__gsm_conversion)
        else:
            values = eval(expr)
        array = []
        if not hasattr(values, '__iter__'):
            array.append(values)
        else:
            array.extend(values)
        self.__assert_parameters_matches(band, array, self.__gsm_conversion)
        return array

    def convert_gsm_channel_to_freq(self, band, channel_array):
        """
        Returns frequency's values converted from a set of gsm channel.

        :type band: str
        :param band: Name of the gsm's band.

        :type channel_array: object
        :param channel_array: list containing arfcn values.

        :rtype: object
        :return: Tuple containing the list of uplink frequencies and a list of downlink frequencies.
        """
        frequencies_UL_array = []
        frequencies_DL_array = []
        frequencies_mgmt = self.__gsm_conversion[str(band)]
        for channel in channel_array:
            for freq_mgmt in frequencies_mgmt:
                if freq_mgmt.contains(channel):
                    frequencies_UL_array.append(freq_mgmt.apply_formula(channel, 'UL'))
                    frequencies_DL_array.append(freq_mgmt.apply_formula(channel, 'DL'))
        return frequencies_UL_array, frequencies_DL_array

    def convert_wifi_channel_to_freq(self, wifi_band, wifi_channel):
        """
        Returns the middle range frequency given a wifi band and a wifi channel.

        :type wifi_band: str
        :param wifi_band: Name of the wifi's band ('2.4GHz' or '5.0GHz').

        :type wifi_channel: int
        :param wifi_channel: One wifi channel.

        :rtype: int
        :return: Middle range frequency.
        """
        try:
            err_msg = "Bad wifi band (%s)." % wifi_band
            dico = self.__wifi_conversion[wifi_band]
            err_msg = "Bad wifi channel (%s) for %s band." % (wifi_channel, wifi_band)
            freq = dico[str(wifi_channel)]
            return float(freq)
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            raise Exception(err_msg)

    def gsm_get_max_pcl(self, gsm_band):
        """
        Returns the max Power control level (only for GSM).

        :type gsm_band: str
        :param gsm_band: Name of GSM band.

        :rtype: int
        :return: Max power control level.
        """
        pcl0_bands = ['DCS', 'PCS']
        gsm_band = str(gsm_band).upper()
        if gsm_band in pcl0_bands:
            return 0
        return 5

    def gsm_convert_rxlevel_to_dbm(self, rxlevel, offset=0):
        """
        Returns minimum and maximum values in dBm from a given rxlevel.

        :type rxlevel: int
        :param rxlevel: RxLevel value (between 0 and 63).

        :type offset: float
        :param offset: Offset.

        :rtype: object
        :return: Pair of min and max rxlevel values in dBm.
        """
        minimum = None
        maximum = None
        if rxlevel == 63:
            minimum = -48
            maximum = 0
        elif rxlevel == 0:
            minimum = -200
            maximum = -110
        elif rxlevel in range(1, 63):
            minimum = -111 + rxlevel + offset
            maximum = minimum + 1
        else:
            raise Exception("RxLevel not in range [0;63]")
        return minimum, maximum

    def gsm_pcl_to_dbm(self, band_name, pcl):
        """
        Returns conversion of PCL to power level.

        :type band_name: str
        :param band_name: GSM band's name.

        :type pcl: int
        :param pcl: Power control level value.

        :rtype: int
        :return: power level in dBm.
        """
        upper_band = str(band_name).upper()
        gsm_function = "-2*x_pcl + 43"
        dcs_pcs_function = "-2*x_pcl + 30"
        x_pcl = int(pcl)
        result = 0
        if upper_band in ['DCS', 'PCS']:
            if 0 <= x_pcl <= 15:
                result = eval(dcs_pcs_function)
                return result
            else:
                msg = "Bad pcl (" + pcl + ") for the band " + band_name + "."
                raise Exception(msg)
        elif upper_band in ['GSM850', 'GSM900', 'GSM']:
            if 5 <= x_pcl <= 19:
                result = eval(gsm_function)
                return result
            else:
                msg = "Bad pcl (" + pcl + ") for the band " + band_name + "."
                raise Exception(msg)
        else:
            msg = "Unknown GSM band (" + band_name + ")."
            raise Exception(msg)

    def __assert_parameters_matches(self, band, channels, conversion):
        """
        Assert that channels are allowed to the band given in parameter.
        Raise an exception if one or more frequencies don't match with the band.

        :type band: str
        :param band: Band name.

        :type channels: object
        :param channels: List of channel values.

        :type conversion: dict
        :param conversion: Dictionary containing data's conversion.

        :raise Exception: channels and band parameters don't match.
        """
        freq_not_allowed = []
        for channel in channels:
            is_allowed = False
            for freq_mgmt in conversion[str(band)]:
                if freq_mgmt.contains(channel):
                    is_allowed = True
                    break
            if not is_allowed:
                freq_not_allowed.append(channel)
        if len(freq_not_allowed) != 0:
            err_msg = "Frequencies " + str(freq_not_allowed)\
                + " not allowed for Band " + str(band)
            raise Exception(err_msg)

    def __retrieve_all_channels(self, band, conversion):
        """
        Retrieve all channels from a given band.
        Raise an exception if band is unknown.

        :type band: str
        :param band: Band name.

        :type conversion: dict
        :param conversion: Dictionary containing data's conversion.

        :raise Exception: band is unknown.

        """
        try:
            channels = []
            for freq_mgmt in conversion[str(band)]:
                channels.extend(freq_mgmt.get_all_channels())
            return channels
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            raise Exception(
                "Can't retrieve all possible channels due to an unknown band : "
                + str(band))
