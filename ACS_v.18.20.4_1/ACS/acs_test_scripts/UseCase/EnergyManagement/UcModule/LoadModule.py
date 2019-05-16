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
:summary: This module represent the load you can apply during test according to what your DUT can support.
:author: vgomberx
:since: 25/11/2013
"""
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsBaseException import AcsBaseException
from acs_test_scripts.UseCase.EnergyManagement.UcModule.VideoCaptureModule import VideoCaptureModule
from acs_test_scripts.UseCase.EnergyManagement.UcModule.MultimediaModule import MultimediaModule
from acs_test_scripts.UseCase.EnergyManagement.UcModule.WifiModule import WifiModule
from acs_test_scripts.UseCase.EnergyManagement.UcModule.OverMind import OverMind
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT


class LoadModule():
    """
    init.
    """
    __LOG_TAG = "[LOAD_MODULE]\t"
    SCREEN_ON = "SCREEN_ON"
    BLUETOOTH_ON = "BLUETOOTH"
    WIFI_ON = "WIFI"
    WIFI_STANDBY = "WIFI_STANDBY"
    VIDEO = "VIDEO"
    AUDIO = "AUDIO"
    TORCHLIGHT = "TORCHLIGHT"
    VIBRA = "VIBRA"
    VIDEO_CAPTURE = "VIDEO_CAPTURE"
    GPS_ON = "GPS_ON"
    EM_BENCHCONFIG_TAG = "LOADMODULE"
    BENCHCONFIG_LOAD = "FROM_BENCHCONFIG"


    def __init__(self):
        """
        parameter to initialize this module.
        all possible load parameters are declared as public variables.

        this is to help to discharge or to apply load during a test and restart them
        NOT to test the load technology directly.
        """
        overmind = OverMind()
        self.__logger = LOGGER_TEST_SCRIPT
        self.__logger.info(self.__LOG_TAG + "INIT")
        self.__device = overmind.get_instance(overmind.DEVICE)
        self.__tc_parameters = overmind.get_instance(overmind.TC_PARAMETERS)
        bench_config = overmind.get_instance(overmind.BENCH_CONFIG)
        self.__load = {}
        self.__load_list_priority = [LoadModule.SCREEN_ON, LoadModule.TORCHLIGHT, LoadModule.WIFI_ON, LoadModule.WIFI_STANDBY, LoadModule.GPS_ON,
                              LoadModule.BLUETOOTH_ON, LoadModule.VIDEO, LoadModule.VIDEO_CAPTURE, LoadModule.AUDIO, LoadModule.VIBRA]

        # check if a default configuration is present on the board
        if bench_config.has_parameter(self.EM_BENCHCONFIG_TAG):
            default_conf = bench_config.get_parameters(self.EM_BENCHCONFIG_TAG)
            self.__default_load_from_bench = default_conf.get_param_value("DefaultLoad", "")
            self.__media_to_use = default_conf.get_param_value("MediaPath", "")
            self.__volume_to_use = default_conf.get_param_value("MediaVolume", "")
            self.__wifi_ap_to_use = default_conf.get_param_value("WifiAP", "")
        else:
            self.__default_load_from_bench = ""

    def add_load(self, load_text):
        """
        add given load to the load dict.
        :type load_text: str
        :param load_text: load to add in str format

        :rtype : bool
        :return : True if all load has been successful added, false otherwise
        """
        result = None
        # if user ask for default load, load it from device catalog
        if str(load_text).upper() == self.BENCHCONFIG_LOAD:
            self.__logger.info(self.__LOG_TAG + "special load %s has been asked" % self.BENCHCONFIG_LOAD)
            load_text = self.__default_load_from_bench

        if load_text in [None, ""]:
            self.__logger.debug(self.__LOG_TAG + "you are trying to add an empty load, skipping the action")
            result = False

        else:
            self.__logger.info(self.__LOG_TAG + "Trying to Add load %s to load list" % load_text)
            raw_list_load = self.__str_to_list(load_text)
            load_list = self.__sort_load(raw_list_load)
            # complete the sorted load list with unknown load in order to have error message below
            for element in raw_list_load:
                if not element in load_list:
                    load_list.append(element)

            if LoadModule.AUDIO in load_list and LoadModule.VIDEO in load_list:
                    tmp_txt = "Cannot start audio and video at the same time, please remove one"
                    self.__logger.error(tmp_txt)
                    raise DeviceException(DeviceException.INVALID_PARAMETER, tmp_txt)
            if LoadModule.VIDEO in load_list and LoadModule.VIDEO_CAPTURE in load_list:
                    tmp_txt = "Cannot start video playback and video record at the same time, please remove one"
                    self.__logger.error(tmp_txt)
                    raise DeviceException(DeviceException.INVALID_PARAMETER, tmp_txt)

            for load in load_list:
                load = load.upper()
                if load not in self.__load.keys():
                    #---------------------------------------------------------------------
                    if load in [LoadModule.VIDEO, LoadModule.AUDIO]:
                        mum = MultimediaModule(load)
                        if not mum.is_media_defined():
                            mum.configure_media_file(self.__media_to_use, self.__volume_to_use)
                            if not mum.is_media_defined():
                                tmp_txt = "Cant use %s load, media path got a problem : %s" % (load, self.__media_to_use)
                                self.__logger.error(tmp_txt)
                                raise DeviceException(DeviceException.INVALID_PARAMETER, tmp_txt)

                        self.__load[load] = {"start": (mum.start_media, [True]),
                                         "stop": (mum.stop_media, []),
                                         "persistent": False,
                                         "check": (mum.is_media_running, [])}

                    #---------------------------------------------------------------------
                    elif load == LoadModule.VIDEO_CAPTURE:
                        capture = VideoCaptureModule()
                        self.__load[load] = {"start": (capture.start_recording_load, []),
                                         "stop": (capture.stop_recording, []),
                                         "check": (capture.check_recording_state, []),
                                         "persistent": False}

                    #---------------------------------------------------------------------
                    # wifi ON and wifi standby are redundant
                    elif load == LoadModule.WIFI_ON and not LoadModule.WIFI_STANDBY in load_list:
                        wifi = self.__device.get_uecmd("Networking", True)
                        self.__load[load] = {"start": (wifi.set_wifi_power, ["on"]),
                                         "stop": (wifi.set_wifi_power, ["off"]),
                                        "persistent": True}
                    #---------------------------------------------------------------------
                    elif load == LoadModule.WIFI_STANDBY:
                        waf = WifiModule()
                        if not waf.is_wifi_ap_defined():
                            waf.configure_wifi_ap(self.__wifi_ap_to_use)
                            if not waf.is_wifi_ap_defined():
                                tmp_txt = "Cant use %s load, wifi AP name got a problem : %s" % (load, self.__wifi_ap_to_use)
                                self.__logger.error(tmp_txt)
                                raise DeviceException(DeviceException.INVALID_PARAMETER, tmp_txt)

                        self.__load[load] = {"start": (waf.wifi_connect, [True]),
                                         "stop": (waf.release, []),
                                         "persistent": True}
                    #---------------------------------------------------------------------
                    elif load == LoadModule.BLUETOOTH_ON:
                        bt = self.__device.get_uecmd("LocalConnectivity", True)
                        self.__load[load] = {"start": (bt.set_bt_power, ["on"]),
                                        "stop": (bt.set_bt_power, ["off"]),
                                        "persistent": True}

                    #---------------------------------------------------------------------
                    # this torch light cant be used if video capture is running
                    # TODO: torchlight should be linked to video capture code
                    elif load == LoadModule.TORCHLIGHT and not LoadModule.VIDEO_CAPTURE in load_list:
                        phone_api = self.__device.get_uecmd("PhoneSystem", True)
                        self.__load[load] = {"start": (phone_api.set_torchlight, ["on"]),
                                        "stop": (phone_api.set_torchlight, ["off"]),
                                        "persistent": False}
                    #---------------------------------------------------------------------
                    elif load == LoadModule.VIBRA:
                        phone_api = self.__device.get_uecmd("PhoneSystem", True)
                        self.__load[load] = {"start": (phone_api.set_vibration, [True]),
                                         "stop": (phone_api.set_vibration, [False]),
                                         "persistent": False,
                                         "check": (phone_api.get_vibration_state, [])}
                    #---------------------------------------------------------------------
                    elif load == LoadModule.SCREEN_ON:
                        phone_api = self.__device.get_uecmd("PhoneSystem", True)
                        self.__load[load] = {"start": (self.screen_on, []),
                                         "stop": (self.screen_off, []),
                                         "persistent": False,
                                         "persistent_action": (self.screen_restart, []),
                                         "check": (self.screen_check, [])}
                    #---------------------------------------------------------------------
                    elif load == LoadModule.GPS_ON:
                        phone_api = self.__device.get_uecmd("Location", True)
                        self.__load[load] = {"start": (phone_api.set_gps_power, [True]),
                                         "stop": (phone_api.set_gps_power, [False]),
                                         "persistent": True,
                                         "check": (phone_api.get_gps_power_status, [])}
                    #---------------------------------------------------------------------
                    else:
                        self.__logger.error(self.__LOG_TAG + "Unsupported load type : %s" % load)
                        result = False
                        continue

                    # if we arrive here it means that the load has been added to the list
                    self.__load[load]["running"] = False
                    self.__logger.info(self.__LOG_TAG + "load %s added" % load)
                    # result is true only if no error has been seen
                    if result is None :
                        result = True

        return result

    def start_load(self, specify_load=None, raise_error=True):
        """
        start load.
        Will play with all load initialized with this object by default
        or just the one you specify among them.

        :type specify_load: str
        :param specify_load: load to start

        :type raise_error: boolean
        :param raise_error: if set to true raise an error at the first meet
        """
        if not self.__is_load_list_empty():
            if specify_load is not None:
                specify_load = self.__str_to_list(specify_load)
                specify_load = self.__sort_load(specify_load)
                self.__logger.info(self.__LOG_TAG + "Starting LOAD  %s" % str(specify_load))
            else:
                self.__logger.info(self.__LOG_TAG + "Starting LOAD  %s" % str(self.__load.keys()))

            error = None
            for load in self.__sort_load(self.__load.keys()):
                # skip the load if we specify which one and we are not in presence of wanted one
                if specify_load is not None and load not in specify_load:
                    continue

                load_obj = self.__load.get(load)
                error = self.__exec_load(load_obj["start"])
                if error is not None:
                    if raise_error:
                        raise error
                    else:
                        self.__logger.warning("error happen during load activation : " + str(error))
                else:
                    # set load running state to track what we need to clean
                    self.__load[load]["running"] = True

    def clean(self):
        """
        Stop all load that has been set in running state from this module point of view.
        designed for cleaning load in uc tear down, this is why it does not raise error
        but return a boolean

        :rtype: boolean
        :return: True if there is no error , False if there was at least one error
        """
        result = True
        if not self.__is_load_list_empty():
            one_error_txt = ""
            self.__logger.info(self.__LOG_TAG + "Trying to stop remaining LOAD activated by this module among %s" % str(self.__load.keys()))

            for load in self.__sort_load(self.__load.keys()):
                # skip the load if we specify which one and we are not in presence of wanted one

                load_obj = self.__load.get(load)
                if load_obj.get("running") == True:
                    error = self.__exec_load(load_obj["stop"])
                    if error is not None:
                        one_error_txt += str(error) + " \n"
                    else:
                        # set load running state to track what we need to clean
                        self.__load[load]["running"] = False

                # collect all errors and log them here, this will help to stop all load before crashing
                if one_error_txt != "":
                    one_error_txt = "Error happened in stop load function : \n" + one_error_txt
                    self.__logger.error(one_error_txt)
                    result = False

        return result

    def __is_load_list_empty(self):
        """
        test if there is any added load

        :rtype: boolean
        :return: True there is no load added to the module
        """
        return len(self.__load) == 0

    def stop_load(self, specify_load=None, raise_error=True):
        """
        stop all load
        usually used at the end of test.

        :type specify_load: str
        :param specify_load: load to stop

        :type raise_error: boolean
        :param raise_error: if set to true raise an error if at least one load failed ,
                            the error is raised after disabling all load.
        .. todo:: MOVE THE RAISE ERROR AFTER TURNING OFF ALL LOAD
        """
        if not self.__is_load_list_empty():
            one_error_txt = ""

            if specify_load is not None:
                specify_load = self.__str_to_list(specify_load)
                specify_load = self.__sort_load(specify_load)
                self.__logger.info(self.__LOG_TAG + "Stopping LOAD  %s" % str(specify_load))
            else:
                self.__logger.info(self.__LOG_TAG + "Stopping LOAD  %s" % str(self.__load.keys()))

            for load in self.__sort_load(self.__load.keys()):
                # skip the load if we specify which one and we are not in presence of wanted one
                if specify_load is not None and load not in specify_load:
                    continue

                load_obj = self.__load.get(load)
                error = self.__exec_load(load_obj["stop"])
                if error is not None:
                    one_error_txt += str(error) + " \n"
                else:
                    # set load running state to track what we need to clean
                    self.__load[load]["running"] = False

            # collect all errors and log them here, this will help to stop all load before crashing
            if one_error_txt != "":
                one_error_txt = "Error happened in stop load function : \n" + one_error_txt
                if raise_error:
                    self.__logger.error(one_error_txt)
                    raise AcsBaseException(AcsBaseException.OPERATION_FAILED, one_error_txt)

                else:
                    self.__logger.warning("error happen during load activation :" + one_error_txt)

    def restart_load(self, specify_load=None, raise_error=True, consider_only_checkable_load=False):
        """
        Will restart load that have stopped.
        only non persistent load are restarted
        Will play with all load initialized with this object by default
        or just the one you specify among them.

        :type specify_load: str
        :param specify_load: load to restart

        :type raise_error: boolean
        :param raise_error: if set to true raise an error at the first meet
        """
        if specify_load is not None:
            specify_load = self.__str_to_list(specify_load)
            specify_load = self.__sort_load(specify_load)
            self.__logger.info(self.__LOG_TAG + "Restarting LOAD %s" % str(specify_load))
        else:
            self.__logger.info(self.__LOG_TAG + "Restarting LOAD %s" % str(self.__load.keys()))

        error = None
        for load in self.__sort_load(self.__load.keys()):
            # skip the load if we specify which one and we are not in presence of wanted one
            if specify_load is not None and load not in specify_load:
                continue

            load_obj = self.__load.get(load)
            if  not load_obj.get("persistent"):
                objet_to_exec = load_obj.get("persistent_action")
                # if the load is not persistent and no persistent action
                # set , use the simple start
                if objet_to_exec is None:
                    objet_to_exec = load_obj["start"]

                # by default we consider that the load is not activated
                check_state = False

                if consider_only_checkable_load:
                    check_fct = load_obj.get("check")
                    if check_fct is not None:
                        check_state = self.__exec_check(check_fct)
                    else:
                        # skip uncheckable load
                        continue

                if not check_state:
                    error = self.__exec_load(objet_to_exec)

            if error is not None:
                if raise_error:
                    raise error
                else:
                    self.__logger.warning("error happen during load %s activation :%s" + str(error))

    def screen_on(self):
        """
        try to set phone screen to always be on
        """
        phone_api = self.__device.get_uecmd("PhoneSystem", True)
        phone_api.set_screen_timeout(36000)
        phone_api.set_brightness_mode("manual")
        phone_api.set_display_brightness(100)
        phone_api.set_stay_on_while_plugged_in(True)
        phone_api.wake_screen()
        phone_api.set_phone_lock("off")

    def screen_off(self):
        """
        try revert the works done by screen_on
        """
        phone_api = self.__device.get_uecmd("PhoneSystem", True)
        phone_api.set_screen_timeout(60)
        phone_api.set_stay_on_while_plugged_in(False)
        phone_api.wake_screen()

    def screen_restart(self):
        phone_api = self.__device.get_uecmd("PhoneSystem", True)
        phone_api.wake_screen()
        phone_api.set_phone_lock("off")

    def screen_check(self):
        result = False
        phone_api = self.__device.get_uecmd("PhoneSystem", True)
        backlight = phone_api.get_backlight_level()
        if backlight > 25:
            result = True
        return result

    def __exec_load(self, fct_bundle):
        """
        Execute the load start or restart
        function

        :type fct_bundle: tuple
        :param fct_bundle:  (fct , parameters)

        :rtype: Exception
        :return: any excpetion object raised
        """
        # Retrieve input parameters
        uecmd_fct = fct_bundle[0]
        parameters = fct_bundle[1]
        error = None
        try:
            # pylint: disable=W0142
            uecmd_fct(*parameters)

        except AcsBaseException as euce:
            error = euce

        return error

    def __exec_check(self, fct_bundle):
        """
        execute check function.

        :type fct_bundle: tuple
        :param fct_bundle:  (fct , parameters)

        :rtype: boolean
        :return: True if load on , False otherwise
        """
        # Retrieve input parameters
        uecmd_fct = fct_bundle[0]
        parameters = fct_bundle[1]
        result = uecmd_fct(*parameters)
        return result

    def __str_to_list(self, load):
        """
        convert load from str to a list with no duplicate elements.
        """
        separator = ""
        if "," in load:
            separator = ","
        elif ";" in load:
            separator = ";"

        result = []
        if separator != "":
            load_list = load.split(separator)
        else:
            load_list = [load]

        for element in load_list:
            element = str(element).strip().upper()
            if  len(element) > 1 and element not in result:
                result.append(element)

        return result

    def __sort_load(self, load_list):
        """
        Sort load in an order where there activation does not affect previously activated loads.
        like unlocking screen first before playing video.
        """
        sorted_result = []
        # now sort element by order
        for element in self.__load_list_priority:
            if element in load_list:
                sorted_result.append(element)

        return sorted_result
