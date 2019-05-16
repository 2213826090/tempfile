"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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
:summary: FLIR thermal camera implementation.
:since: 08/01/2015
:author: vgombert
"""

from acs_test_scripts.Equipment.IEquipment import EquipmentBase
from acs_test_scripts.Equipment.ThermalCamera.Interface.IthermalCamera import IthermalCamera
from ErrorHandling.TestEquipmentException import TestEquipmentException
from telnetlib import Telnet
from ftplib import FTP, error_perm
from shutil import move
import tempfile
import time
import os
from UtilitiesFWK.Utilities import is_number, str_to_bool


class FLIRA305sc(EquipmentBase, IthermalCamera):

    """
    Class that implements FLIRA305sc.
    """
    # constant related to telnet shell
    __TLNT_PROMPT = "\\>"
    __TLNT_PATH_SEP = "\\"
    __TLNT_CARRIAGE_RETURN = "\n"
    # The philosophy is to not allow user to interact with file outside this folder
    __TLNT_PICTURE_FOLDER = "\\acs_picture\\"
    __TLNT_PALETTE_DIR = "\\flashFs\\system"
    __KELVIN_CONSTANT = 273.15

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
        EquipmentBase.__init__(self, name, model, eqt_params)
        IthermalCamera.__init__(self)
        self.__bench_params = bench_params
        self.__protocol = None
        self.__camera_ip_address = self.__bench_params.get_param_value("IP")
        self.__camera_port = self.__bench_params.get_param_value("TelnetPort", "")
        self.__login = self.__bench_params.get_param_value("FtpLoggin", "ACS")
        self.__password = self.__bench_params.get_param_value("FtpPwd", "anonymous")
        # camera setup to perform if you call the auto setup
        self.__setup_auto_focus = self.__bench_params.get_param_value("AutoFocus", False)
        self.__setup_image_color = self.__bench_params.get_param_value("ImageColor", False)
        self.__setup_nuc = self.__bench_params.get_param_value("NonUniformityCorrection", False)
        # image object parameters


        if self.__login != "" and self.__password == "":
            msg = "no password set for ftp login , this may cause a crash when you will try to pull file (ftplib limitation)"
            self.get_logger().warning(msg)

        self.__ftp = None
        self.__meas_box = ".image.sysimg.measureFuncs.mbox.1"
        self.__object_params = ".image.sysimg.basicImgData.objectParams"

    def init(self):
        """
        create picture folder if it does not exist
        """
        self.connect()

    def __del__(self):
        """
        Destructor: release automatically the connection
        """
        self.get_logger().debug("Delete %s" % str(self.get_name()))
        self.release()

    def connect(self):
        """
        Establish the connection with the equipment
        """
        self.get_logger().info("Connecting to %s" % str(self.get_name()))
        # open the telnet connection
        try :
            self.__protocol = Telnet()
            if self.__camera_port != "":
                self.__protocol.open(self.__camera_ip_address,
                                     self.__camera_port, timeout=10)
            else:
                self.__protocol.open(self.__camera_ip_address, timeout=10)
        except Exception as e:
            self.__protocol = None
            msg = "error happen when trying to connect to the camera : %s" % str(e)
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR, msg)

        # wait for prompt return
        output = self.__read_info(10)
        if output != "":
            self.get_logger().info(output)

        # check that the picture storing folder exist and create it if not
        dir_list = self.__list_dir()
        # remove the separator before comparison
        pic_dir = self.__TLNT_PICTURE_FOLDER.replace(self.__TLNT_PATH_SEP, "")
        if pic_dir not in dir_list:
            cmd = "mkdir %s" % pic_dir
            self.__send_command(cmd)
            output = self.__read_info()
            if output != "":
                self.get_logger().debug(output)

    def auto_setup(self):
        """
        Perform an auto setup depending
        of the option you set in bench configuration
        """
        # CHECK WHAT TYPE IS THE PARAM READ FRO MBENCH CONFIG
        ################### PARAMS LINK TO THE IMAGE QUALITY #########################
        auto_focus = str_to_bool(self.__bench_params.get_param_value("SetupPerformAutoFocus", "false"))

        nuc = str_to_bool(self.__bench_params.get_param_value("SetupPerformNonUniformityCorrection", "false"))
        pal_color = self.__bench_params.get_param_value("SetupSetImagePalette", "")

        ################### PARAMS LINK TO THE TEMPERATURE MEASUREMENT #########################
        objt_emmisvity = self.__bench_params.get_param_value("Emissivity", "")
        objt_reflected_tmp = self.__bench_params.get_param_value("ReflectedTemperature", "")
        distance_target = self.__bench_params.get_param_value("DistanceOfTheTarget", "")
        # atmospheric parameters
        atmos_temp = self.__bench_params.get_param_value("AtmosphericTemperature", "")
        atmos_humidity = self.__bench_params.get_param_value("AtmosphericHumidity", "")
        atmos_trans = self.__bench_params.get_param_value("AtmosphericTransmission", "")
        # external optic parameters
        ext_opt_temp = self.__bench_params.get_param_value("ExternalOpticTemperature", "")
        ext_opt_trans = self.__bench_params.get_param_value("ExternalOpticTransmission", "")

        # do all the action here to let python crash if there was an error in params above
        self.set_image_object_params(objt_emmisvity, objt_reflected_tmp, distance_target)
        self.set_atmospheric_params(atmos_temp, atmos_humidity, atmos_trans)
        self.set_external_optic_params(ext_opt_temp, ext_opt_trans)

        # compare with True to be sure that the var is boolean
        if auto_focus == True:
            self.perform_auto_focus()
        if nuc == True:
            self.perform_nuc()
        if type(pal_color) is str and pal_color != "" :
            self.set_palette(pal_color)

    def set_image_object_params(self, emissivity=None, reflected_temp=None, object_distance=None):
        """
        allow to edit several constants link to the target to tune the measurement

        :type emissivity: float
        :param emissivity: Object emissivity (0.001 to 1.0)

        :type reflected_temp: float
        :param reflected_temp: Temperature of the surroundings reflected in the object, in degree celsius.

        :type object_distance: float
        :param object_distance: Distance from the camera to the target in meter.

        """
        # convert to kelvin this value

        if is_number(reflected_temp):
            reflected_temp = self.__c_to_k(float(reflected_temp))

        params_list = {"emissivity":emissivity,
                     "ambTemp":reflected_temp,
                     "objectDistance":object_distance}
        self.__set_object_params(params_list, self.__object_params)

    def __c_to_k(self, value):
        """
        convert celsius to Kelvin
        """
        return  value + self.__KELVIN_CONSTANT

    def set_atmospheric_params(self, atmospheric_temp=None, humidity=None, transmission=None):
        """
        allow to edit several constant link to atmospheric to tune the measurement

        :type atmospheric_temp: float
        :param atmospheric_temp : Atmospheric temperature in degree celsius.

        :type humidity: float
        :param humidity: Relative humidity of the air. (0.0 to 1.0,30% = 0.30)

        :type transmission: float
        :param transmission: Estimated atmospheric transmission. If it is 0, the camera will calculate a transmission
        """
        if is_number(atmospheric_temp):
            atmospheric_temp = self.__c_to_k(float(atmospheric_temp))

        params_list = {"atmTemp":atmospheric_temp,
                     "relHum":humidity,
                     "estAtmTransm":transmission}
        self.__set_object_params(params_list, self.__object_params)

    def set_external_optic_params(self, temperature=None, transmission=None):
        """
        allow to edit several constant link to optical to tune the measurement

        :type temperature: float
        :param temperature: External Optics temperature in degree celsius. Used for heat shields, close-up lenses etc.

        :type transmission: float
        :param transmission: External Optics transmission. (0.001 to 1.0) Set to 1.0 if no external optics is present
        """
        if is_number(temperature):
            temperature = self.__c_to_k(float(temperature))

        params_list = {"extOptTemp":temperature,
                     "extOptTransm":transmission}
        self.__set_object_params(params_list, self.__object_params)

    def __set_object_params(self, params_list, ressource):
        """
        internal function to set numeric params
        """
        for key, value in params_list.items():
            if value not in [None, ""] :
                if is_number(value):
                    cmd = "rset " + ressource + ".%s %s" % (key, str(value))
                    self.__send_command(cmd)
                    output = self.__read_info()
                    if output != "":
                        msg = "error happen when performing [%s] : %s" % (cmd, output)
                        self.get_logger().error(msg)
                        raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)
                else:
                    msg = "set_image_object_params %s parameter should be a number" % (key)
                    self.get_logger().error(msg)
                    raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

    def release(self):
        """
        Releases the connection with the equipment and all resources allocated
        to its use.
        """
        self.get_logger().info("Release connection with %s" % str(self.get_name()))
        if self.__protocol is not None:
            self.__protocol.close()
        self.__close_ftp_session()

    def perform_nuc(self):
        """
        Perform a non uniformity correction
        """
        self.get_logger().info("Trying to perform a Non Uniformity Correction")
        self.__send_command("nuc")
        output = self.__read_info()
        if output != "":
            msg = "error happen when performing the nuc action : %s" % (output)
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

    def perform_auto_focus(self):
        """
        Perform an auto focus
        """
        cmd = "rset .system.focus.autofull true"
        ls_cmd = "rls .system.focus"
        focus_done = False
        self.get_logger().info("Trying to perform an auto focus")
        self.__send_command(cmd)
        timeout = 30
        output = self.__read_info()
        if output != "":
            msg = "error happen when performing the auto focus action : %s" % (output)
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        dico, raw_output = self.__get_cam_resource(ls_cmd, True, True)
        state = dico.get("STATE")
        if state in [None, ""]:
            msg = "Cant track auto focus state as STATE key is not seen : %s" % raw_output
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        elif state == "BUSY":
            self.get_logger().info("waiting for focus to be done")
            start_time = time.time()
            while time.time() - start_time < timeout:
                dico, _ = self.__get_cam_resource(ls_cmd, True, True)
                if dico.get("STATE") == "IDLE":
                    focus_done = True
                    break

            if not focus_done:
                msg = "AUTO focus exceed %ss of timeout" % (timeout)
                self.get_logger().error(msg)
                raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

    def set_palette(self, pal_color="default", reverse_pal=False, use_extrem=False):
        """
        Set camera palette

        :type pal_color: str
        :param pal_color: the palette to set: rainbow, iron, gray, or default to apply the other option on current palette

        :type reverse_pal: str
        :param reverse_pal: reverse palette

        :type use_extrem: str
        :param use_extrem: turn on extremes (above/below colors)

        """
        self.get_logger().info("Trying to change camera palette")
        cmd = "palette"
        pal_color = pal_color.lower()
        pal_dico = {"rainbow": "rainbow.pal",
                    "iron" : "iron.pal",
                    "gray":"bw.pal"}

        if reverse_pal:
            cmd += " -r"

        if use_extrem:
            cmd += " -e"

        if pal_color != "default":
            if pal_color not in pal_dico.keys():
                msg = "palette color format %s is unknown, it can only be %s" % (pal_color, str(pal_dico.keys()))
                self.get_logger().error(msg)
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

            dir_list = self.__list_dir(self.__TLNT_PALETTE_DIR)
            pal_found = False
            for element in dir_list:
                if element.endswith(".pal") and pal_dico[pal_color] in element.lower():
                    cmd += " " + element
                    pal_found = True
                    break

            if not pal_found :
                msg = "unsupported palette color format %s, it can only be %s" % (pal_color, str(pal_dico.keys()))
                self.get_logger().error(msg)
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)
        else:
            cmd += " -u"

        self.__send_command(cmd)
        output = self.__read_info()

        if output != "":
            self.get_logger().debug(output)

    def set_linear_temperature_mode(self, active, resolution_format=None):
        """
        Activate linear temperature mode
        this allow to get pixels values in Kelvin

        :type active: boolean
        :param active: True = Temperature linear mode
                       False = Raw mode (pixel value linearity)

        :type resolution_format: int
        :param resolution_format: 0 = 100 mK resolution (0 - 6535 K)
                                  1 = 10 mK resolution (0 - 653 K)
                                  leave empty to ignore this option

        """
        self.get_logger().info("Trying to set linear temperature mode")

        if resolution_format not in [None, 0, 1]:
            msg = "unsupported resolution format %s, it can only be 0 or 1" % (resolution_format)
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        if type(active) is not bool:
            msg = "unsupported active option %s, it can only be True or False" % (active)
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        cmd = "rset .tlut.active %s" % str(active).lower()
        self.__send_command(cmd)
        output = self.__read_info()
        if output != "":
            msg = "error happen when setting linear temperature mode: %s" % (output)
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        if resolution_format is not None:
            cmd = "rset .tlut.format %s" % str(resolution_format)
            self.__send_command(cmd)
            output = self.__read_info()
            if output != "":
                msg = "error happen when setting resolution format: %s" % (output)
                self.get_logger().error(msg)
                raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

    def get_image_geometry(self):
        """
        get the image geometry.
        you can use this function to configure the measurement box to the max image resolution
        :rtype: tuple
        :return: x, y, height, width
        """
        self.get_logger().info("Trying to get image geometry")
        cmd = "rls .image.sysimg.basicImgData.geometricInfo"
        raw_dico, raw_output = self.__get_cam_resource(cmd, True, True)

        if len(raw_dico) <= 0:
            msg = "error happen when reading measurement box info : %s" % raw_output
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        h = raw_dico.get("IMAGEHEIGHT")
        w = raw_dico.get("IMAGEWIDTH")
        x = raw_dico.get("UPPERLEFTX")
        y = raw_dico.get("UPPERLEFTY")

        if None in [h, w, x, y] or "" in [h, w, x, y]:
            msg = "a None or empty value was returned when reading image geometry (x,y, width, height) : (%s,%s,%s,%s) " % (x, y, w, h)
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        # cast to int
        h = int(h)
        w = int(w)
        x = int(x)
        y = int(y)
        return x, y, h, w

    def configure_measurement_box(self, active=None, x=None, y=None, box_height=None, box_width=None):
        """
        configure an area where to perform measurement.

        :type active: boolean
        :param active: True = Active measurement box , mandatory to read its value
                       False = deactivate it

        :type x: int
        :param x: the image x coordinate, 0 is the most left side

        :type y: int
        :param y: the image y coordinate, 0 is the most upper side

        :type box_height: int
        :param box_height: the image height, cannot be set outside the image zone or an error will be raised

        :type box_width: int
        :param box_width: the image width, cannot be set outside the image zone or an error will be raised
        """
        self.get_logger().info("Trying to configure a measurement box")
        cmd_list = []
        # x, y initial position is located at the top left corner
        if x is not None:
            cmd_list.append(".x %s" % str(x))
        if y is not None:
            cmd_list.append(".y %s" % str(y))
        if box_height is not None:
            cmd_list.append(".height %s" % str(box_height))
        if box_width is not None:
            cmd_list.append(".width %s" % str(box_width))
        if active is not None:
            cmd_list.append(".active %s" % str(active).lower())

        for cmd in cmd_list:
            cmd = "rset " + self.__meas_box + cmd
            self.__send_command(cmd)
            output = self.__read_info()
            if output != "":
                msg = "error happen when performing [%s] : %s" % (cmd, output)
                self.get_logger().error(msg)
                raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

    def get_measurement_from_box(self):
        """
        read the temperature info in defined box

        :rtype: dict
        :return: return a several information seen in the box like average temperature ,max temperature , min temperature
                    each information is a tuple (value, unit)
        """
        tag_list = ["MINT", "MAXT", "AVGT", "MEDIANT", "SDEVT"]
        cmd = " rls %s" % self.__meas_box
        raw_dico, raw_output = self.__get_cam_resource(cmd, True, True)
        raw_dico_key = raw_dico.keys()
        # is the measurement is not active , bad info will be return
        box_active_state = raw_dico.get("ACTIVE")
        if box_active_state in ["FALSE", "", None]:
            msg = "measurement box is not ON but %s, cant return valid information" % str(box_active_state)
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        dico_result = {}
        for wanted_tag in tag_list:
            if wanted_tag in raw_dico_key:
                # the last is a value + unit
                raw_value = raw_dico[wanted_tag]
                unit = ""
                if raw_value.endswith("C") or raw_value.endswith("K"):
                    unit = raw_value[-1]
                    value = float(raw_value[0:-1])
                else:
                    value = float(raw_value)
                dico_result[wanted_tag] = (value, unit)

        if len(dico_result) <= 0:
            msg = "error happen when reading measurement box info : %s" % raw_output
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        self.get_logger().info("measurement box return:  %s" % str(dico_result))
        return dico_result

    def __get_cam_resource(self, cmd, upper_tag=False, upper_value=False):
        """
        get the camera resource

        :type cmd: str
        :param cmd: the resource command to send, typically a rls

        :type upper_tag: boolean
        :param upper_tag: is True , cast the tag in upper case

        :type upper_value: boolean
        :param upper_value: if true , cast the value in upper case

        :rtype: dict
        :return: return a dictionary containing the resource name and its value, all as str
        """
        dico = {}
        self.__send_command(cmd)
        raw_output = self.__read_info()
        split_output = map(str.strip, raw_output.split("\n"))
        split_output = filter(None, split_output)
        for ele in split_output:
            # split by two empty space to avoid splitting composed words
            ele = ele.split("  ", 1)
            ele = map(str.strip, ele)
            # get the tag
            if len(ele) > 0:
                tag = ele[0]
                if upper_tag:
                    tag = tag.upper()
                # remove special character
                tag = tag.strip("'")
                tag = tag.strip('"')
                value = ""
                # get the tag value
                if len(ele) > 1:
                    value = str(ele[1])
                    if upper_value:
                        value = value.upper()
                    # remove special character
                    value = value.strip("'")
                    value = value.strip('"')

                dico[tag] = value

        return dico, raw_output

    def take_picture(self, file_name, wanted_format="RADIOMETRIC_JPEG", wanted_option=""):
        """
        Get a thermal picture.

        :type file_name: str
        :param file_name: the file name should not include an extension as this last one will be added depending of wanted_format value

        :type wanted_format: str
        :param wanted_format: the output file format, possible values are: RADIOMETRIC_FFF, RADIOMETRIC_JPEG,
                                                                            VISUAL_JPEG, VISUAL_FFF, SEQUENCE_FFF

        :type wanted_option: str
        :param wanted_option: the file option, possible values are: PNG, JPEG_ONLY (non-radiometric),
                                                                    NO_OVERLAY, CUT_IMAGE (with overlay to IR image size)

        :rtype: str
        :return: the path of the picture with host path separator
        """
        output_format = wanted_format.upper()
        option = wanted_option.upper()
        file_extension = ""
        # get the format
        known_format = {"RADIOMETRIC_FFF": "",
                         "RADIOMETRIC_JPEG" : "-j",
                         "VISUAL_JPEG": "-v",
                         "VISUAL_FFF": "-y",
                         "SEQUENCE_FFF": "-s"}

        if output_format not in known_format.keys():
            msg = "unsupported output format %s, it can only be %s" % (wanted_format, str(known_format.keys()))
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)
        else:
            if "FFF" in output_format:
                file_extension = ".fff"
            elif "JPEG" in output_format:
                file_extension = ".jpg"
            output_format = known_format[output_format]

        # get the option
        known_option = {"PNG": "-p",
                         "JPEG_ONLY" : "-e",
                         "NO_OVERLAY": "-o",
                         "CUT_IMAGE": "-c"}

        if option != "":
            if option not in known_option.keys():
                msg = "unsupported file option %s, it can only be %s" % (wanted_option, str(known_option.keys()))
                self.get_logger().error(msg)
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)
            else:
                option = known_option[option]

        store_path = self.__TLNT_PICTURE_FOLDER + file_name
        # delete any previous picture with this name
        self.__send_command("del %s%s" % (store_path, file_extension))
        output = self.__read_info()
        if output != "":
            self.get_logger().warning(output)

        # store picture in a dedicated directory
        cmd = "store %s %s %s" % (output_format, option, store_path)
        self.__send_command(cmd)
        output = self.__read_info()
        if output != "":
            self.get_logger().warning(output)

        # check that the picture has been generated
        dir_list = self.__list_dir(self.__TLNT_PICTURE_FOLDER)
        # we try to target exactly the rigt file name to avoid detecting a wrong a file
        final_file_name = file_name + file_extension
        if final_file_name not in dir_list:
            msg = "camera picture failed to be generated, picture capture cmd output='%s'" % output
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        return final_file_name

    def __list_dir(self, folder_path=""):
        """
        list dir current or given directory
        """
        result = []
        cmd = "dir %s" % folder_path
        cmd = cmd.strip()
        cmd += " /B"

        self.__send_command(cmd)
        output = self.__read_info()
        if output.lower() not in ["", "file not found"]:
            split_output = map(str.strip, output.split("\n"))
            result = filter(None, split_output)

        return result

    def __read_info(self, timeout=10):
        """
        read all info until prompt or timeout is reached

        :rtype: string
        :return: string
        """
        msg = self.__protocol.read_until(self.__TLNT_PROMPT, timeout).strip()
        if msg.endswith(self.__TLNT_PROMPT):
            msg = msg.replace(self.__TLNT_PROMPT, "")
            split_msg = map(str.strip, msg.split("\n"))
            split_msg = filter(None, split_msg)
            msg = "\n".join(split_msg)

        return msg

    def __send_command(self, cmd):
        """
        send a command trough telnet
        """
        if self.__protocol is not None:
            sent_cmd = cmd.strip()
            self.get_logger().debug("sending command [%s]" % sent_cmd)
            if not sent_cmd.endswith(self.__TLNT_CARRIAGE_RETURN):
                sent_cmd += self.__TLNT_CARRIAGE_RETURN
            self.__protocol.write(sent_cmd)
        else:
            msg = "camera communication protocol is not opened, cant send command : %s" % cmd
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR, msg)

    # *******************************************FTP related operation******************************************
    def pull_picture(self, file_name, destination_folder, change_name=None):
        """
        pull a picture

        :type file_name: str
        :param file_name: the file name from the camera

        :type destination_folder: str
        :param destination_folder: the folder on the host side where picture will be stored

        :rtype: str
        :return: the pulled file path

        """
        self.__open_ftp_session()
        # write file at given place
        if not change_name is None:
            destination_name = change_name
        else:
            destination_name = file_name.rsplit(self.__TLNT_PATH_SEP, 1)[-1].strip()
        destination_file = os.path.join(destination_folder, destination_name)
        tmpd = tempfile.gettempdir()
        tmp_file = os.path.join(tmpd, destination_name)
        file_full_path = self.__get_pic_path(file_name)

        self.get_logger().info("trying to pull %s to host file %s" % (file_full_path, destination_file))
        # create a temporary file and move it to the right folder after some check
        with open(tmp_file, "wb") as f:
            self.__ftp.retrbinary("RETR " + file_full_path, f.write)

        self.__close_ftp_session()
        # check the generated file size before copying it
        if os.path.getsize(tmp_file) > 0:
            # consider the case where destination folder may be tmpdir
            if destination_file != tmp_file:
                # remove any previous existing file as copy will fail otherwise
                if os.path.exists(destination_file):
                    os.remove(destination_file)
                move(tmp_file, destination_file)
        else:
            msg = "problem happen after pulling file %s : it size is 0" % (file_name)
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.READ_PARAMETER_ERROR, msg)

        return destination_file

    def __get_pic_path(self, file_name):
        """
        return the right path to a pic
        """
        if self.__TLNT_PICTURE_FOLDER not in file_name:
            file_full_path = self.__TLNT_PICTURE_FOLDER + file_name
        else:
            file_full_path = file_name

        return file_full_path

    def pull_all_pictures(self, destination_folder):
        """
        pull all pictures

        :type destination_folder: str
        :param destination_folder: the folder on the host side where picture will be stored
        """
        self.__open_ftp_session()
        # write file at given place
        picture_list = self.__ftp.nlst(self.__TLNT_PICTURE_FOLDER)
        tmpd = tempfile.gettempdir()
        file_list = []

        if len(picture_list) > 0:
            # pull each file one by one
            for file_name in picture_list:
                destination_file = os.path.join(destination_folder, file_name)
                tmp_file = os.path.join(tmpd, file_name)
                # as we list file name from a folder, we need to reconstruct the full path
                file_full_path = self.__get_pic_path(file_name)

                self.get_logger().info("Trying to pull %s to host folder %s" % (file_full_path, destination_folder))
                with open(tmp_file, "wb") as f:
                    self.__ftp.retrbinary("RETR " + file_full_path, f.write)

                # check the generated file size before copying it
                if os.path.getsize(tmp_file) > 0:
                    if os.path.exists(destination_file):
                        os.remove(destination_file)
                    move(tmp_file, destination_folder)
                else:
                    msg = "problem happen after pulling file %s : it size is 0" % (file_full_path)
                    self.get_logger().error(msg)
                    raise TestEquipmentException(TestEquipmentException.READ_PARAMETER_ERROR, msg)

                file_list.append(destination_file)

        else:
            self.get_logger().warning("there is no picture to pull")

        self.__close_ftp_session()
        return file_list

    def delete_picture(self, file_name):
        """
        Delete a given picture

        :type file_name: str
        :param file_name: the file name from the camera
        """
        # use ftp to do the job, it can be done trough telnet too
        self.__open_ftp_session()
        file_full_path = self.__get_pic_path(file_name)
        # write file at given place
        self.get_logger().info("trying to delete %s" % (file_full_path))

        try :
            self.__ftp.delete(file_full_path)
        except error_perm as e:
            self.get_logger().warning("error when trying to delete %s: %s" % str(file_name, str(e)))
        self.__close_ftp_session()

    def delete_all_pictures(self):
        """
        Delete all generated pictures
        """
        # use ftp to do the job, it can be done trough telnet too
        self.__open_ftp_session()
        # write file at given place
        self.get_logger().info("trying to delete all pictures in %s folder" % (self.__TLNT_PICTURE_FOLDER))
        picture_list = self.__ftp.nlst(self.__TLNT_PICTURE_FOLDER)
        if len(picture_list) > 0:
            # pull each file one by one
            for file_name in picture_list:
                file_full_path = self.__get_pic_path(file_name)
                try :
                    self.__ftp.delete(file_full_path)
                except error_perm as e:
                    self.get_logger().warning("error when trying to delete picture %s: %s" % (file_full_path, str(e)))

        self.__close_ftp_session()

    def __open_ftp_session(self):
        """
        open a ftp session
        """
        if self.__ftp is None:
            self.__ftp = FTP(self.__camera_ip_address)  # connect to host, default port
            # need to check what is the cost to call this each time
        output = self.__ftp.login(user=self.__login, passwd=self.__password).strip()
        if output != "" and "already logged in" not in output.lower() and "logged in, proceed" not in output.lower():
            msg = "error happen when trying to open ftp session: %s" % str(output)
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR, msg)

    def __close_ftp_session(self):
        """
        close ftp session
        """
        if self.__ftp is not None:
            try:
                self.__ftp.quit()
            except Exception as e:
                self.get_logger().warning("error happen when closing ftp session %s" % str(e))
            self.__ftp = None
