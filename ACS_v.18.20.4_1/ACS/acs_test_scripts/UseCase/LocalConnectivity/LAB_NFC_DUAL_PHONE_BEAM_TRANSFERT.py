"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
The source code contained or described herein and all documents related
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
:summary: test for BEAM reliability
:since: 07/07/2014
:author: jortetx
"""

from acs_test_scripts.UseCase.LocalConnectivity.LAB_NFC_BASE import LabNfcBase
from UtilitiesFWK.Utilities import Global
from Device.DeviceManager import DeviceManager
import posixpath
import time
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LabNfcDualPhoneBeamTransfert(LabNfcBase):

    # the name of ref phone is the name in BENCH_CONFIG file
    REF_PHONE = "PHONE2"

    def __init__(self, tc_name, global_config):
        LabNfcBase.__init__(self, tc_name, global_config)

        # second phone init
        self._phone_ref = DeviceManager().get_device(LabNfcDualPhoneBeamTransfert.REF_PHONE)
        self._phone_ref.connect_board()

        # Check if we have the ref phone available
        if self._phone_ref is not None:
            if not self._phone_ref.is_available():
                DeviceManager().boot_device(LabNfcDualPhoneBeamTransfert.REF_PHONE)
            self._nfc_api_phone_ref = self._phone_ref.get_uecmd("LocalConnectivity")

        # get PhoneSystem uecmd instance for the ref phone
        self._phonesystem_api_dut = self._device.get_uecmd("PhoneSystem")
        self._phonesystem_api_ref =  self._phone_ref.get_uecmd("PhoneSystem")

        self._appmgmt_api = self._phone_ref.get_uecmd("AppMgmt")
        # get file uecmd instance
        self._file_api_dut = self._device.get_uecmd("File")
        self._file_api_ref = self._phone_ref.get_uecmd("File")


        # Get NFC Tags application package name
        self._tag_app = str(self._dut_config.get("TagsAppPackageName"))
        # Get Browsers application package name (need to disable in order to be able to read URI tags)
        self._browser_app = str(self._dut_config.get("BrowserAppPackageName"))
        self._chrome_app = str(self._dut_config.get("ChromeAppPackageName"))

        # force phone 2 state unlock and display on
        # for the DUT this is made in LiveNfcBase class
        self._phonesystem_api_ref.display_on()
        self._phonesystem_api_ref.set_phone_lock(0)
        self._nfc_api_phone_ref.force_nfc_state(1)

        # BE careful here we have to be sure that the file is here (setup embbeded)
        self.__file_to_share = str(self._tc_parameters.get_param_value("FILE_TO_SHARE"))

        self.__beam_duration = int(self._tc_parameters.get_param_value("BEAM_DURATION"))
        self.__direction = str(self._tc_parameters.get_param_value("DIRECTION")).upper()

        self._nfc_robot_param = global_config.benchConfig.get_parameters("NFC_ROBOT1")
        self._beam_path = self._tc_parameters.get_param_value("BEAM_PATH")

        # get ref phone position
        self.__ref_phone_position = self._tc_parameters.get_param_value("REF_PHONE_POSITION")

        self._tag_up = self._nfc_robot_param.get_param_value(self.__ref_phone_position + "Up")
        self._tag_down = self._nfc_robot_param.get_param_value(self.__ref_phone_position + "Down")
        self._tag_X = self._nfc_robot_param.get_param_value(self.__ref_phone_position + "X")
        # get the antenna position
        antenna_position = self._device.get_config("NfcAntennaPosition")
        self._tag_Y = str(int(self._nfc_robot_param.get_param_value(self.__ref_phone_position + "Y")) + int(antenna_position.split(",")[1]))

    # ----------------------------------------------------------------------------------------------

    def set_up(self):

        LabNfcBase.set_up(self)
        # dont need to check if BEAM activated because of the parameters
        # self._Device is DUT
        # self._phone_ref is the reference phone
        # disable "Tags" and browsers built-in apps

        # In DUT
        self._appmgmt_api.app_enable_disable(self._tag_app, False)
        self._appmgmt_api.app_enable_disable(self._browser_app, False)
        self._appmgmt_api.app_enable_disable(self._chrome_app, False)
        # in phone ref
        self._appmgmt_api.app_enable_disable(self._tag_app, False)
        self._appmgmt_api.app_enable_disable(self._browser_app, False)
        self._appmgmt_api.app_enable_disable(self._chrome_app, False)
        self.__size_of_file = 0
        # file have to be in both device, and get its size
        file_exist, result = self._file_api_dut.exist(posixpath.join(self._device.multimedia_path, self.__file_to_share))
        if file_exist:
            size = self._file_api_dut.size(posixpath.join(self._device.multimedia_path, self.__file_to_share))
            self.__size_of_file = size
        else:
            msg = "File to transfert missing on DUT "
            raise AcsConfigException(AcsConfigException.FILE_NOT_FOUND, msg)
        size = 0
        file_exist, result = self._file_api_ref.exist(posixpath.join(self._device.multimedia_path, self.__file_to_share))
        if file_exist:
            size = self._file_api_ref.size(posixpath.join(self._device.multimedia_path, self.__file_to_share))
            if size != self.__size_of_file:
                msg = "File to share are different size on DUT and reference phone"
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        else:
            msg = "File to transfert missing on reference phone "
            raise AcsConfigException(AcsConfigException.FILE_NOT_FOUND, msg)

        return Global.SUCCESS, "No errors"
    # ----------------------------------------------------------------------------------------------

    def run_test(self):

        msg = "No errors"

        # move robot to start position
        self._robot_positioning(self._tag_X, self._tag_Y, self._tag_up, 'null')

        # do the DUT to ref phone transfert
        if "SEND" or "FULL" in self.__direction:
            # starting beam sequence with DUT
            self.__beam_sequence(self._device)
            if not self.__is_transfert_ok(self._file_api_ref):
                msg = "Error : send BEAM failed on %s sequence" % self.__direction
        # clear beamed files
        self.__clear_beam_sequence()
        time.sleep(5)
        # do the ref phone to DUT transfert
        if "RECEIVE" or "FULL" in self.__direction:
            # starting beam sequence with phone ref
            self.__beam_sequence(self._phone_ref)
            if not self.__is_transfert_ok(self._file_api_dut):
                msg += "Error : receive BEAM failed on %s sequence" % self.__direction
        # clear beamed files
        self.__clear_beam_sequence()

        if msg != "No errors":
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return Global.SUCCESS, msg
    # ----------------------------------------------------------------------------------------------

    def tear_down(self):
        # move robot to start position
        self._robot_positioning(self._tag_X, self._tag_Y, self._tag_up, 'null')
        LabNfcBase.tear_down(self)

        return Global.SUCCESS, "No errors"
    # ----------------------------------------------------------------------------------------------

    def __is_transfert_ok(self, device_system_api):
        file_exist, result = device_system_api.exist(posixpath.join(self._beam_path, self.__file_to_share))
        if file_exist:
            size = device_system_api.size(posixpath.join(self._beam_path, self.__file_to_share))
            if size == self.__size_of_file:
                return True
            else:
                return False
    # ----------------------------------------------------------------------------------------------

    def __clear_beam_sequence(self):
        # clear beamed files on both devices
        self._file_api_dut.delete(posixpath.join(self._beam_path, self.__file_to_share))
        self._file_api_ref.delete(posixpath.join(self._beam_path, self.__file_to_share))
        return True
    # ----------------------------------------------------------------------------------------------

    def __open_image(self, device, file_to_open):
        cmd = "adb shell am start -d file:%s -t image/* -a android.intent.action.VIEW -n com.google.android.gallery3d/com.android.gallery3d.app.GalleryActivity" % str(file_to_open)
        device.run_cmd(cmd, 10)
        return True
    # ----------------------------------------------------------------------------------------------

    def __close_image(self, device):
        # close image
        cmd = "adb shell pm clear com.google.android.gallery3d"
        device.run_cmd(cmd, 10)
        time.sleep(1)
        return True
    # ----------------------------------------------------------------------------------------------

    def __beam_sequence(self, beamer):
        # first open photo on the beamer
        # DEBUG try to use an uecmd instead like start_display_image
        self._logger.info("Start to BEAM")
        self.__open_image(beamer, posixpath.join(beamer.multimedia_path, self.__file_to_share))
        # move nfc bot
        self._robot_positioning('null', 'null', self._tag_down, 'null')
        # wait a while
        time.sleep(2)

        if beamer.get_config("device_name") == "PHONE1":
            res = self._phonesystem_api_dut.get_screen_resolution().split("x")
            mres = "%dx%d" % (int(res[0])/2, int(res[1])/2)
            self._nfc_api.nfc_touch_to_beam(mres, 1)
        else:
            res = self._phonesystem_api_ref.get_screen_resolution().split("x")
            mres = "%dx%d" % (int(res[0])/2, int(res[1])/2)
            self._nfc_api_phone_ref.nfc_touch_to_beam(mres, 1)
        time.sleep(2)
        self._robot_positioning('null', 'null', self._tag_up, 'null')
        time.sleep(self.__beam_duration)
        # close gallery application
        self.__close_image(beamer)
