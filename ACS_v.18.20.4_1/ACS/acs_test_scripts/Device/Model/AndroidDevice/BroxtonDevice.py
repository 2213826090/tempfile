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
:summary: This file implements the Broxton platform.  It is mostly a
duplicate of the Cherrytrail implementation at the same time frame.
:since: 10/06/2015
:author: acpreble
"""

import time

from Device.Model.AndroidDevice.IntelDeviceBase import IntelDeviceBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Equipment.EquipmentManager import EquipmentManager

class BroxtonDevice(IntelDeviceBase):

    """
        Cherrytrail platform implementation
    """
    VIDEO_DECODER_ERROR_MESSAGES = [
         "DUMMY: This message should be replaced with an appropriate one"
        #"OMXVideoDecoder: Decoder returned DECODE_MEMORY_FAIL",
        #"OMXVideoDecoder: Decoder returned DECODE_FAIL",
        #"OMXVideoDecoder: Decoder returned DECODE_DRIVER_FAIL",
        #"VideoDecoder: endDecodingFrame failed. status",
        #"VideoDecoder: Reference frame 8 is missing for current frame 4"
    ]

    VIDEO_ENCODER_ERROR_MESSAGES = [
        "DUMMY: This message should be replaced with an appropriate one"
        #"OMXVideoEncoder: Failed to encode frame"
    ]

    GFX_ERROR_MESSAGES = [
        "DUMMY: This message should be replaced with an appropriate one"
        #"RGXSendCommandRaw failed to acquire CCB slot.",
        #"PVRSRV_ERROR_OUT_OF_MEMORY"
    ]

    VIDEO_ENHANCER_ERROR_MESSAGES = [
        "DUMMY: This message should be replaced with an appropriate one"
        #"pvr_drv_video: vsp_context_flush_cmdbuf fails with 'Bad address' at vendor/intel/hardware/psb_video/src/vsp_cmdbuf.c",
        #"KERNEL: [drm:vsp_submit_cmdbuf] *ERROR* The VSP is hang abnormally!",
        #"VPPWorker: vaEndPicture failed",
        #"VPPProcThread: process error",
        #"[drm:psb_fence_lockup] *ERROR* VSP timeout (probable lockup) detected, reset vsp",
        #"[drm:psb_fence_lockup] *ERROR* fence sequence is %d"
    ]

    CAMERA_HAL_START_MESSAGES = [
        "Camera_HAL: atom_preview_enabled"
    ]

    AUDIO_STREAMING_START_MESSAGES = [
        "AudioIntelHAL: startStream: output stream"
    ]

    VIDEO_DECODER_START_MESSAGES = [
        "DUMMY: This message should be replaced with an appropriate one"
        #"OMXCodec: [OMX.Intel.VideoDecoder.AVC] video dimensions are"
    ]

    GFX_START_MESSAGES = [
        "DUMMY: This message should be replaced with an appropriate one"
        #"OpenGLRenderer: Enabling debug mode 0"
    ]

    DISPLAY_ERROR_MESSAGES = [
        "DUMMY: This message should be replaced with an appropriate one"
        #"[drm:mdfld_dsi_send_dcs] *ERROR* DBI FIFO timeout, drop frame",
        #"[drm:psb_vsync_set_ioctl] *ERROR* fail to get vsync on pipe %d, ret %d",
        #"[drm:_Flip_Overlay] *ERROR* _Flip_Overlay: overlay %d is disabled"
    ]

    BLUETOOTH_ERROR_MESSAGES = [
        "DUMMY: This message should be replaced with an appropriate one"
        #"ERROR : timer_thread: tick delayed > %d slots (%d,%d) -- cpu overload"
    ]

    APP_START_MESSAGES = [
        "ActivityManager: Start proc {0} for activity {0}/.{1}"
    ]

    #1200 x 1920
    GOOGLE_CAMERA_TOUCHCMDS_12X19 = {
        'change_camera': [740, 1545],
        'select_settings': [1150, 1555],
        'select_mode': [600, 965],
        'select_video': [165, 1105],
        'select_imgae': [165, 965]
    }

    #1200 x 1920
    INTEL_CAMERA_TOUCHCMDS_12X19 = {
        'change_camera': [1135, 40],
        'select_mode': [1135, 1800],
        'select_video': [1105, 1545],
        'select_imgae': [1105, 1330],
        'select_burst': [1105, 1465],
    }

    # overrides boot modes, defined in parent class
    BOOT_STATES = {"MOS": "unknown",
                   "ROS": "recovery",
                   "COS": "charger"}

    def __init__(self, config, logger):
        """
        Constructor

        :type  phone_name: string
        :param phone_name: Name of the current phone(e.g. PHONE1)
        """

        IntelDeviceBase.__init__(self, config, logger)

        self._cellular_network_interface = self.get_config("cellularNetworkInterface", "rmnet0")

        self._em = EquipmentManager()
        iocard = self.get_config("IoCard", "IO_CARD", str)
        self._io_card = self._em.get_io_card(iocard)
        self._usb_recovery_ongoing = False
        self.prevent_usb_plug = 5

    def switch_on(self, boot_timeout=None, settledown_duration=None,
                  simple_switch_mode=False):
        """
        Switch ON the device
        This can be done either via the power supply
        or via IO card

        :param boot_timeout: Total time to wait for booting
        :type boot_timeout: int

        :param settledown_duration: Time to wait until start to count \
                                    for timeout,
                                    Period during which the device must \
                                    have started.
        :type settledown_duration: int

        :param simple_switch_mode: a C{boolean} indicating whether we want
        to perform a simple switch on.
        :type simple_switch_mode: bool

        :rtype: list
        :return: Output status and output log
        """
        self.prevent_usb_plug = 5 # Hack to avoid usb plug/unplug on the first get_boot_mode calls

        (return_code, return_message) = IntelDeviceBase.switch_on(self, boot_timeout,
                                                                  settledown_duration,
                                                                  simple_switch_mode)
        if return_code == Global.SUCCESS:
            if self._embedded_log:
                # Initialize embedded log mechanism for MODEM if required
                self._embedded_log.start("MODEM")

        return return_code, return_message

    def get_cellular_network_interface(self):
        """
        Return the ip interface of celluar network

        this interface can be obtain from the command "busybox ifconfig"
        :rtype: str
        :return: telephony ip interface
        """
        return self._cellular_network_interface

    def get_video_decoder_error_messages(self):
        """
        Get log messages for video decoder error
        :type None
        :param None
        :rtype : List
        :return : List of video decoder error messages
        """

        return self.VIDEO_DECODER_ERROR_MESSAGES

    def get_gfx_error_messages(self):
        """
        Get log messages for gfx error
        :type None
        :param None
        :rtype : List
        :return : List of video encoder error messages
        """

        return self.GFX_ERROR_MESSAGES

    def get_video_encoder_error_messages(self):
        """
        Get log messages for video encoder error
        :type None
        :param None
        :rtype : List
        :return : List of video encoder error messages
        """

        return self.VIDEO_ENCODER_ERROR_MESSAGES

    def get_video_enhancer_error_messages(self):
        """
        Get log messages for video enhancer error
        :type None
        :param None
        :rtype : List
        :return : List of video enhancer error messages
        """

        return self.VIDEO_ENHANCER_ERROR_MESSAGES

    def get_camera_start_messages(self):
        """
        Get log messages for camera start
        :type : None
        :param : None
        :rtype : List
        :return : List of camera start messages
        """

        return self.CAMERA_HAL_START_MESSAGES

    def get_audio_playback_start_messages(self):
        """
        Get log messages for audio playback start
        :type : None
        :param : None
        :rtype : List
        :return : List of audio playback start messages
        """

        return self.AUDIO_STREAMING_START_MESSAGES

    def get_video_decoder_start_messages(self):
        """
        Get log messages for video decoder start
        :type : None
        :param : None
        :rtype : List
        :return : List of video decoder start messages
        """

        return self.VIDEO_DECODER_START_MESSAGES

    def get_gfx_start_messages(self):
        """
        Get log messages for gfx start
        :type : None
        :param : None
        :rtype : List
        :return : List of gfx start messages
        """

        return self.GFX_START_MESSAGES

    def get_display_error_messages(self):
        """
        Get log messages for display error
        :type None
        :param None
        :rtype : List
        :return : List of display error messages
        """
        return self.DISPLAY_ERROR_MESSAGES

    def get_bluetooth_error_messages(self):
        """
        Get log messages for display error
        :type None
        :param None
        :rtype : List
        :return : List of bluetooth error messages
        """

        return self.BLUETOOTH_ERROR_MESSAGES

    def get_app_start_messages(self, package_name=None, app_name=None):
        """
        Get log messages for app start
        :type : package_name
        :param : Name of the package
        :type : app_name
        :param : Name of the app
        :rtype : List
        :return : List of app start messages
        """

        return_messages = []
        for start_msg in self.APP_START_MESSAGES:
            return_messages.append(start_msg.format(package_name, app_name))

        return return_messages

    def get_camera_touchcmds(self, target_name=None):
        """
        Get coordinates for camera app
        :type : target_name
        :param : Name of the target camera app
        :rtype : List
        :return : List of coordinates
        """

        _phone_system = self.get_uecmd("PhoneSystem")
        _phone_resolution = _phone_system.get_screen_resolution()
        _disp_width = _phone_resolution.split('x', 1)[0]
        _disp_height = _phone_resolution.split('x', 1)[1]

        _touchCmds = None
        _status = Global.SUCCESS

        if target_name == 'intel' and _disp_width == '1200':
            _touchCmds = self.INTEL_CAMERA_TOUCHCMDS_12X19
        elif target_name == 'google' and _disp_width == '1200':
            _touchCmds = self.GOOGLE_CAMERA_TOUCHCMDS_12X19
        else:
            self._logger.error('Camera API: Display resolution {0}x{1} is not supported in Broxton for now.'.format(_disp_width, _disp_height))
            _status = Global.FAILURE

        return _status, _touchCmds

    def usb_unplug(self):
        """
        unplug usb

        :rtype: bool
        :return: usb state : True if it could be unplugged, else False
        """
        if self._io_card:
            # Unplug USB
            self._io_card.usb_host_pc_connector(False)
            # Unplug wall charger only if it is AC_CHGR
            if self.get_default_wall_charger() == self._io_card.AC_CHGR:
                self._io_card.wall_charger_connector(False)
            return True
        return False

    def usb_plug(self):
        """
        plug usb

        :rtype: bool
        :return: usb state : True if it could be plugged, else False
        """
        if self._io_card:
            # Plug wall charger only if it is AC_CHGR
            if self.get_default_wall_charger() == self._io_card.AC_CHGR:
                # Plug wall charger
                self._io_card.wall_charger_connector(True)
            # Plug USB
            self._io_card.usb_host_pc_connector(True)
            return True
        return False

    def get_boot_mode(self):
        """
        get the boot mode from adb

        :rtype: string
        :return: device state : MOS, ROS, POS, DNX, COS or UNKNOWN
        """

        # Hack to avoid usb unplug/replug at the beginning of boot
        if self.prevent_usb_plug > 0:
            self.prevent_usb_plug -= 1
            time.sleep(5)
            return "UNKNOWN"

        return IntelDeviceBase.get_boot_mode(self)

    def _wait_board_is_ready(self, boot_timeout=None, settledown_duration=None):
        """
        Wait until device is ready to be used

        :type boot_timeout: int
        :param boot_timeout: max time in seconds to wait for device

        :type settledown_duration: int
        :param settledown_duration: fixed time waited if requested after boot procedure

        :rtype: int
        :return: Device status - ready to be used (boot procedure OK) or NOT (Global.SUCCESS, Global.FAILURE)

        :rtype: str
        :return: error message
        """
        if boot_timeout is None:
            # Timeout for total boot duration
            boot_timeout = self.get_boot_timeout()

        # By default we do wait settledown_duration after the boot procedure only if required by the user
        # if user needs a settledown duration after boot procedure it shall be set to a value (not None)

        # Loop while boot time not exceeded & read device state
        timer = 0
        status = Global.FAILURE
        return_code = Global.FAILURE
        return_message = ""

        usbReplugRetries = self.get_config("usbReplugRetries", 0, int)
        if usbReplugRetries:
            wait_device_timeout = boot_timeout / (usbReplugRetries+1)
            if wait_device_timeout > 20:
                wait_device_timeout -= 10 # for the sleep between unplug and replug
        else:
            wait_device_timeout = boot_timeout

        while status == Global.FAILURE and timer < boot_timeout:
            # pylint: disable=W0702
            # at this method, the device is not yet available
            # we can have unexpected behavior on run_cmd
            # agree to catch all exception
            t_0 = time.time()
            try:
                # Break when Global.SUCCESS

                if not self._use_adb_over_ethernet:
                    status, status_msg = self.run_cmd("adb wait-for-device", wait_device_timeout, force_execution=True)
                    # Check output status and send debug log
                    if status == Global.SUCCESS:
                        self.get_logger().info("Device booted")
                    else:
                        msg = "No boot mode state returned"
                        if status_msg is not None:
                            msg = msg + ", error message: %s" % str(status_msg)
                        self.get_logger().debug(msg)

                        if usbReplugRetries:
                            self.recover_usb(usbReplugRetries)

                # FIXME: the adb connection should be handle outside the check, because this method should only
                # check availability as its name indicate
                elif self._check_ethernet_connection_is_available():
                    status = Global.SUCCESS

            except (KeyboardInterrupt, SystemExit):
                raise
            except Exception as error:
                # Inform user of the exception
                self.get_logger().debug("Exception while booting the device : %s", str(error))
                # Wait 1 seconds to avoid relaunching the command multiple times
                time.sleep(1)

            t_1 = time.time()
            timer += t_1 - t_0

        if timer < boot_timeout and status == Global.SUCCESS:
            return_code, return_message = self._finalize_os_boot(timer, boot_timeout, settledown_duration)
        else:
            # Device still not booted
            # Device not connected or adb connection issue
            return_message = "Device has failed to boot after %d seconds! " % boot_timeout
            return_message += "Device not detected or adb connection issue"
            self.get_logger().warning(return_message)

        return return_code, return_message


    def reboot(self, mode="MOS", wait_for_transition=True,
               transition_timeout=None, skip_failure=False,
               wait_settledown_duration=False):
        """
        Perform a SOFTWARE reboot on the device.
        By default will bring you to MOS and connect acs once MOS is seen.
        this reboot require that you are in a state where adb or fastboot command can be run.

        :type mode: str or list
        :param mode: mode to reboot in, support MOS, COS, POS, ROS. It can be a list of these modes
               (ie ("COS","MOS"))
               .. warning:: it is not always possible to reboot in a mode from another mode
                eg: not possible to switch from ROS to COS

        :type wait_for_transition: bool
        :param wait_for_transition: if set to true,
                                    it will wait until the wanted mode is reached

        :type transition_timeout: int
        :param transition_timeout: timeout for reaching the wanted mode
                                    by default will be equal to boot timeout set on
                                    device catalog

        :type skip_failure: bool
        :param skip_failure: skip the failure, avoiding raising exception, using
                                this option block the returned value when it is equal to False

        :type wait_settledown_duration: bool
        :param wait_settledown_duration: if set to True, it will wait settleDownDuration seconds
                                          after reboot for Main OS only.

        :rtype: bool
        :return: return True if reboot action succeed depending of the option used, False otherwise
                 - if wait_for_transition not used, it will return True if the reboot action has been seen
                   by the device
                 - if wait_for_transition used , it will return True if the reboot action has been seen
                   by the device and the wanted reboot mode reached.
        """

        # TODO : REMOVEME
        # This is a workaround for BXT because the device is too unstable now
        # So we do not run adb reboot, but we force shutdown with USB unplugged and then press the power button

        if mode == "MOS":
            self.hard_shutdown(wait_for_board_off=False)
            return self.switch_on(simple_switch_mode=True)[0] == Global.SUCCESS
        else:
            return IntelDeviceBase.reboot(self, mode, wait_for_transition, transition_timeout, skip_failure, wait_settledown_duration)

    def enable_adb_root(self):
        """
        Switch adb to adb root
        :rtype: boolean
        :return: true if root is successfully set, false otherwise
        """

        usbReplugRetries = self.get_config("usbReplugRetries", 0, int)

        self.get_logger().info("Adb root requested, enabling it ...")
        end_time = time.time() + self._adb_root_timeout

        result, output = self.run_cmd("adb root", self._adb_root_cmd_timeout, force_execution=True)
        while time.time() < end_time and output.find("already running as root") == -1:
            time.sleep(1)
            if usbReplugRetries and result != Global.SUCCESS:
                self._recover_usb_no_connect(1)
            if self._use_adb_over_ethernet:
                # reconnect device, as "adb root" reset adb socket
                self._phone_handle.adb_ethernet_start(self._ip_address, self._adb_port,
                                                      self._adb_connect_retries_nb, self._adb_connect_timeout)
            result, output = self.run_cmd("adb root", self._adb_root_cmd_timeout, force_execution=True)

        # adb root should be enabled, now we wait for some time and then try to recover usb if needed
        time.sleep(1)
        self._recover_usb_no_connect(1)

        return result == Global.SUCCESS and output.find("already running as root") != -1

    def _run_adb_cmd(self, cmd, timeout, silent_mode=False, wait_for_response=True, cancel=None):
        """
        Execute the input adb command and return the result message
        If the timeout is reached, return an exception

        :type  cmd: string
        :param cmd: cmd to be run
        :type  timeout: integer
        :param timeout: Script execution timeout in sec
        :type  cancel: Cancel
        :param cancel: a Cancel object that can be used to stop execution, before completion or timeout(default None)

        :return: Execution status & output string
        :rtype: Integer & String
        """
        result, msg = IntelDeviceBase._run_adb_cmd(self, cmd, timeout, silent_mode, wait_for_response, cancel)

        # if both the command and adb wait-for-device fail, try to unplug/replug usb several times
        # and try the command again if successful
        if result != Global.SUCCESS:
            usbReplugRetries = self.get_config("usbReplugRetries", 0, int)
            if usbReplugRetries and self.recover_usb(usbReplugRetries):
                result, msg = IntelDeviceBase._run_adb_cmd(self, cmd, timeout, silent_mode, wait_for_response, cancel)
        return result, msg

    def _connect_adb(self, tcp_connect_retries_nb=None, adb_connect_retries_nb=None):
        """
        Connect ADB though USB or ETHERNET, ensure adb root connection if set
        :param tcp_connection_retry_nb: allow to override retry nb
        :param adb_connection_retry_nb:  allow to override retry nb
        :rtype: bool, str
        :return: status of ADB connection and status message
        """

        usbReplugRetries = self.get_config("usbReplugRetries", 0, int)
        if usbReplugRetries:
            self.recover_usb(usbReplugRetries)

        return IntelDeviceBase._connect_adb(self, tcp_connect_retries_nb, adb_connect_retries_nb)

    def _recover_usb_no_connect(self, retries=1):
        """
        Try to recover usb by unpluging and repluging it
        and then return the status of last wait-for-device command
        :rtype: str
        :return: status of wait-for-device command
        """
        try_nb = retries
        status, _ = self.run_cmd("adb wait-for-device", 3, force_execution=True)
        while try_nb > 0 and status != Global.SUCCESS:
            try_nb -= 1
            if self.usb_unplug():
                time.sleep(5)
            if self.usb_plug():
                status, _ = self.run_cmd("adb wait-for-device", 10, force_execution=True)
        return status

    def recover_usb(self, retries=1):
        """
        Try to recover usb by unpluging and repluging it
        and then return usb status (based on adb wait-for-device
        :param tcp_connection_retry_nb: allow to override retry nb
        :param adb_connection_retry_nb:  allow to override retry nb
        :rtype: bool, str
        :return: status of ADB connection and status message
        """
        # If we are already trying to recover usb, do nothing (deadock prevention)
        if self._usb_recovery_ongoing:
            return False

        # If usb plug is prevented, return after 10 seconds
        if self.prevent_usb_plug > 0:
            self.prevent_usb_plug -= 1
            time.sleep(10)
            return False

        self._usb_recovery_ongoing = True # lock
        status = self._recover_usb_no_connect(retries)

        # if the device was recovered but still not reconnected to fwk, reconnect it
        if status == Global.SUCCESS and not self.is_available() and not self._connection_lock.locked():
            self.connect_board()
        self._usb_recovery_ongoing = False # release the lock
        return status == Global.SUCCESS

    def soft_shutdown(self, wait_for_board_off=False):
        """"
        Perform a soft shutdown and wait for the device is off

        :type wait_for_board_off: boolean
        :param wait_for_board_off: Wait for device is off or not after soft shutdown

        :rtype: boolean
        :return: If device is off or not
        """

        # Soft shutdown does not work on BXT yet
        return self.hard_shutdown(False)
