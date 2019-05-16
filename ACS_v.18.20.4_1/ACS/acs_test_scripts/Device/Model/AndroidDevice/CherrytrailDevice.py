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
:summary: This file implements the Cherrytrail platform
:since: 27/11/2013
:author: cbonnard
"""

from Device.Model.AndroidDevice.IntelDeviceBase import IntelDeviceBase
from UtilitiesFWK.Utilities import Global


class CherrytrailDevice(IntelDeviceBase):

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

    #Button coordinates for Google Camera app on 1200 x 1920 resolution.
    #This dictionary is referred by get_camera_touchcmds which is called from Device/UECmd/Imp/Android/LLP/Multimedia/Camera.py
    #toggle_burst_mode and change_camera in the Camera UEcmd require these coordinates
    GOOGLE_CAMERA_TOUCHCMDS_12X19 = {
        'change_camera': [740, 1545],
        'select_settings': [1150, 1570],
        'select_mode': [600, 965],
        'select_video': [165, 1105],
        'select_imgae': [165, 965]
    }

    #Button coordinates for Intel Camera app on 1200 x 1920 resolution.
    #This dictionary is referred by get_camera_touchcmds which is called from Device/UECmd/Imp/Android/LLP/Multimedia/Camera.py
    #toggle_burst_mode and change_camera in the Camera UEcmd require these coordinates
    INTEL_CAMERA_TOUCHCMDS_12X19 = {
        'change_camera': [1135, 40],
        'select_mode': [1135, 1800],
        'select_video': [1105, 1545],
        'select_imgae': [1105, 1330],
        'select_burst': [1105, 1465],
        'take_picture': [605, 1770]
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
        This function is called from Device/UECmd/Imp/Android/LLP/Multimedia/Camera.py
        toggle_burst_mode and change_camera in the Camera UEcmd require this function
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
            self._logger.error('Camera API: Display resolution {0}x{1} is not supported in Cherrytrail for now.'.format(_disp_width, _disp_height))
            _status = Global.FAILURE

        return _status, _touchCmds

    def get_socperf_driver(self):
        """
        Get socperf driver name. This required for SOCWatch
        :rtype : String
        :return : socperf driver name
        """
        return "socperf1_2.ko"

    def get_socwatch_driver(self):
        """
        Get socwatch driver name. This required for SOCWatch
        :rtype : String
        :return : socwatch driver name
        """
        output = ""
        cmd = "adb shell cd /lib/modules/; stat -c %n * | grep socwatch"
        (status, output) = self.run_cmd(cmd, 2)
        self._logger.info("output : %s " % output)
        if output == "":
            self._logger.error("get_socwatch_driver: socwatch driver not found in /lib/modules/")
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, "socwatch driver not found")
        return output
