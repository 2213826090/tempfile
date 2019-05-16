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

:organization: INTEL NDG
:summary: This file implements the Miscellaneous UEcmd for Linux devices
:since: 17 jul 2014
:author: floeselx
"""
from datetime import datetime

from acs_test_scripts.Device.UECmd.Imp.Linux.Common.Base import Base
from acs_test_scripts.Device.UECmd.Interface.Misc.IPhoneSystem import IPhoneSystem
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
import UtilitiesFWK.Utilities as Util


class PhoneSystem(Base, IPhoneSystem):
    """
    :summary: PhoneSystem UEcommands operations for Linux platforms
    """

    def __init__(self, phone):
        """
        Constructor.

        """
        Base.__init__(self, phone)
        IPhoneSystem.__init__(self, phone)

        self._logger = phone.get_logger()

    def mount_device(self, device, folder, behavior=None, behavior_value=None):
        """
        Mounts given device into folder (create it if it doesnt exist).

        :type device: str
        :param device: path of the device to mount , often on /dev
        :type folder: str
        :param folder: folder where to mount the device , eg : /dev/mnt

        :type behavior: str
        :param behavior: change the behavior of the method :
                         - "scheduled" to schedule the method execution
                         - "read" to read the output of a scheduled method

        :type behavior_value: int
        :param behavior_value: the value link to the behavior:
                         - time to wait for "scheduled" option
                         - pid given by a former scheduled method execution

        :rtype: str
        :return: Pid of a scheduled operation or None
        """
        self._logger.info("mount device partition %s on mount point %s..." % (device, folder))

        # Check validity of given parameters
        device_file_exist = self.check_file_exist(device)
        mount_point_exist = self.check_file_exist(folder)

        # If device file does not exist, must raise exception
        if not device_file_exist:
            msg = "Device %s does not exist !" % str(device)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        if not mount_point_exist:
            msg = "Mount point does not exist, create it before mounting ..."
            self._logger.warning(msg)
            cmd = "mkdir -p %s" % folder
            self._internal_exec(cmd, timeout=5)

        # Now mount partition
        cmd = "mount %s %s" % (device, folder)
        status, output = self._internal_exec(cmd, raise_exception=False)

        if not status:
            f = folder if not folder.endswith("/") else folder[:-1]
            if "already mounted on {0}".format(f) in output:
                self._logger.info("Device {0} already mounted on {1}".format(device, folder))
                return

            msg = "Unable to mount partition {0} on {1} ({2})".format(device, folder, str(output))
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def unmount_device(self, device, behavior=None, behavior_value=None):
        """
        Unmounts given device.

        :type device: str
        :param device: path of the device to unmount
                       or path of a folder where has been moutn a device

        :type behavior: str
        :param behavior: change the behavior of the method :
                         - "scheduled" to schedule the method execution
                         - "read" to read the output of a scheduled method

        :type behavior_value: int
        :param behavior_value: the value link to the behavior:
                         - time to wait for "scheduled" option
                         - pid given by a former scheduled method execution

        :rtype: str
        :return: Pid of a scheduled operation or None
        """
        self._logger.debug("Umounting %s ..." % (device))

        # Unmount partition
        cmd = "umount %s" % device
        status, output = self._internal_exec(cmd)
        if not status:
            msg = "Unable to umount %s (%s)" % (str(device), str(output))
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return status, output

    def check_file_exist(self, file_path):
        """
        Check if the file exists on the target file system

        :type file_path: str
        :param file_path: Full path of the file to be checked

        :return: boolean
        """
        status = True
        self._logger.info("Checking that file %s exists" % file_path)
        cmd = "file %s" % file_path
        status, output = self._internal_exec(cmd, raise_exception=False)

        if status:
            # uart verdict (always true) or ssh ok
            if "no such file or directory" in str(output).lower():
                self._logger.warning("File {0} does not exists !".format(file_path))
                status = False
            else:
                self._logger.info("File {0} exists !".format(file_path))
        elif not status and "no such file or directory" in str(output).lower():
            # ssh verdict
            self._logger.warning("File {0} does not exists !".format(file_path))
            status = False
        else:
            # ssh failed for other reason
            msg = "Unable to determine status of file {0} ({1})".format(file_path, str(output))
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, msg)

        return status

    def delete(self, filename, raise_exception=True):
        """
        Deletes the file.

        :type filename: str
        :param filename: file to delete

        :type raise_exception_on_failure: Boolean
        :param raise_exception_on_failure: Raise exception on failure or not

        :rtype: boolean
        :return: True if the file has been well deleted, False otherwise
        """
        self._logger.info("delete file: %s " % filename)

        cmd = "rm -f %s " % filename
        status, output = self._internal_exec(cmd, raise_exception=raise_exception)

        return status

    def display_on(self):
        """
        set phone screen on
        :param None
        :rtype: None
        """
        self._logger.debug("display_on: Not used on Linux devices")

    def display_off(self):
        """
        Try to set phone screen off
        :param None
        :rtype: None
        """

        self._logger.debug("display_off: Not used on Linux devices")

    def set_screen_timeout(self, timeout):
        """
        Sets the screen timeout.

        :type timeout: int
        :param timeout: the screen timeout in seconds. Use 0 to set maximum delay

        :return: None
        """

        self._logger.debug("set_screen_timeout: Not used on Linux devices")

    def get_screen_timeout(self):
        """
        Gets the screen timeout.

        :rtype: int
        :return: screen timeout in seconds
        """

        self._logger.debug("get_screen_timeout: Not used on Linux devices")
        return -1

    def get_rtc_mode(self):
        """
        Gets the rtc mode parameter status on device

        :rtype: bool
        :return: If rtc is in local (true) or utc (false)
        """
        return_code, msg = self._internal_exec("timedatectl status | grep \"RTC in local TZ:\"")
        if return_code == Util.Global.FAILURE or msg == "":
            raise DeviceException(DeviceException.OPERATION_FAILED, "Unable to get the RTC mode of the DUT: {0}".format(msg))

        result = msg.split(':')[1].rstrip()
        if result == "yes":
            # RTC in local TZ
            return True
        else:
            return False

    def set_rtc_mode(self, mode):
        """
        Sets the rtc mode parameter status on device

        :type mode: int
        :param mode: The mode in what the rtc mode to be set (0 utc|1 local)

        :rtype: None
        """
        return_code, msg = self._internal_exec("timedatectl set-local-rtc {0}".format(mode))
        if return_code == Util.Global.FAILURE:
            raise DeviceException(DeviceException.OPERATION_FAILED, "Unable to set the RTC mode of the DUT: {0}".format(msg))

    def get_auto_time(self):
        """
        Gets the auto time parameter status on device

        :rtype: bool
        :return: If parameter is set or not.
        """
        self._logger.info("Get Auto Time value from dut ...")

        status, output = self._internal_exec(cmd="systemctl is-active systemd-timesyncd", raise_exception=False)

        self._logger.debug("Auto Time value from dut is: %s"
                           % output)

        if output == "active":
            return True
        else:
            return False

    def set_auto_time(self, mode):
        """
        Sets the auto time parameter status on device

        :type mode: int
        :param mode: The mode in what the auto time to be set (0|1)

        :rtype: None
        """
        if mode in ("active", "on", "1", 1):
            mode = 1
            self._logger.info("Trying to set Auto Time ON")
            self._internal_exec(cmd="timedatectl set-ntp true")

        elif mode in ("inactive", "off", "0", 0):
            mode = 0
            self._logger.info("Trying to set Auto Time OFF")
            self._internal_exec(cmd="timedatectl set-ntp false")

        else:
            self._logger.error("set_auto_time: Parameter mode %s is not valid" % str(mode))
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "mode is not valid !")

        self._logger.info("set Auto Time value to {0} on dut ...".format(mode))

    def get_current_time(self):
        """
        Get the current date and time, readably formatted

        :rtype: tuple of str
        :return: A tuple containing date and time (y, m, d, h, m, s)
        """
        self._logger.info("Getting current time on DUT")

        cmd = "date '+%Y.%m.%d.%H.%M.%S'"

        self._logger.info("Getting the current date")
        (_result, output) = self._internal_exec(cmd, 10)

        date = output.split(".")
        if len(date) < 6:
            raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR,
                                  "Error during date retrieval ({0})".format(output))
        datetime = tuple(date)[:6]
        self._logger.debug("Current DUT time is: {0}".format(str(datetime)))

        return datetime

    def configure_time(self, year="1970", month="01", day="01",
                       hour="00", minutes="00", seconds="00"):
        """
        Configure the DUT date and time with given values.

        :type year: str
        :param year: The year to be set on DUT

        :type month: str
        :param month: The month to be set on DUT

        :type day: str
        :param day: The day to be set on DUT

        :type hour: str
        :param hour: The hour to be set on DUT

        :type minutes: str
        :param minutes: The minutes to be set on DUT

        :type seconds: str
        :param seconds: The seconds to be set on DUT

        :rtype: None
        """
        structure_datetime = datetime(int(year), int(month), int(day), int(hour), int(minutes), int(seconds))
        self._device.get_uecmd("System").set_date_and_time(structure_datetime)

    def set_stay_on_while_plugged_in(self, value, use_agent=True):
        """
        Set the value for android to stay awake when plugged to host.
        :type value: int or str
        :param value: the value to set (between 0 to 3) or in (true|false|usb|ac|wireless)
        :type use_agent: bool
        :param use_agent: tell if command will use android api or call adb
        """
        self._logger.debug("set_stay_on_while_plugged_in: Not used on Linux devices")

    def set_phone_screen_lock_on(self, lock_on):
        """
        set phone screen lock on
        :type lock_on: integer
        :param lock_on: phone screen light state in idle mode. Can be:
        -{0}: unlocked display always ON
        -{1}: lock display ON
        :rtype: None
        """
        self._logger.debug("set_phone_screen_lock_on: Not used on Linux devices")

    def home_page_menu(self):
        """
        Go to home page
        """
        self._logger.debug("home_page_menu: Not used on Linux devices")

    def wake_screen(self):
        """
        Wakes up the phone screen

        :return: None
        """
        self._logger.debug("wake_screen: Not used on Linux devices")

    def set_phone_lock(self, mode):
        """
        When phone is in idle mode, force the phone to stay locked/unlocked.

        :type mode: int
        :param mode: phone lock state in idle mode. Can be:
            -{0}: unlocked
            -{1}: lock
        """
        self._logger.debug("wake_screen: Not used on Linux devices")

    def set_display_brightness(self, brightness):
        """
        Sets the display brightness
        :type: brightness: int
        :param brightness: brightness to set, in percentage from 0 to 100

        :rtype: list
        :return: operation status & output log
        """
        self._logger.debug("set_display_brightness: No display on Linux devices")
