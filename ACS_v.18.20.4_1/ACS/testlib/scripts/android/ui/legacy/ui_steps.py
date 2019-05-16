

class click_button_with_scroll2(ui_step):
    """ description:
            clicks on a view (view_to_find) scrolling for it.
            Unlike the original method, it does not look/set scrollable=True and
            it searches for a ListView, not ScrollView with the property focused=True.
            This property is a double check for the situations in which background ListView
            objects are still visible for UiAutomator.
        usage:
            ui_steps.click_button_with_scroll2(
                view_to_find = {"text": "Apps"},
                view_to_check = {"text": "Downloaded"})()

        tags:
            ui, android, press, click, button, settings
    """

    def __init__(self, view_to_find, view_to_check=None, view_presence=True,
                 **kwargs):
        ui_step.__init__(self, **kwargs)
        self.view_to_find = view_to_find
        self.view_to_check = view_to_check
        self.view_presence = view_presence
        self.set_passm(str(view_to_find) + " checking " + str(view_to_check))
        self.set_errorm("", str(view_to_find) + " checking " + str(view_to_check))

    def do(self):
        if self.uidevice(className="android.widget.ListView", focused=True).exists:
            self.uidevice(className="android.widget.ListView", focused=True).scroll.to(**self.view_to_find)
        self.uidevice(**self.view_to_find).click()

    def check_condition(self):
        if self.view_to_check is None:
            return True
        return self.uidevice(**self.view_to_check).exists


class FolderUtilityVariables(object):
    """description:
        auxiliary class for folder creation variables
    """
    SEND_BT_PICS_NEW_FOLDER = 'bluetooth_send_pic'
    DEFAULT_SAVE_LOCATION = 'sdcard/'


class open_picture_from_photos(ui_step, adb_step):
    """ description:
            open picture from gallery with an intent

        usage:
            adb_steps.open_picture_from_photos(serial=dut_serial,
                                                photo_name=photo_name,
                                                photo_path=photo_path_on_device)()

        tags:
            ui, android, press, click, picture, photos
    """
    _PHOTOS_START_COMMAND = 'am start -t image/* -d file:///'
    _PHOTOS_HOME_ACTIVITY = ' -n com.google.android.apps.plus/com.google.android.apps.photos.phone.PhotosLauncherActivity'
    _DEFAULT_SAVE_LOCATION = 'sdcard/'
    _FOLDER_TO_USE = FolderUtilityVariables.DEFAULT_SAVE_LOCATION + FolderUtilityVariables.SEND_BT_PICS_NEW_FOLDER

    def __init__(self, photo_path, photo_name, view_to_check=None, **kwargs):
        ui_step.__init__(self, **kwargs)
        adb_step.__init__(self, **kwargs)
        self.view_to_check = view_to_check
        self.photo_path = photo_path
        self.photo_name = photo_name

    def do(self):
        grep_for_string = 'Starting: Intent'
        launch_command = ' am start ' + self._PHOTOS_HOME_ACTIVITY
        makedir_cmd = 'mkdir ' + self._FOLDER_TO_USE
        copy_cmd = 'cp ' + self.photo_path + self.photo_name + ' ' + self._FOLDER_TO_USE
        refresh_media_cmd = 'am broadcast -a android.intent.action.MEDIA_MOUNTED -d file:///' + FolderUtilityVariables.SEND_BT_PICS_NEW_FOLDER

        #make a new folder, move our wanted picture there
        self.adb_connection.parse_cmd_output(
            cmd=makedir_cmd
        )

        self.adb_connection.parse_cmd_output(
            cmd=copy_cmd
        )

        #perform media refresh
        self.adb_connection.parse_cmd_output(
            cmd=refresh_media_cmd
        )

        #launch the Photos activity to its home
        self.adb_connection.parse_cmd_output(
            cmd=launch_command,
            grep_for=grep_for_string
        )

        #dismiss potential setup wizard
        if self.uidevice(text="Sign in").wait.exists(timeout=1000):
            click_button(serial=self.serial,
                         view_to_find=dict(text='No thanks'),
                         view_to_check=dict(descriptionContains='Open navigation drawer'))()
        else:
            pass

        #dismiss a potential connection error pop-up message
        if self.uidevice(text="Couldn't connect to the server.").wait.exists(timeout=1000):
            click_button(serial=self.serial,
                         view_to_find=dict(text='Cancel'),
                         view_to_check=dict(descriptionContains='Open navigation drawer'))()
        else:
            pass

        #select 'On device" photos
        click_button(serial=self.serial,
                     view_to_find=dict(descriptionContains='Open navigation drawer'),
                     view_to_check=dict(descriptionContains='Close navigation drawer'))()

        click_button_with_scroll2(serial=self.serial,
                                  view_to_find=dict(text='On device'),
                                  view_to_check=dict(resourceId='com.google.android.apps.plus:id/tiles'))()

        click_button(serial=self.serial,
                     view_to_find=dict(text=FolderUtilityVariables.SEND_BT_PICS_NEW_FOLDER),
                     view_to_check=dict(resourceId='com.google.android.apps.plus:id/grid'))()

        click_button(serial=self.serial,
                     view_to_find=dict(longClickable='true', index='0'),
                     view_to_check=dict(descriptionContains='Share'))()

    def check_condition(self):
        return self.uidevice(descriptionContains='Share').wait.exists(timeout=1000)


class ForceStopPhotosApp(ui_step, adb_step):
    """description:
        this class force stops the Photos application
    """
    _PHOTOS_PACKAGE_NAME = ' com.google.android.apps.plus'

    def __init__(self, **kwargs):
        ui_step.__init__(self, **kwargs)
        adb_step.__init__(self, **kwargs)

    def do(self):
        force_stop_cmd = "am force-stop " + self._PHOTOS_PACKAGE_NAME
        self.adb_connection.parse_cmd_output(cmd=force_stop_cmd)


class RemoveBluetoothAuxiliaryFolder(ui_step, adb_step):
    """ description:
            remove the auxiliary file created for opening a single photo for bluetooth opp transfers

        usage:
            adb_steps.RemoveBluetoothAuxiliaryFolder(serial=dut_serial)()

        tags:
            ui, android, press, click, picture, photos
    """
    _FOLDER_TO_REMOVE = FolderUtilityVariables.DEFAULT_SAVE_LOCATION + FolderUtilityVariables.SEND_BT_PICS_NEW_FOLDER

    def __init__(self, **kwargs):
        ui_step.__init__(self, **kwargs)
        adb_step.__init__(self, **kwargs)

    def do(self):
        remove_command = " rm -rf " + self._FOLDER_TO_REMOVE

        #run command
        self.adb_connection.parse_cmd_output(cmd=remove_command)

    def check_condition(self):
        check_string = 'No such file or directory'
        check_cmd = ' ls ' + self._FOLDER_TO_REMOVE
        return self.adb_connection.parse_cmd_output(
            cmd=check_cmd,
            grep_for=check_string)


class LaunchCameraIntent(ui_step, adb_step):
    """ description:
            This class launches the GMS Camera app on the device

        usage:
            ui_steps.LaunchCameraActivity(serial=dut_serial)()

        tags:
            ui, android, press, click, picture, photos
    """

    def __init__(self, **kwargs):
        ui_step.__init__(self, **kwargs)
        adb_step.__init__(self, **kwargs)

    def do(self):
        launch_camera_app = 'am start -a android.intent.action.MAIN -n com.google.android.GoogleCamera/com.android.camera.CameraActivity'
        grep_for_string = 'Starting: Intent'
        self.adb_connection.parse_cmd_output(
            cmd=launch_camera_app,
            grep_for=grep_for_string
        )

        if self.uidevice(text='NEXT').wait.exists(timeout=1000):
            click_button(serial=self.serial,
                         view_to_find=dict(text='NEXT'),
                         view_to_check=dict(descriptionContains='Shutter'))()



class LaunchCameraActivity(ui_step, adb_step):
    """ description:
            This class launches the device into picture taking mode

        usage:
            ui_steps.LaunchCameraActivity(serial=dut_serial)()

        tags:
            ui, android, press, click, picture, photos
    """

    def __init__(self, **kwargs):
        ui_step.__init__(self, **kwargs)
        adb_step.__init__(self, **kwargs)
        self.y_center = self.uidevice.info['displayHeight'] / 2

    def do(self):

        LaunchCameraIntent(serial=self.serial)()

        swipe(serial=self.serial,
              sx=0,
              sy=self.y_center,
              ex=self.uidevice.info['displayWidth'] - 5,
              ey=self.y_center * 130 / 100)()

        # click_button(serial=self.serial,
        #              view_to_find=dict(text='Video'),
        #              view_to_check=dict(descriptionContains='Options'))()

        long_click(serial=self.serial,
                     view_to_find=dict(text='Camera'),
                     view_to_check=dict(resourceId='com.android.camera2:id/shutter_button'))()

        launch_video_cmd = 'am start -a android.media.action.IMAGE_CAPTURE'
        grep_for_string = 'Starting: Intent'
        self.adb_connection.parse_cmd_output(
            cmd=launch_video_cmd,
            grep_for=grep_for_string
        )

        if self.uidevice(text='NEXT').wait.exists(timeout=1000):
            click_button(serial=self.serial,
                         view_to_find=dict(text='NEXT'),
                         view_to_check=dict(descriptionContains='Shutter'))()


    def check_condition(self):
        return self.uidevice(resourceId='com.android.camera2:id/shutter_button').wait.exists(timeout=1000)


class LaunchVideoActivity(ui_step, adb_step):
    """ description:
            This class launches the device into video recording mode

        usage:
            ui_steps.LaunchVideoActivity(serial=dut_serial)()

        tags:
            ui, android, press, click, picture, photos
    """

    def __init__(self, **kwargs):
        ui_step.__init__(self, **kwargs)
        adb_step.__init__(self, **kwargs)
        self.y_center = self.uidevice.info['displayHeight'] / 2

    def do(self):
        LaunchCameraIntent(serial=self.serial)()

        swipe(serial=self.serial,
              sx=0,
              sy=self.y_center,
              ex=self.uidevice.info['displayWidth'] - 5,
              ey=self.y_center * 130 / 100)()

        # click_button(serial=self.serial,
        #              view_to_find=dict(text='Video'),
        #              view_to_check=dict(descriptionContains='Options'))()

        long_click(serial=self.serial,
                     view_to_find=dict(text='Video'),
                     view_to_check=dict(resourceId='com.android.camera2:id/shutter_button'))()


    def check_condition(self):
        return self.uidevice(resourceId='com.android.camera2:id/shutter_button').wait.exists(timeout=1000)


class RecordVideo(ui_step, adb_step):
    """ description:
            Record a video file with a specified duration

        usage:
            ui_steps.RecordVideo(serial=dut_serial, record_time=RECORDING_TIME)()

        tags:
            ui, android, press, click, picture, photos
    """

    def __init__(self, record_time, **kwargs):
        ui_step.__init__(self, **kwargs)
        adb_step.__init__(self, **kwargs)
        self.record_time = record_time

    def do(self):
        #launching the video activity
        LaunchVideoActivity(serial=self.serial)()

        #start recording:
        print "Start recording"
        record_command = 'input keyevent KEYCODE_CAMERA'
        self.adb_connection.parse_cmd_output(
            cmd=record_command
        )
        # click_button(serial=self.serial,
        #              view_to_find=dict(descriptionContains='Shutter'),
        #              view_to_check=dict(resourceId='com.android.camera2:id/recording_time'))()

        #wait for the specified time in self.record_time
        time.sleep(self.record_time)

        #stop recording
        # click_button(serial=self.serial,
        #              view_to_find=dict(descriptionContains='Shutter'),
        #              view_to_check=dict(descriptionContains='Options'))()

        long_click(serial=self.serial,
                     view_to_find=dict(descriptionContains='Shutter'),
                     view_to_check=dict(descriptionContains='Options'))()

        #if an approval prompt appears, dismiss is by approving
        if self.uidevice(description='Done').wait.exists(timeout=1000):
            click_button(serial=self.serial,
                         view_to_find=dict(resourceId='com.android.camera2:id/done_button'),
                         view_to_check=dict(descriptionContains='Shutter'))()


    def check_condition(self):
        return self.uidevice(resourceId='com.android.camera2:id/shutter_button').wait.exists(timeout=1000)


class OpenRecordedVideo(ui_step, adb_step):
    """ description:
            Open the fresh files panel from the Camera app by perfoming a swipe

        usage:
            ui_steps.OpenRecordedVideo(serial=dut_serial)()

        tags:
            ui, android, press, click, picture, photos
    """

    def __init__(self, **kwargs):
        ui_step.__init__(self, **kwargs)
        adb_step.__init__(self, **kwargs)
        self.x_center = self.uidevice.info['displayWidth']
        self.y_center = self.uidevice.info['displayHeight'] / 2

    def do(self):
        #swipe left to open the most recently recorded video
        swipe(serial=self.serial,
              sx=self.uidevice.info['displayWidth'] - 5,
              sy=self.y_center,
              ex=0,
              ey=self.y_center * 130 / 100)()

    def check_condition(self):
        return self.uidevice(descriptionContains='Share').wait.exists(timeout=1000)
