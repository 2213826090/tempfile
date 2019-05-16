from PyUiApi.common.storage_usb_utils import *
from PyUiApi.common.system_utils import *
from PyUiApi.app_utils.settings_utils import Settings
from PyUiApi.app_utils.camera_utils import QuickCamera, CameraFiles
from PyUiApi.app_utils.photos_utils import Photos
from PyUiApi.app_utils.camera_utils import Camera
from PyUiApi.tests.storage_usb_tests.generic import StorageUSBTests
import threading


class StorageUSBTestsGUI(unittest.TestCase):

    def setUp(self):
        UiAutomatorUtils.unlock_screen()
        self.initial_orientation = d.orientation
        self.screenshooter = ScreenshotUtils()
        LOG.info("Test started")

    def tearDown(self):
        self.log_before_cleanup()
        UiAutomatorUtils.close_all_tasks()
        if d.orientation != self.initial_orientation:
            d.orientation = self.initial_orientation
            d.freeze_rotation(False)
        self.screenshooter.remove_all_screenshots()
        LOG.info("Test finished")

    def log_before_cleanup(self):
        result = self._resultForDoCleanups
        if not result.wasSuccessful():
            TestUtils.log_screenshot_and_dump()

    def test_open_all_apps_same_time(self):
        # test is affected if a google account is not configured in advance
        packages_with_ui_activity = ["com.google.android.youtube", "com.google.android.googlequicksearchbox",
                                     "com.android.contacts", "com.google.android.deskclock", "com.google.android.gm",
                                     "com.google.android.music", "com.google.android.apps.docs",
                                     "com.google.android.apps.maps", "com.google.android.apps.plus",
                                     "com.google.android.calculator", "com.android.chrome", "com.google.android.apps.photos",
                                     "com.google.android.calendar", "com.android.settings",
                                     "com.google.android.GoogleCamera"]
        installed_app_packages = StorageUsbUtils.get_installed_packages()
        LOG.info("installed packages: " + str(installed_app_packages))
        app_failed_to_launch = False
        app_crashed = False
        packages_that_failed_to_start = []
        uiskipper.start_watching()
        try:
            for package_name in installed_app_packages:
                # try to open only packages with activities that can be launched
                if package_name not in packages_with_ui_activity:
                    continue  # if current package does not have launchable activity, go to next
                d.press.home()
                time.sleep(2)
                AdbUtils.start_activity_with_package_name(package_name)
                if d(packageName=package_name).wait.exists(timeout=10000):
                    LOG.info("package UI successfully started: " + package_name)
                    if SystemPopupsAndDialogs.is_app_crash_popup_visible():
                        app_crashed = True
                        LOG.info("package UI has crashed: " + package_name)
                else:
                    LOG.info("package UI WAS NOT started: " + package_name)
                    app_failed_to_launch = True
                    packages_that_failed_to_start.append(package_name)
            # test fails if one app fails to launch
            for failed_package in packages_that_failed_to_start:
                LOG.info("failed to start package: " + failed_package)
            self.assertFalse(app_failed_to_launch, "failed to launch all apps")
            self.assertFalse(app_crashed, "there was a crash while starting the apps")
        finally:
            uiskipper.stop_watching()
            # sleep 3 secs after closing each app to prevent unwanted app open
            apps_closed = UiAutomatorUtils.close_tasks_when_there_are_many_open()
            self.assertTrue(apps_closed >= len(packages_with_ui_activity) - 1,
                            "not all opened apps could be closed")

    def test_open_all_apps_same_time_full_storage(self):
        fill_dir_path = os.path.join(Environment.emulated_storage_path, "fill_mem_dir")
        try:
            StorageUsbUtils.fill_emulated_memory(10, fill_dir_path, leave_free_space_files=5)
            self.test_open_all_apps_same_time()
        finally:
            AdbUtils.delete_files(fill_dir_path)

    def test_low_internal_memory(self):
        fill_dir_path = os.path.join("/data/data/", "fill_mem_dir")
        try:
            # fill memory until there is less than 1GB space (50x10MB = 500MB free)
            StorageUsbUtils.fill_emulated_memory(10, fill_dir_path, leave_free_space_files=50)
            df_status = AdbUtils.run_adb_cmd("df | grep /data", add_ticks=False)
            free_data_space = df_status.split()[3]
            LOG.info("free space available for /data: " + free_data_space)
            free_space_value, free_space_unit = re.findall(u'(\d+\.\d+)(\w+)', free_data_space)[0]
            free_space_value = float(free_space_value)
            self.assertTrue("G" not in free_space_unit or free_space_value < 1,
                            "df reports incorrect free emulated storage value for /data")
            UiAutomatorUtils.unlock_screen()  # a long time has passed, maybe screen is locked
            Settings.open_storage_usb_options()
            time.sleep(3)
            self.screenshooter.take_screenshot()
            notification_pxs = self.screenshooter\
                .get_nr_of_pixels_of_color(STORAGE_FULL_STORAGE_NOTIFICATION_COLOR, color_error=15)
            self.assertTrue(notification_pxs > 40)  # magic nr 40
        finally:
            AdbUtils.delete_files(fill_dir_path)

    def test_low_external_memory(self):
        fill_dir_path = os.path.join(Environment.emulated_storage_path, "fill_mem_dir")
        try:
            # fill memory until there is less than 1GB space (50x10MB = 500MB free)
            StorageUsbUtils.fill_emulated_memory(10, fill_dir_path, leave_free_space_files=50)
            df_status = AdbUtils.run_adb_cmd("df | grep " + Environment.storage_emulated_root, add_ticks=False)
            free_data_space = df_status.split()[3]
            LOG.info("free space available for /data: " + free_data_space)
            free_space_value, free_space_unit = re.findall(u'(\d+\.\d+)(\w+)', free_data_space)[0]
            free_space_value = float(free_space_value)
            self.assertTrue("G" not in free_space_unit or free_space_value < 1,
                            "df reports incorrect free emulated storage value for /data")
            UiAutomatorUtils.unlock_screen()  # a long time has passed, maybe screen is locked
            Settings.open_storage_usb_options()
            time.sleep(3)
            self.screenshooter.take_screenshot()
            notification_pxs = self.screenshooter\
                .get_nr_of_pixels_of_color(STORAGE_FULL_STORAGE_NOTIFICATION_COLOR, color_error=15)
            self.assertTrue(notification_pxs > 40)  # magic nr 40
        finally:
            AdbUtils.delete_files(fill_dir_path)

    def test_adopted_unmount_mount(self):
        mount_stats_cmd = "cat /proc/self/mountstats | grep /mnt/expand"
        sdcard_mounted_str = "mounted on"
        try:
            # ensure sdcard is mounted before taking some pictures
            Settings.mount_sdcard()
            QuickCamera.take_photos(nr_of_photos=3)
            initial_pics = CameraFiles.get_camera_pictures()
            LOG.info("initial pictures before unmount: " + str(initial_pics))
            # unmount
            Settings.unmount_sdcard()
            mount_stats = AdbUtils.run_adb_cmd(mount_stats_cmd, add_ticks=False)
            self.assertTrue(sdcard_mounted_str not in mount_stats, "sdcard was not correctly unmounted")
            LOG.info("sdcard mountstats: " + str(mount_stats))
            unmount_pics = CameraFiles.get_camera_pictures()
            LOG.info("pictures after unmount: " + str(unmount_pics))
            Photos.launch_without_account()
            unmount_nr_of_photos = Photos.get_nr_of_photos()
            LOG.info("found photos in Photo app after unmount: " + str(unmount_nr_of_photos))
            # remount
            Settings.mount_sdcard()
            mount_stats = AdbUtils.run_adb_cmd(mount_stats_cmd, add_ticks=False)
            self.assertTrue(sdcard_mounted_str in mount_stats, "sdcard was not correctly mounted")
            LOG.info("sdcard mountstats: " + str(mount_stats))
            mount_pics = CameraFiles.get_camera_pictures()
            LOG.info("pictures after remount: " + str(mount_pics))
            Photos.launch_without_account()
            remount_nr_of_photos = Photos.get_nr_of_photos()
            LOG.info("found photos in Photo app after remount: " + str(remount_nr_of_photos))
            self.assertTrue(remount_nr_of_photos > unmount_nr_of_photos and len(unmount_pics) < len(mount_pics),
                            "photos after unmounting are comparable to photos after remounting")
        finally:
            Settings.mount_sdcard()

    def test_delete_data_make_space_for_camera(self):
        fill_dir_path = os.path.join("/data/data", "fill_mem_dir")
        try:
            # fill memory until there is less than 1GB space (50x10MB = 500MB free)
            StorageUsbUtils.fill_emulated_memory(10, fill_dir_path, leave_free_space_files=1)
            time.sleep(5)
            AdbUtils.start_activity_with_package_name("com.google.android.GoogleCamera")
            time.sleep(5)  # wait a while for the "sdcard full" notification to appear
            logcat_messages = AdbUtils.run_adb_cmd("logcat -d | grep 'SD card'", adb_shell=False)
            LOG.info("logcat low memory SD card notifications: " + logcat_messages)
            self.assertTrue("Storage warning: Your SD card is running out of space" in logcat_messages,
                            "there was no low space warning message in logcat")
            AdbUtils.run_adb_cmd("rm -rf " + os.path.join(fill_dir_path, "testfile1*"))
            time.sleep(5)
            Camera.launch()
            self.assertTrue(d(resourceId=CAMERA_SHUTTER_RESID).wait.exists(timeout=3000),
                             "camera shutter button should be accesible after deleting some files")
        finally:
            AdbUtils.delete_files(fill_dir_path)

    def test_format_and_switch_orientation(self):
        orientation_changer = ContinuousOrientationChanger(change_wait=0.5)
        test_result = []

        def format_sdcard():
            try:
                # pass test result to the outside context
                test_result.append(SingleMethodRunner.run_single_test(StorageUSBTests, "test_format_as_portable"))
            finally:
                ContinuousOrientationChanger.stop_operation = True

        def change_orientation():
            orientation_changer.start_running()

        format_thread = threading.Thread(target=format_sdcard,)
        orientation_thread = threading.Thread(target=change_orientation,)
        orientation_thread.start()
        format_thread.start()
        format_thread.join()
        orientation_thread.join()
        self.assertTrue(len(test_result) > 0, "format sdcard did not finish properly")
        self.assertTrue(test_result.pop().wasSuccessful(), "format sdcard was not successful")
