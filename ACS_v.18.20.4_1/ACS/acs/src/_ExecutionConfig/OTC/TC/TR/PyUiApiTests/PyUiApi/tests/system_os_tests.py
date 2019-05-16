from PyUiApi.common.system_utils import *
from PyUiApi.common.status_bar import *
import signal
from PyUiApi.common.environment_utils import *
from PyUiApi.app_utils.settings_utils import *
from PyUiApi.dut_info.dut_info import *
from PyUiApi.app_utils.clock_utils import *
from PyUiApi.app_utils.gmail_utils import *
from PyUiApi.app_utils.calendar_utils import *
from datetime import datetime
from PyUiApi.common.common_utils import *
from threading import Thread
from PyUiApi.app_utils.chrome_utils import *


class SystemOsTests(unittest.TestCase):
    TAG = "SystemOsTests"
    youtube_sample_video = "http://www.youtube.com/watch?v=YRhFSWz_J3I"
    ndk_depends_bin = "ndk-depends"
    bootanimation_bin = "bootanimation"
    hwuitest_bin = "hwuitest"
    gcm_sender_jar = "gcm_message_sender.jar"

    def setUp(self):
        UiAutomatorUtils.unlock_screen()
        self.initial_orientation = d.orientation
        self.screenshooter = ScreenshotUtils()

    def tearDown(self):
        self.log_before_cleanup()
        UiAutomatorUtils.close_all_tasks()
        if d.orientation != self.initial_orientation:
            d.orientation = self.initial_orientation
            d.freeze_rotation(False)
        self.screenshooter.remove_all_screenshots()

    def log_before_cleanup(self):
        result = self._resultForDoCleanups
        if not result.wasSuccessful():
            TestUtils.log_screenshot_and_dump()

    def test_verify_20_recent_apps(self):
        activity_string_template = "com.example.test$NR$/.MainActivity"
        app_string_template = "com.example.test$NR$"
        UiAutomatorUtils.close_all_tasks()
        for i in range(1, 21):
            package_name = app_string_template.replace("$NR$", str(i))
            activity_string = activity_string_template.replace("$NR$", str(i))
            AdbUtils.start_activity_from_shell(activity_string)
            d(packageName=package_name).wait.exists(timeout=3000)
            AdbUtils.force_stop_app(package_name)
        # wait a little for the tasks to be registered with the visual manager component
        nr_of_dismissed_tasks = 0
        # hack to count a large number of dismissed tasks
        while True:
            task_dismissed_at_current_step = UiAutomatorUtils.close_all_tasks()
            nr_of_dismissed_tasks += task_dismissed_at_current_step
            if task_dismissed_at_current_step == 0:
                break
        print nr_of_dismissed_tasks
        LOG.info("dismissed " + str(nr_of_dismissed_tasks) + " tasks", self)
        self.assertTrue(nr_of_dismissed_tasks >= 20, "test was not able to dismiss 20 recent tasks")

    def test_verify_saturation_and_hue(self):
        test_cmds = ["am instrument -e class com.intel.test.apitests.tests.ColorTestsDriver#testColorToHsv"
                     " -w com.intel.test.apitests/com.intel.test.apitests.runners.GenericArgumentPassingTestRunner",
                     "am instrument -e class android.graphics.cts.ColorTest#testHSVToColor1"
                     " -w com.android.cts.graphics/android.test.InstrumentationCtsTestRunner",
                     "am instrument -e class android.graphics.cts.ColorTest#testHSVToColor2"
                     " -w com.android.cts.graphics/android.test.InstrumentationCtsTestRunner",
                     "am instrument -e class android.graphics.cts.ColorMatrixTest#testSetSaturation"
                     " -w com.android.cts.graphics/android.test.InstrumentationCtsTestRunner"]
        if ANDROID_VERSION is 'N':
            # graphics color tests not supported for Android N
            test_cmds = test_cmds[:1]
        for cmd in test_cmds:
            output = AdbUtils.run_adb_cmd(cmd)
            self.assertTrue("OK" in output, "some saturation and hue instrumentation tests failed")

    def test_animation_cts(self):
        test_cmd = "am instrument -e package android.animation.cts " \
                   "-w com.android.cts.animation/android.test.InstrumentationCtsTestRunner"
        output = AdbUtils.run_adb_cmd(test_cmd)
        self.assertTrue("OK" in output, "some animation instrumentation tests failed")

    def test_accounts_cts(self):
        test_cmd = "am instrument -e package android.accounts.cts " \
                   "-w android.accounts.cts/android.test.InstrumentationCtsTestRunner"
        output = AdbUtils.run_adb_cmd(test_cmd)
        self.assertTrue("OK" in output, "some accounts instrumentation tests failed")

    def test_hardware_acceleration_cts(self):
        test_cmd = "am instrument -e package android.acceleration.cts " \
                   "-w com.android.cts.acceleration/android.test.InstrumentationCtsTestRunner"
        output = AdbUtils.run_adb_cmd(test_cmd)
        self.assertTrue("OK" in output, "some hardware instrumentation tests failed")

    def test_verify_x86_abi(self):
        cmd = "getprop | grep -i abi"
        output = AdbUtils.run_adb_cmd(cmd)
        print output
        expected_results = ["[ro.product.cpu.abi]: [x86]",
                            "[ro.product.cpu.abilist32]: [x86,armeabi-v7a,armeabi]",
                            "[ro.product.cpu.abilist64]: []",
                            "[ro.product.cpu.abilist]: [x86,armeabi-v7a,armeabi]"]
        for result in expected_results:
            self.assertTrue(result in output, "reference string " + str(result) + " not found in abi props")
        cmd = "echo 1 > /proc/sys/kernel/kptr_restrict"
        AdbUtils.run_adb_cmd(cmd)
        cmd = "cat /proc/kallsyms | head -5"
        output = AdbUtils.run_adb_cmd(cmd, add_ticks=False)
        print output
        for line in output.splitlines():
            strings = line.split()
            self.assertTrue(len(strings[0]) == 16, "cat /proc/kallsyms address is not 16 bits: " + str(line))
        cmd = "cat /proc/1/maps | head -5"
        output = AdbUtils.run_adb_cmd(cmd, add_ticks=False)
        print output
        self.assertTrue("/init" in output, "/init not in cat /proc/1/maps output")
        self.assertTrue("[anon:libc_malloc]" in output, "[anon:libc_malloc] not in cat /proc/1/maps output")
        output_lines = output.splitlines()
        for line in output_lines:
            addr = line.split()[2]
            print addr
            # 32 bit address = 8 hex digits
            self.assertTrue(len(addr) == 8, "cat /proc/1/maps address is not 8 hex digits long: " + str(line))

    def test_verify_x86_abi_kernel_64_userspace_64(self):
        cmd = "getprop | grep -i abi"
        output = AdbUtils.run_adb_cmd(cmd)
        print output
        expected_results = ["[ro.product.cpu.abi]: [x86_64]",
                            "[ro.product.cpu.abilist32]: [x86,armeabi-v7a,armeabi]",
                            "[ro.product.cpu.abilist64]: [x86_64,arm64-v8a]",
                            "[ro.product.cpu.abilist]: [x86_64,x86,armeabi-v7a,armeabi,arm64-v8a]",
                            "[sys.chaabi.version]: "]
        for result in expected_results:
            self.assertTrue(result in output, "reference string: " + str(result) + " not in getprop abi output")
        cmd = "echo 1 > /proc/sys/kernel/kptr_restrict"
        AdbUtils.run_adb_cmd(cmd)
        cmd = "cat /proc/kallsyms | head -5"
        output = AdbUtils.run_adb_cmd(cmd, add_ticks=False)
        print output
        for line in output.splitlines():
            strings = line.split()
            self.assertTrue(len(strings[0]) == 16, "cat /proc/kallsyms address is not 16 bits: " + str(line))
        cmd = "cat /proc/1/maps | head -5"
        output = AdbUtils.run_adb_cmd(cmd, add_ticks=False)
        print output
        self.assertTrue("/init" in output, "/init not in cat /proc/1/maps output")
        # self.assertTrue("[anon:libc_malloc]" in output, "[anon:libc_malloc] not in cat /proc/1/maps output")
        output_lines = output.splitlines()
        for line in output_lines:
            if "[anon:libc_malloc]" in line:
                strings = line.split()
                addr = strings[0]
                # observe large address
                self.assertTrue(len(addr) == 25, "cat /proc/1/maps address is not 25 chars long: " + str(line))

    def test_verify_x86_abi_kernel_32_userspace_32(self):
        cmd = "getprop | grep -i abi"
        output = AdbUtils.run_adb_cmd(cmd)
        print output
        expected_results = ["[ro.product.cpu.abi]: [x86]",
                            "[ro.product.cpu.abilist32]: [x86,armeabi-v7a,armeabi]",
                            "[ro.product.cpu.abilist64]: []",
                            "[ro.product.cpu.abilist]: [x86,armeabi-v7a,armeabi]"]
        for result in expected_results:
            self.assertTrue(result in output, "reference string: " + str(result) + " not in getprop abi output")
        cmd = "echo 1 > /proc/sys/kernel/kptr_restrict"
        AdbUtils.run_adb_cmd(cmd)
        cmd = "cat /proc/kallsyms | head -5"
        output = AdbUtils.run_adb_cmd(cmd, add_ticks=False)
        print output
        for line in output.splitlines():
            strings = line.split()
            self.assertTrue(len(strings[0]) == 8, "cat /proc/kallsyms address is not 8 bits: " + str(line))
        cmd = "cat /proc/1/maps | head -5"
        output = AdbUtils.run_adb_cmd(cmd, add_ticks=False)
        print output
        self.assertTrue("/init" in output, "/init not in cat /proc/1/maps output")
        self.assertTrue("[anon:libc_malloc]" in output, "[anon:libc_malloc] not in cat /proc/1/maps output")
        output_lines = output.splitlines()
        for line in output_lines:
            addr = line.split()[2]
            print addr
            # 32 bit address = 8 hex digits
            self.assertTrue(len(addr) == 8, "cat /proc/1/maps address is not 8 bit long: " + str(line))
            if "[anon:libc_malloc]" in line:
                strings = line.split()
                addr = strings[0]
                self.assertTrue(len(addr) == 17, "libc_malloc address is not 17 char long")  # observe large address

    def test_immersive_mode(self):
        UiAutomatorUtils.launch_app_from_apps_menu(SYSTEM_OS_IMMERSIVE_APP_SHORTCUT_NAME)
        # enter immersive mode
        self.assertTrue(d(text=SYSTEM_OS_ENTER_IMMERSIVE_MODE_TXT).wait.exists(timeout=4000))
        d(text=SYSTEM_OS_ENTER_IMMERSIVE_MODE_TXT).click()
        # wait for immersive mode
        time.sleep(3)
        self.screenshooter.take_screenshot()
        StatusBar.open_notifications(nr_of_swipes=2)
        time.sleep(1)
        self.screenshooter.take_screenshot()
        # check if status bar is showing after swipe
        self.assertFalse(self.screenshooter.same_screenshots(-1, -2))

    def test_data_usage(self):
        data_usage = Settings.get_data_usage_info()
        print data_usage
        self.assertTrue(len(data_usage) > 0)

    def test_screen_recording(self):
        recorder = ScreenRecorder(3000000, 50, 1)
        test_file_name = "recording_test_video.mp4"
        screen_record_file_path = None
        try:
            if ScreenRecorderManager.start_recording_delay_var_name in os.environ:
                del os.environ[ScreenRecorderManager.start_recording_delay_var_name]
            recorder.start_recording()
            recording_time_secs = 20
            orientations = ["n", "l", "r"]
            for i in range(recording_time_secs):
                time.sleep(1)
                if i % 4 == 0:
                    OrientationChanger.change_orientation(orientations[(i / 4) % 3])
            recorder.stop_recording()
            time.sleep(2)
            screen_record_file_path = recorder.retrieve_recording(test_file_name)
            print screen_record_file_path
            self.assertTrue(screen_record_file_path is not None)
            statinfo = os.stat(screen_record_file_path)
            self.assertTrue(statinfo.st_size > 80000L)  # magic size number
            subprocess.call("rm -rf *.mpg", shell=True)  # ensure there is no duplicate file
            # throws exception if conversion yields errors
            subprocess.check_output("mediainfo " + screen_record_file_path + " test.mpg", shell=True)
        finally:
            if screen_record_file_path is not None:
                os.remove(screen_record_file_path)  # remove the pulled media file
                subprocess.call("rm -rf *.mpg", shell=True)

    def test_screenshot_app_rotate(self):
        app_start_cmd = "am start -n com.intel.test.apitests/.ApiTests"
        AdbUtils.run_adb_cmd(app_start_cmd)
        time.sleep(5)
        screenshoter = ScreenshotUtils()
        OrientationChanger.change_orientation("n")
        screenshoter.take_screenshot()
        screenshot_width_n = screenshoter.screen_width
        screenshot_height_n = screenshoter.screen_height
        print "natural orientation dimensions: ", screenshot_width_n, screenshot_height_n
        OrientationChanger.change_orientation("l")
        screenshoter.take_screenshot()
        screenshot_width_l = screenshoter.screen_width
        screenshot_height_l = screenshoter.screen_height
        print "left orientation dimensions: ", screenshot_width_l, screenshot_height_l
        self.assertTrue(screenshot_height_n == screenshot_width_l)
        self.assertTrue(screenshot_height_l == screenshot_width_n)

    def test_screen_density_reporting(self):
        metrics = SystemUtils.get_display_metrics()
        print metrics
        reported_density = metrics["densityDpi"]
        platform_name = SystemUtils.get_platform_name()
        current_platform = PlatformCatalogue.find_platform(platform_name)
        if current_platform is not None:
            reference_density = current_platform.get_property("ro.sf.lcd_density")
            print "reported density: ", reported_density
            print "reference density: ", reference_density
            self.assertTrue(reported_density in reference_density)
            lcd_density_info = current_platform.get_property("ro.sf.lcd_density_info")
            print "reference density info: ", lcd_density_info
            if lcd_density_info is not None:
                self.assertTrue("density: " + reported_density in lcd_density_info)
        else:
            shell_prop_density = SystemUtils.get_property("ro.sf.lcd_density", "density")
            print shell_prop_density
            print reported_density
            self.assertTrue(int(shell_prop_density) == int(reported_density))

    def test_verify_screenshot_width_height(self):
        screen_shooter = ScreenshotUtils()
        try:
            screen_shooter.take_screenshot()
            print "screenshot: ", screen_shooter.screen_width, screen_shooter.screen_height
            shell_prop_density_info = SystemUtils.get_property("ro.sf.lcd_density_info", "density")
            test_width, test_height = SystemUtils.get_screen_size_from_lcd_density_info(shell_prop_density_info)
            print "shell prop:", test_width, test_height
            self.assertTrue(int(screen_shooter.screen_width) == int(test_width))
            self.assertTrue(int(screen_shooter.screen_height) == int(test_height))
        finally:
            screen_shooter.remove_all_screenshots()

    def get_density_value(self, density_string):
        density = re.findall(u'density:(\s+\d+)', density_string)[0]
        return int(density)

    def test_screen_size_reporting(self):
        metrics = SystemUtils.get_display_metrics()
        print metrics
        reported_density_info = SystemUtils.get_property('ro.sf.lcd_density_info', 'ro.sf')
        platform_name = SystemUtils.get_platform_name()
        current_platform = PlatformCatalogue.find_platform(platform_name)
        if current_platform is not None:
            lcd_density_info = current_platform.get_property("ro.sf.lcd_density_info")
            print "reported density info: ", reported_density_info
            print "expected density info: ", lcd_density_info
            reported_density = self.get_density_value(reported_density_info)
            lcd_density = self.get_density_value(lcd_density_info)
            if lcd_density_info is not None:
                self.assertTrue(reported_density ==  lcd_density, "reported info: "
                                + reported_density_info + " ; expected info: " + lcd_density_info)
            elif reported_density_info is not None:
                self.assertTrue(str(current_platform.screenWidth) + " x " + str(current_platform.screenHeight)
                                in reported_density_info)
            test_width = None
            test_height = None
            if lcd_density_info is None:
                test_width = current_platform.screenWidth
                test_height = current_platform.screenHeight
            else:
                test_width, test_height = SystemUtils.get_screen_size_from_lcd_density_info(lcd_density_info)
            print "testing values for screen size: " + str(test_width) + " x " + str(test_height)
            screen_size_instrumentation_test = "am instrument -e class com.intel.test.apitests.tests" \
                                               ".DisplayMetricsTestDriver#testDisplayInterfaceSize -e args" \
                                               ' "DisplayWidth:$WIDTH$ DisplayHeight:$HEIGHT$" -w com.intel.test' \
                                               ".apitests/com.intel.test.apitests.runners.GenericArgumentPassingTestRunner"
            test_cmd = screen_size_instrumentation_test.replace('$WIDTH$', str(test_width)) \
                .replace('$HEIGHT$', str(test_height))
            test_output = AdbUtils.run_adb_cmd(test_cmd)
            print test_output
            self.assertTrue('OK' in test_output)
        else:
            shell_prop_density_info = SystemUtils.get_property("ro.sf.lcd_density_info", "density")
            test_width, test_height = SystemUtils.get_screen_size_from_lcd_density_info(shell_prop_density_info)
            print test_width, test_height
            reported_height = metrics["heightPixels"]
            reported_width = metrics["widthPixels"]
            print reported_width
            print reported_height
            self.assertTrue(abs(int(reported_width) - int(test_width)) <= int(test_width) / 15)
            self.assertTrue(abs(int(reported_height) - int(test_height)) <= int(test_height) / 15)

    def test_memtrack_test_from_android_tree(self):
        root_cmd = "root"
        AdbUtils.run_adb_cmd(root_cmd, adb_shell=False)
        set_permissions_cmd = "chmod 755 /data/memtrack_test"
        AdbUtils.run_adb_cmd(set_permissions_cmd)
        run_memtrack_cmd = "/data/data/memtrack_test"
        output = AdbUtils.run_adb_cmd(run_memtrack_cmd)
        self.assertTrue("/system/bin/" in output)
        # assert the output has at least 2 lines
        lines = output.splitlines()
        self.assertTrue(len(lines) > 2)
        # assert that a line has 8 columns
        self.assertTrue(len(lines[0].split()) == 8)

    def test_crash_reporting_bugreport(self):
        bug_report_filename = "bug_report_test.rpt"
        bugreport_sections = {"UPTIME": 0, "MMC PERF": 0, "MEMORY INFO": 0, "CPU INFO": 0, "PROCRANK": 0,
                              "VIRTUAL MEMORY STATS": 0, "VMALLOC INFO": 0, "SLAB INFO": 0, "ZONEINFO": 0,
                              "PAGETYPEINFO": 0, "BUDDYINFO": 0, "FRAGMENTATION INFO": 0, "KERNEL WAKELOCKS": 0,
                              "KERNEL WAKE SOURCES": 0, "KERNEL CPUFREQ": 0, "KERNEL SYNC": 0, "KERNEL LOG": 0,
                              "SHOW MAP": 0, "PROCESSES": 0, "SYSTEM LOG": 0, "EVENT LOG": 0, "pid": 0,
                              "NETWORK": 0, "QTAGUID": 0, "IP RULES": 0, "ROUTE TABLE": 0,
                              "CACHE": 0, "SYSTEM PROPERTIES": 0, "VOLD DUMP": 0, "BINDER": 0, "DUMPSYS": 0, "APP": 0}

        def manage_bugreport_line(bugreport_line):
            for section in bugreport_sections.keys():
                if section in line:
                    bugreport_sections[section] += 1

        try:
            cmd = "bugreport > " + bug_report_filename
            AdbUtils.run_adb_cmd(cmd, False)
            report_file = open(bug_report_filename)
            statinfo = os.stat(bug_report_filename)
            self.assertTrue(statinfo.st_size > 5000000L)  # magic size number
            for line in report_file:
                if "-----" in line:
                    manage_bugreport_line(line)
        finally:
            os.remove(bug_report_filename)
        missing_section = False
        for key, value in bugreport_sections.iteritems():
            if value == 0:
                missing_section = True
                LOG.info("found missing section " + str(key))
        self.assertFalse(missing_section)

    def test_dumpsys_memtrack_reporting(self):
        dumpsys_cmd = "dumpsys meminfo"
        output = AdbUtils.run_adb_cmd(dumpsys_cmd)
        self.assertTrue("Total PSS by category:" in output)
        self.assertTrue(": Dalvik" in output and ": Native" in output and ": Ashmem" in output and
                        ": GL" in output)
        self.assertTrue("Total RAM:" in output and "Free RAM:" in output and "Used RAM:" in output and
                        "Lost RAM:" in output)

    def test_loop_read_of_memtrack(self):
        egl_mtrack_dumpsys_meminfo = AdbUtils.run_adb_cmd('dumpsys meminfo | grep "EGL mtrack"')
        LOG.info("EGL mtrack: " + egl_mtrack_dumpsys_meminfo)
        self.assertTrue(" 0 " not in egl_mtrack_dumpsys_meminfo)
        AdbUtils.run_adb_cmd("dmesg -c", add_ticks=False)
        loop_read_cmd = 'for i in 1 2 3 4 5 6 7 8 9 10; do sh -c "cat /sys/class/drm/card0/gfx_memtrack/*" & done'
        output = AdbUtils.run_adb_cmd(loop_read_cmd)
        if Environment.file_not_found_cmd_output in output:
            return
        self.assertTrue("No such file or directory" not in output)
        self.assertTrue("Total used GFX Shmem" in output)
        error_msgs_cmd = "dmesg"
        output = AdbUtils.run_adb_cmd(error_msgs_cmd, add_ticks=False)
        self.assertTrue(" trace " not in output)
        self.assertTrue(" bug " not in output)
        self.assertTrue(" stack " not in output)

    def test_long_loop_read_of_memtrack(self):
        null_pipe = open(os.devnull, 'w')
        AdbUtils.run_adb_cmd("dmesg -c", add_ticks=False)
        long_loop_read_proc_cmd = 'for i in 1 2 3 4 5 6 7 8 9 10; do while :; ' \
                                  'do cat /sys/class/drm/card0/gfx_memtrack/* > /dev/null; done & done'
        long_loop_proc = subprocess.Popen(long_loop_read_proc_cmd, shell=True, stdout=null_pipe,
                                          stderr=subprocess.STDOUT)
        try:
            # launch youtube and watch a video
            UiAutomatorUtils.launch_activity_with_data_uri(SystemOsTests.youtube_sample_video)
            if d(text=OS_OPEN_JUST_ONCE_TXT).wait.exists(timeout=3000):
                d(text=OS_OPEN_JUST_ONCE_TXT).click()
            self.assertTrue(d(packageName=YOUTUBE_PACKAGE_NAME).wait.exists(timeout=10000))
            # let the youtube play for some time
            time.sleep(10)
            # check for potential errors
            error_msgs_cmd = "dmesg"
            output = AdbUtils.run_adb_cmd(error_msgs_cmd, add_ticks=False)
            self.assertTrue(" trace " not in output)
            self.assertTrue(" bug " not in output)
            self.assertTrue(" stack " not in output)
        finally:
            long_loop_proc.terminate()
            os.kill(long_loop_proc.pid, signal.SIGTERM)
            ShellUtils.kill_processes_containing_string("gfx_memtrack")

    def test_screenshot_video_playback(self):
        UiAutomatorUtils.launch_activity_with_data_uri(SystemOsTests.youtube_sample_video)
        if d(text=OS_OPEN_JUST_ONCE_TXT).wait.exists(timeout=3000):
            d(text=OS_OPEN_JUST_ONCE_TXT).click()
        self.assertTrue(d(packageName=YOUTUBE_PACKAGE_NAME).wait.exists(timeout=10000))
        # wait for the video to start
        time.sleep(10)
        self.screenshooter.take_screenshot()
        # wait for misc error messages to appear
        time.sleep(2)
        self.assertTrue(d(packageName=YOUTUBE_PACKAGE_NAME).wait.exists(timeout=1000))

    def test_gpu_overdraw(self):
        try:
            # enable showing of overdraw areas
            Settings.enable_gpu_overdraw_show_overdraw_areas()
            self.screenshooter.take_screenshot()
            total_nr_of_pixels = self.screenshooter.screen_width * self.screenshooter.screen_height
            dark_red_pixels = self.screenshooter.get_dark_red_pixels_from_current_screenshot()
            nr_of_dark_red_pixels = len(dark_red_pixels)
            LOG.info("dark red overdraw pixels nr.: " + str(nr_of_dark_red_pixels))
            percent_of_dark_red_pixels = nr_of_dark_red_pixels * 100.0 / total_nr_of_pixels
            LOG.info("dark red percent: " + str(percent_of_dark_red_pixels))
            self.assertTrue(percent_of_dark_red_pixels < 5, "dark red pixels must be under 5%")
            # enable showing of deuteranomaly
            Settings.enable_gpu_overdraw_show_deuteranomaly()
            self.screenshooter.take_screenshot()
            dark_red_pixels = self.screenshooter.get_dark_red_pixels_from_current_screenshot()
            nr_of_dark_red_pixels = len(dark_red_pixels)
            LOG.info("dark red deuteranomaly pixels nr.: " + str(nr_of_dark_red_pixels))
            percent_of_dark_red_pixels = nr_of_dark_red_pixels * 100.0 / total_nr_of_pixels
            LOG.info("dark red percent: " + str(percent_of_dark_red_pixels))
            self.assertTrue(percent_of_dark_red_pixels < 5, "dark red pixels must be under 5%")
        finally:
            # disable gpu_overdraw
            Settings.disable_gpu_overdraw()

    def test_take_bug_report(self):
        Settings.take_bug_report()
        # a popup for taking a bug report should appear
        self.assertTrue(d(text=SETTINGS_TAKE_BUG_REPORT_POPUP_REPORT_BUTTON_TXT).wait.exists(timeout=3000))
        d(text=SETTINGS_TAKE_BUG_REPORT_POPUP_REPORT_BUTTON_TXT).click()
        poll_waiting_tries_for_notification = 20
        found_bug_report_notification = False
        while poll_waiting_tries_for_notification > 0:
            time.sleep(10)
            StatusBar.open_notifications()
            if d(text=SETTINGS_TAKE_BUG_REPORT_NOTIFICATION_TXT).wait.exists(timeout=3000):
                # bug report was completed
                found_bug_report_notification = True
                break
            d.press.home()  # close notifications
            poll_waiting_tries_for_notification -= 1
        self.assertTrue(found_bug_report_notification,
                        "a notification for the bugreport should have appeared")
        # StatusBar.open_notifications()  # notifications already opened after break in for
        self.assertTrue(d(text=SETTINGS_TAKE_BUG_REPORT_NOTIFICATION_TXT).wait.exists(timeout=3000),
                        "a bug report notification should exist in the notifications bar")
        d(text=SETTINGS_TAKE_BUG_REPORT_NOTIFICATION_TXT).click()
        # popup to warn not to share sensitive data with untrusted parties
        SystemPopupsAndDialogs.popup_ok()
        # popup for choosing wich app will handle the bugreport
        # there are different types of popup for different Android versions and devices
        SystemPopupsAndDialogs.open_resource_with(GMAIL_APP_NAME)
        # gmail application must open with attached bug report information
        self.assertTrue(d(packageName=GMAIL_PACKAGE_NAME).wait.exists(timeout=5000),
                        "gmail should have opened displaying the send bug report mail")
        time.sleep(5)
        SystemPopupsAndDialogs.popup_skip()
        Gmail.select_sync_account_now()
        self.assertTrue(d(textStartsWith=GMAIL_TAKE_BUG_REPORT_ATTACHEMENT_PREFIX_TXT)
                        .wait.exists(timeout=50000),
                        "mail should have an attachement containing bugreport details")

    def test_notifications_alarms(self):
        hh, mm, ss = ShellUtils.get_current_dut_time()
        try:
            alarm_time = Clock.set_new_alarm(hh, mm)
            hh, mm, ss = ShellUtils.get_current_dut_time()
            current_time = datetime.datetime(1, 1, 1, int(hh), int(mm), int(ss))
            time_delta = alarm_time - current_time
            time.sleep(time_delta.seconds + 15)
            StatusBar.open_notifications()
            self.assertTrue(d(text="Alarm").wait.exists(timeout=3000))
            self.assertTrue(d(text=CLOCK_SNOOZE_ALARM_TXT).wait.exists(timeout=3000))
            d(text=CLOCK_SNOOZE_ALARM_TXT).click()
            d.press.home()
            time.sleep(5)
            StatusBar.open_notifications()
            self.assertTrue(d(text="Alarm").wait.exists(timeout=3000))
            self.assertTrue(d(textContains=CLOCK_SNOOZING_STATUS_TXT).wait.exists(timeout=3000))
            self.assertTrue(d(textContains=CLOCK_DISMISS_ALARM_TXT).wait.exists(timeout=3000))
            d(textContains=CLOCK_DISMISS_ALARM_TXT).click()
        finally:
            Clock.delete_all_alarms()

    def test_ST_SYST_SWUPD_MT_036(self):
        hh, mm, ss = ShellUtils.get_current_dut_time()
        try:
            alarm_time = Clock.set_new_alarm(hh, mm)
            hh, mm, ss = ShellUtils.get_current_dut_time()
            current_time = datetime.datetime(1, 1, 1, int(hh), int(mm), int(ss))
            time_delta = alarm_time - current_time
            time.sleep(time_delta.seconds + 15)
            StatusBar.open_notifications()
            self.assertTrue(d(text="Alarm").wait.exists(timeout=3000))
            self.assertTrue(d(text=CLOCK_SNOOZE_ALARM_TXT).wait.exists(timeout=3000))
            self.assertTrue(d(textContains=CLOCK_DISMISS_ALARM_TXT).wait.exists(timeout=3000))
            d(text=CLOCK_DISMISS_ALARM_TXT).click()
        finally:
            Clock.delete_all_alarms()

    def test_ST_SYST_SWUPD_MT_034(self):
        hh, mm, ss = ShellUtils.get_current_dut_time()
        try:
            # Set two alarms
            Clock.set_new_alarm(hh, mm)
            alarm_time = Clock.set_new_alarm(hh, mm)
            hh, mm, ss = ShellUtils.get_current_dut_time()
            current_time = datetime.datetime(1, 1, 1, int(hh), int(mm), int(ss))
            time_delta = alarm_time - current_time
            time.sleep(time_delta.seconds + 15)
            StatusBar.open_notifications()
            self.assertTrue(d(text="Alarm").wait.exists(timeout=3000))
            self.assertTrue(d(text=CLOCK_SNOOZE_ALARM_TXT).wait.exists(timeout=3000))
            self.assertTrue(d(textContains=CLOCK_DISMISS_ALARM_TXT).wait.exists(timeout=3000))
            d(text=CLOCK_DISMISS_ALARM_TXT).click()
        finally:
            Clock.delete_all_alarms()

    def test_alarm_after_reboot(self):
        hh, mm, ss = ShellUtils.get_current_dut_time()
        result = SystemApiTestsInterface.run_instrumentation(class_name="CommonTestsDriver",
                                                             method_name="testDisableAutoTime",
                                                             instrumentation_args=None,
                                                             runner_name="GenericArgumentPassingTestRunner")
        self.assertTrue(SystemApiTestsInterface.was_instrumentation_test_successful(result),
                        "Could not disable auto time sync")
        try:
            # set an alarm at least 250 seconds into the future
            alarm_time = Clock.set_new_alarm(hh, mm, offset=120)
            hh, mm, ss = ShellUtils.get_current_dut_time()
            current_time = datetime.datetime(1, 1, 1, int(hh), int(mm), int(ss))
            time.sleep(5)
            reboot_time = UiAutomatorUtils.reboot_device()
            UiAutomatorUtils.unlock_screen()
            time_delta = alarm_time - current_time
            sleep_time = time_delta.seconds - reboot_time
            if sleep_time > 0:
                LOG.info("sleeping for %s seconds" % time_delta.seconds)
                time.sleep(sleep_time)
            StatusBar.open_notifications()
            self.assertTrue(d(text="Alarm").wait.exists(timeout=3000))
            self.assertTrue(d(text=CLOCK_SNOOZE_ALARM_TXT).wait.exists(timeout=3000))
            self.assertTrue(d(textContains=CLOCK_DISMISS_ALARM_TXT).wait.exists(timeout=3000))
            d(text=CLOCK_DISMISS_ALARM_TXT).click()
        finally:
            Clock.delete_all_alarms()

    def test_check_reminder_granularity(self):
        result = SystemApiTestsInterface.run_instrumentation(class_name="CommonTestsDriver",
                                                             method_name="testDisableAutoTime",
                                                             instrumentation_args=None,
                                                             runner_name="GenericArgumentPassingTestRunner")
        self.assertTrue(SystemApiTestsInterface.was_instrumentation_test_successful(result),
                        "Could not disable auto time sync")

        time_slided_on_dut = None
        try:
            # set a reminder 5 minutes (300 seconds) in the future
            event_title = "GranularityTestMinutes"
            event_created = Calendar.create_calendar_event(event_title, "GranularityTestMinutesDesc", 300, 100)
            self.assertTrue(event_created, "5 minute granularity event was not created")
            SystemUtils.set_system_time_slide_seconds(250)
            self.assertTrue(SystemUtils.wait_for_reminder_notification(event_title),
                            "5 minute granularity event did not trigger a notification")
            time_slided_on_dut = 250

            # set a reminder 1 hour (3600 seconds) in the future
            event_title = "GranularityTestHours"
            event_created = Calendar.create_calendar_event(event_title, "GranularityTestHoursDesc", 3600, 100)
            self.assertTrue(event_created, "1 hour granularity event was not created")
            SystemUtils.set_system_time_slide_seconds(3600)
            self.assertTrue(SystemUtils.wait_for_reminder_notification(event_title),
                            "1 hour granularity event did not trigger a notification")
            time_slided_on_dut += 3600

            # set a reminder 1 day (86400 seconds) in the future
            event_title = "GranularityTestDays"
            event_created = Calendar.create_calendar_event(event_title, "GranularityTestDaysDesc", 86400, 100)
            self.assertTrue(event_created, "1 hour granularity event was not created")
            SystemUtils.set_system_time_slide_seconds(86400)
            self.assertTrue(SystemUtils.wait_for_reminder_notification(event_title),
                            "1 day granularity event did not trigger a notification")
            time_slided_on_dut += 86400
        finally:
            if time_slided_on_dut is not None:
                SystemUtils.set_system_time_slide_seconds(-time_slided_on_dut)

    def test_system_time_update(self):
        result = SystemApiTestsInterface.run_instrumentation(class_name="CommonTestsDriver",
                                                             method_name="testDisableAutoTime",
                                                             instrumentation_args=None,
                                                             runner_name="GenericArgumentPassingTestRunner")
        self.assertTrue(SystemApiTestsInterface.was_instrumentation_test_successful(result),
                        "Could not disable auto time sync")
        initial_system_time = SystemUtils.get_system_time()
        LOG.info("initial time: " + str(initial_system_time))
        initial_hour_format = SystemUtils.get_hour_format()
        LOG.info("initial hour format: " + str(initial_hour_format))
        new_system_time = initial_system_time.copy()
        new_system_time.min = 0
        new_system_time.day = 2
        new_system_time.time_zone = "Cuba"
        hour_format_toggle = {12: 24, 24: 12}
        new_hour_format = hour_format_toggle[initial_hour_format]

        try:
            self.assertTrue(SystemUtils.set_hour_format(new_hour_format), "could not set new hour format")
            self.assertTrue(SystemUtils.set_system_time(new_system_time), "could not set new System time")
            reboot_time = UiAutomatorUtils.reboot_device()
            LOG.info("reboot time was: " + str(reboot_time))
            UiAutomatorUtils.unlock_screen()
            system_time_after_reboot = SystemUtils.get_system_time()
            LOG.info("after reboot time: " + str(system_time_after_reboot))
            hour_format_after_reboot = SystemUtils.get_hour_format()
            LOG.info("after reboot hour format: " + str(hour_format_after_reboot))
            self.assertTrue(system_time_after_reboot.time_zone == new_system_time.time_zone,
                            "timezone did not persist after reboot")
            self.assertTrue(abs(system_time_after_reboot.day - new_system_time.day) < 2,
                            "system day did not persist after reboot")
            self.assertTrue(system_time_after_reboot.min <= new_system_time.min + 2 * reboot_time / 60 + 5,
                            "system minute was affected by the reboot")
            self.assertTrue(hour_format_after_reboot == new_hour_format,
                            "system hour format was affected by the reboot")
        finally:
            SystemUtils.set_hour_format(initial_hour_format)
            SystemUtils.set_system_time(initial_system_time)

    def test_data_usage_new_installed_app(self):
        ApiTestsInterface.args_template = Template('$args')
        instrumentation_args = '-e apTestPage "https://www.youtube.com/watch?v=wCQcMKzxNY4" -e waitBeforeLogCheck 15000'
        surf_internet_result = ApiTestsInterface.run_instrumentation(class_name="WebViewTestsDriver",
                                                                     method_name="testStreamPlayback",
                                                                     instrumentation_args=instrumentation_args,
                                                                     runner_name="WifiEncryptionTestRunner")
        self.assertTrue(ApiTestsInterface.was_instrumentation_test_successful(surf_internet_result),
                        "Internet browsing was not successful")
        try:
            data_usage = Settings.get_data_usage_values()
            Settings.go_to_main_screen()
            Settings.disable_wifi()
            Settings.go_to_main_screen()
            time.sleep(5)
            final_data_usage = Settings.get_data_usage_values()
            self.assertTrue("ApiTests" in data_usage.keys(), "could not find browsing app in initial data usage stats")
            LOG.info("initial ApiTests data usage: " + str(data_usage["ApiTests"]))
            self.assertTrue("ApiTests" in final_data_usage.keys(),
                            "could not find browsing app in final data usage stats")
            LOG.info("final ApiTests data usage: " + str(final_data_usage["ApiTests"]))
        finally:
            Settings.go_to_main_screen()
            Settings.enable_wifi()

    def test_show_GPU_view_updates(self):

        def get_gpu_update_pixels(x, y, color):
            return color[0] >= 245 and color[1] <= 150 and color[2] <= 150

        Settings.enable_gpu_show_updates()
        time.sleep(3)
        try:
            for i in range(5):
                self.screenshooter.take_screenshot()
                show_update_pixels = self.screenshooter.search_for_pixels(get_gpu_update_pixels)
                pixels_percent = 1.0 * len(show_update_pixels) / (self.screenshooter.screen_width *
                                                                  self.screenshooter.screen_height) * 100
                LOG.info("Percent of update pixels: " + str(pixels_percent))
                if pixels_percent >= 60:
                    return  # Number of update pixels greater than 60%. Test passed.
                ScreenSwiper.swipe_up()
            self.assertTrue(False, "Could not find any update pixels")
        finally:
            UiAutomatorUtils.close_all_tasks()
            Settings.enable_gpu_show_updates()

    def test_SmartLock_TrustedLocation(self):
        trusted_place = "Havana, Cuba"
        try:
            # Enable location
            LOG.info("Enabling location", self)
            Settings.enable_location()

            # Enable screen lock pin and add a trusted place (current location)
            LOG.info("Enabling PIN screen lock", self)
            Settings.enable_pin()
            LOG.info("Adding trusted place - current location", self)
            Settings.add_trusted_place()

            # Lock the screen
            LOG.info("Locking the screen", self)
            d.press.power()
            time.sleep(2)

            # Unlock the device
            LOG.info("Unlocking the screen", self)
            UiAutomatorUtils.unlock_screen()

            LOG.info("Checking the PIN keyguard in NOT present", self)
            assert not d(resourceId=ANDROID_KEYGUARD_PIN_VIEW_RESID).wait.exists(
                    timeout=5000), "The keyguard should not be present in the current trusted place"

            if d(resourceId=SETTINGS_TRUSTED_FIRST_TIME_RESID).wait.exists(timeout=5000):
                d(resourceId=SETTINGS_TRUSTED_FIRST_TIME_RESID).click()

            # Remove the current trusted place
            LOG.info("Removing the current trusted place", self)
            Settings.remove_trusted_place()

            # Add a trusted place in another location
            LOG.info("Adding trusted place - {0}".format(trusted_place), self)
            Settings.add_trusted_place(trusted_place)

            # Lock the screen
            LOG.info("Locking the screen", self)
            d.press.power()
            time.sleep(2)

            # Unlock the device
            LOG.info("unlocking the screen", self)
            UiAutomatorUtils.unlock_screen()

            LOG.info("Checking the PIN keyguard is present in a non-trusted place", self)
            assert d(resourceId=ANDROID_KEYGUARD_PIN_VIEW_RESID).wait.exists(
                    timeout=5000), "The keyguard should appear when not in a trusted location"

            # Input the PIN via shell
            AdbUtils.input_text(TEST_PIN)
            time.sleep(1)
            d.press.enter()
            time.sleep(3)

            # Remove the trusted place
            LOG.info("Removing the trusted place - {0}".format(trusted_place), self)
            Settings.remove_trusted_place(trusted_place)
        finally:
            UiAutomatorUtils.close_all_tasks()
            Settings.disable_location()
            Settings.disable_pin()
            UiAutomatorUtils.close_all_tasks()

    def test_multi_user_shell(self):
        # list current system users
        initial_users = SystemUserUtils.get_system_users()
        LOG.info("found initial users: ")
        for user in initial_users:
            LOG.info(user)
        self.assertTrue(len(initial_users) > 0, "there must be at least 1 initial user")

        # add a test user
        SystemUserUtils.create_user("system_os_test_user")
        subsequent_users = SystemUserUtils.get_system_users()
        LOG.info("found subsequent users: ")
        for user in subsequent_users:
            LOG.info(user)
        self.assertIsNotNone(SystemUserUtils.get_user_by_name("system_os_test_user"),
                             "could not create user")
        self.assertTrue(len(subsequent_users) > len(initial_users), "added user does not show in user list")

        # delete test user
        SystemUserUtils.delete_user(user_name="system_os_test_user")
        final_users = SystemUserUtils.get_system_users()
        LOG.info("found final users: ")
        for user in final_users:
            LOG.info(user)
        self.assertIsNone(SystemUserUtils.get_user_by_name("system_os_test_user"))
        self.assertTrue(len(subsequent_users) > len(final_users), "deleted user remains in user list")

    def test_check_multiple_sequential_reminders(self):
        events_added = []
        for i in range(6):
            title = "ApiTestsEvent" + str(i+1)
            description = "ApiTestsDescription"
            start_point_sec = 100 + 40 * i  # add reminder every 40 seconds
            duration_sec = 60
            event_added = Calendar.create_calendar_event(title, description, start_point_sec, duration_sec)
            LOG.info(str(title) + " event add was: " + str(event_added))
            if event_added:
                events_added.append(title)
        self.assertTrue(len(events_added) > 4, " there were not enough events added")
        try:
            notifications_elapsed = False
            for i in range(35):
                LOG.info("waiting for event notification, time elapsed: " + str(i*20) + " seconds")
                time.sleep(20)
                StatusBar.open_notifications()
                for event_title in events_added:
                    if d(textContains=event_title).wait.exists(timeout=1000):
                        LOG.info("notification for event %s appeared" % event_title)
                        events_added.remove(event_title)
                        LOG.info("notifications left: " + str(events_added))
                    if len(events_added) == 0:  # all events notifications appeared
                        notifications_elapsed = True
                d.press.home()
                if notifications_elapsed:
                    break
            self.assertTrue(len(events_added) == 0, "some added reminders did not yield notifications")
        finally:
            StatusBar.open_notifications()
            StatusBar.clear_notifications()

    def test_skia_lib_existance(self):
        try:
            check_so_lib_cmd = "ls /system/lib | grep skia"
            output = AdbUtils.run_adb_cmd(check_so_lib_cmd, add_ticks=False)
            self.assertTrue("libskia.so" in output, "could not find libskia.so in /system/lib")
            AdbUtils.pull("/system/bin/bootanimation", Environment.tmp_dir_path)
            ndk_depends_abs_path = os.path.join(Environment.tmp_dir_path, SystemOsTests.ndk_depends_bin)
            bootanimation_bin_path = os.path.join(Environment.tmp_dir_path, SystemOsTests.bootanimation_bin)
            output = ShellUtils.run_shell_cmd(ndk_depends_abs_path + " " + bootanimation_bin_path)
            self.assertTrue("libskia.so" in output, "could not find libskia.so in bootanimation dependecies")
        finally:
            ShellUtils.clean_local_dir(Environment.tmp_dir_path)

    def test_hwui_hwuitest(self):
        try:
            # this is the host path where the hwuitest bin resides
            hwuitest_file_path = os.path.join(Environment.tmp_dir_path, SystemOsTests.hwuitest_bin)
            hwuitest_dut_path = os.path.join(Environment.dut_tmp_dir, SystemOsTests.hwuitest_bin)
            # push the hwuitest bin on the dut
            AdbUtils.push(hwuitest_file_path, hwuitest_dut_path)
            hwuitest_cmd_template = Template(hwuitest_dut_path + " $testname")
            for test_name in ["shadowgrid", "rectgrid", "oval"]:
                # run the 3 available HWUI tests
                hwuitest_cmd = hwuitest_cmd_template.substitute(testname=test_name)
                result = AdbUtils.run_adb_cmd(hwuitest_cmd)
                self.assertTrue("Success!" in result, "hwuitest %s was not successful" % test_name)
        finally:
            ShellUtils.clean_local_dir(Environment.tmp_dir_path)

    def send_google_cloud_message_from_host(self, message_text):
        send_message_jar = os.path.join(Environment.tmp_dir_path, SystemOsTests.gcm_sender_jar)
        send_message_cmd = Template("java -jar " + send_message_jar + ' "$message"').substitute(message=message_text)
        result = ShellUtils.run_shell_cmd(send_message_cmd)
        self.assertTrue("message_id" in result, "could not send cloud message from host")

    def gen_unique_google_cloud_message(self):
        return "google cloud message " + CommonUtils.get_current_time_string()

    gcm_receiver_package = "gcm.play.android.samples.com.gcmquickstart"
    gcm_receiver_tag = "MyGcmListenerService"

    def wait_for_gcm_message(self, message_text):
        # wait for message to arrive on DUT
        for i in range(15):
            time.sleep(5)
            messages_received = AdbUtils.run_adb_cmd("logcat -d | grep " + SystemOsTests.gcm_receiver_tag,
                                                     adb_shell=False, add_ticks=False)
            if message_text in messages_received:
                LOG.info("google cloud message was received on DUT")
                return
        self.assertTrue(False, "did not receive any google cloud message")

    def test_google_cloud_messaging(self):
        try:
            message_text = self.gen_unique_google_cloud_message()
            AdbUtils.start_activity_with_package_name(SystemOsTests.gcm_receiver_package)
            self.assertTrue(d(packageName=SystemOsTests.gcm_receiver_package).wait.exists(timeout=20000),
                            "google cloud messaging receiver app failed to start on DUT")
            self.send_google_cloud_message_from_host(message_text)
            self.wait_for_gcm_message(message_text)
        finally:
            ShellUtils.clean_local_dir(Environment.tmp_dir_path)

    def test_google_cloud_messaging_disable_wifi(self):
        try:
            Settings.disable_wifi()
            time.sleep(4)  # wait for wifi to be switched off
            message_text = self.gen_unique_google_cloud_message()
            AdbUtils.start_activity_with_package_name(SystemOsTests.gcm_receiver_package)
            self.assertTrue(d(packageName=SystemOsTests.gcm_receiver_package).wait.exists(timeout=20000),
                            "google cloud messaging receiver app failed to start on DUT")
            self.send_google_cloud_message_from_host(message_text)
            time.sleep(5)  # wait a little before turning on wifi
            Settings.enable_wifi()
            self.wait_for_gcm_message(message_text)
        finally:
            Settings.enable_wifi()
            ShellUtils.clean_local_dir(Environment.tmp_dir_path)

    def test_google_cloud_messaging_web_browsing(self):
        try:
            message_text = self.gen_unique_google_cloud_message()
            AdbUtils.start_activity_with_package_name(SystemOsTests.gcm_receiver_package)
            self.assertTrue(d(packageName=SystemOsTests.gcm_receiver_package).wait.exists(timeout=20000),
                            "google cloud messaging receiver app failed to start on DUT")

            def browse_pages():
                url_list = ["https://www.google.de", "http://www.w3schools.com/",
                            "https://www.wikipedia.org/", "https://500px.com/"]
                Chrome.launch()
                for url in url_list:
                    Chrome.go_to_url(url)
                    time.sleep(3)

            def send_gcm_message():
                time.sleep(8)
                self.send_google_cloud_message_from_host(message_text)

            browse_web_thread = Thread(target=browse_pages)
            send_gcm_message_thread = Thread(target=send_gcm_message)
            browse_web_thread.start()
            time.sleep(5)
            send_gcm_message_thread.start()
            browse_web_thread.join()
            send_gcm_message_thread.join()

            self.wait_for_gcm_message(message_text)
        finally:
            ShellUtils.clean_local_dir(Environment.tmp_dir_path)

    immersive_3d_app_package = "com.imangi.templerun"

    def test_immersive_mode_3D_game(self):
        AdbUtils.start_activity_with_package_name(SystemOsTests.immersive_3d_app_package)
        self.assertTrue(d(packageName=SystemOsTests.immersive_3d_app_package).wait.exists(timeout=20000),
                        "3D app failed to start on DUT")
        width, height = DeviceInfo.get_device_screen_dimensions()
        # start the game by clicking randomly
        time.sleep(3)
        for i in range(1, height, height/10):
            AdbUtils.tap(width/2, i)
            time.sleep(2)
        for i in range(5):
            time.sleep(1)
            if i%2 == 0:
                ScreenSwiper.swipe_left()
            else:
                ScreenSwiper.swipe_right()
        StatusBar.open_notifications(2)
        self.assertTrue(d(packageName=STATUS_BAR_PACKAGE).wait.exists(timeout=10000),
                        "system bar could not be opened")
        d.press.back()
        self.screenshooter.take_screenshot()
        # crop a small upper corner area area
        system_bar_obscured_crop = self.screenshooter.crop_upper_right_corner(25)
        system_bar_hidden = False
        for i in range(5):
            time.sleep(5)
            self.screenshooter.take_screenshot()
            system_bar_hidden_crop = self.screenshooter.crop_upper_right_corner(25)
            if not self.screenshooter.compare_color_lists(system_bar_hidden_crop, system_bar_obscured_crop):
                system_bar_hidden = True
                break
        self.assertTrue(system_bar_hidden, "system bar did not dissapear after 25 seconds")

    # ============================================================================================================
    # HWUI Optimization test

    dut_atrace_log_path = "/data/local/tmp/atrace.log"
    host_atrace_task_limits_file = "atrace_task_limits.log"
    task_limit_marker = "tracing_mark_write"
    host_atrace_log_path = None

    def delete_atrace_log(self):
        AdbUtils.delete_files(SystemOsTests.dut_atrace_log_path)

    def atrace_capture(self, capture_time=3, categories="sched am view gfx wm hal res dalvik rs"):
        cmd_template = Template("atrace -t $time $categories > " + SystemOsTests.dut_atrace_log_path)
        cmd = cmd_template.substitute(time=capture_time, categories=categories)
        AdbUtils.run_adb_cmd(cmd)

    def pull_atrace_log(self):
        AdbUtils.pull(SystemOsTests.dut_atrace_log_path, Environment.tmp_dir_path)
        SystemOsTests.host_atrace_log_path = os.path.join(Environment.tmp_dir_path,
                                                          os.path.basename(SystemOsTests.dut_atrace_log_path))

    def gen_task_limits_file(self):
        cmd = "cat " + SystemOsTests.host_atrace_log_path + " | grep " + SystemOsTests.task_limit_marker + \
              " > " + os.path.join(Environment.tmp_dir_path, SystemOsTests.host_atrace_task_limits_file)
        ShellUtils.run_shell_cmd(cmd)

    def parse_atrace_limits_log(self, file_abs_path, task_name):
        task_start_regex = "B\|\d+\|"
        task_change_regex = "C\|\d+\|"
        task_end_regex = ": E$"
        task_start_detected = False
        tasks_started_after = []
        tasks_changed_after = []
        current_tasks_stack = []

        def get_task_info_from_line(current_line):
            task_proc = re.findall("^([^-]+)", current_line)[0]
            task_name = re.findall(": (.+)$", current_line)[0]
            return task_proc, task_name

        def parse_line_after_task_detected(current_line):
            subseq_task_start = re.findall(task_start_regex, current_line)
            subseq_task_change = re.findall(task_change_regex, current_line)
            subseq_task_end = re.findall(task_end_regex, current_line)
            current_task_proc, current_task_name = get_task_info_from_line(current_line)
            if len(subseq_task_start) > 0:
                tasks_started_after.append((current_task_proc, current_task_name))
                current_tasks_stack.append((current_task_proc, current_task_name))
            elif len(subseq_task_change) > 0:
                tasks_changed_after.append((current_task_proc, current_task_name))
                # current_tasks_stack.append((current_task_proc, current_task_name))
            elif len(subseq_task_end) > 0:
                if len(current_tasks_stack) > 0:
                    current_tasks_stack.pop()

        with open(file_abs_path) as f:
            for file_line in f:
                task_start = re.findall(task_start_regex + task_name, file_line)
                if len(task_start) > 0:
                    task_start_detected = True
                elif task_start_detected:
                    parse_line_after_task_detected(file_line)
                if len(current_tasks_stack) < 1:
                    break
            return tasks_started_after, tasks_changed_after

    def test_hwui_optimization(self):
        def run_atrace():
            self.atrace_capture()

        def start_settings_app():
            AdbUtils.start_activity_with_package_name(SETTINGS_PACKAGE_NAME)

        try:
            atrace_thread = Thread(target=run_atrace)
            start_settings_thread = Thread(target=start_settings_app)
            LOG.info("Starting atrace on dut")
            atrace_thread.start()
            time.sleep(0.1)  # allow atrace thread to start first
            LOG.info("Starting settings app on dut")
            start_settings_thread.start()
            start_settings_thread.join()
            atrace_thread.join()
            LOG.info("Atrace finished ...")

            LOG.info("pulling atrace_log ...")
            self.pull_atrace_log()
            self.gen_task_limits_file()

            # =========================================== setSurface asynchronicity
            LOG.info("testing setSurface is async ...")
            tasks_started, tasks_changed = self.parse_atrace_limits_log(os.path.join(Environment.tmp_dir_path,
                                                                        SystemOsTests.host_atrace_task_limits_file),
                                                                        "setSurface")
            set_surface_is_async = False
            for task_info in tasks_started:
                if ".settings" in task_info[0]:
                    LOG.info("found settings task executing while setSurface: " + str(task_info))
                    set_surface_is_async = True
            self.assertTrue(set_surface_is_async, "no settings tasks were detected during setSurface call")

            # =========================================== allocateBuffers asynchronicity
            LOG.info("testing allocateBuffers is async ...")
            tasks_started, tasks_changed = self.parse_atrace_limits_log(os.path.join(Environment.tmp_dir_path,
                                                                        SystemOsTests.host_atrace_task_limits_file),
                                                                        "allocateBuffers")
            allocate_buffers_is_async = False
            for task_info in tasks_started:
                if ".settings" in task_info[0]:
                    LOG.info("found settings task executing while allocateBuffers: " + str(task_info))
                    set_surface_is_async = True
            self.assertTrue(allocate_buffers_is_async, "no settings tasks were detected during allocateBuffers call")

        finally:
            self.delete_atrace_log()
            # Environment.clean_tmp_dir()

    # HWUI Optimization test end
    # ============================================================================================================

    memtrack_loop_cmd = "while :; do cat /sys/class/drm/card0/gfx_memtrack/i915_gem_meminfo > /dev/null; done"
    memtrack_proc = None
    jank_test_output = None

    def test_memtrack_effect_framerate(self):

        def run_memtrack():
            global memtrack_proc
            SystemOsTests.memtrack_proc = AdbUtils.get_adb_cmd_process(SystemOsTests.memtrack_loop_cmd)

        def run_jank_test():
            jank_runner = InstrumentationInterface()
            SystemOsTests.jank_test_output = jank_runner.run_instrumentation("android.cts.jank.ui.CtsDeviceJankUi", "",
                                                      "android.cts.jank/android.support.test.runner.AndroidJUnitRunner")

        # must stop python uiautomator in order to run the jank test
        AdbUtils.kill_python_uiautomator_rpc_server_on_dut()

        # initial run of jank test
        run_jank_test()
        LOG.info("initial run of the jank test yielded: " + SystemOsTests.jank_test_output)
        self.assertTrue(InstrumentationInterface.was_instrumentation_test_successful(SystemOsTests.jank_test_output),
                        "initial run of the jank test was not successful")
        fps_values = re.findall("\|fps\|(\d+\.\d+)", SystemOsTests.jank_test_output)
        self.assertTrue(len(fps_values) > 0, "could not find fps information in jank test output")
        initial_fps_value = float(fps_values[0])

        UiAutomatorUtils.close_all_tasks()
        # must stop python uiautomator in order to run the jank test
        AdbUtils.kill_python_uiautomator_rpc_server_on_dut()

        # subsequent run of the jank test, with memtrack running along side
        memtrack_thread = Thread(target=run_memtrack)
        jank_thread = Thread(target=run_jank_test)
        LOG.info("Starting memtrack")
        memtrack_thread.start()
        time.sleep(0.5)  # allow memtrack to start before running the jank test
        LOG.info("Starting jank ui on dut")
        jank_thread.start()
        jank_thread.join()
        SystemOsTests.memtrack_proc.kill()  # kill memtrack so the thread will join
        memtrack_thread.join()

        LOG.info("second run of the jank test yielded: " + SystemOsTests.jank_test_output)
        self.assertTrue(InstrumentationInterface.was_instrumentation_test_successful(SystemOsTests.jank_test_output),
                        "second run of the jank test was not successful")
        fps_values = re.findall("\|fps\|(\d+\.\d+)", SystemOsTests.jank_test_output)
        self.assertTrue(len(fps_values) > 0, "could not find fps information in jank test output")
        second_fps_value = float(fps_values[0])

        LOG.info("initial fps value: %s, second fps value: %s" % (initial_fps_value, second_fps_value))
        self.assertTrue(second_fps_value < initial_fps_value, "fps when running memtrack was not smaller than "
                                                              "fps without running memtrack")


if __name__ == "__main__":
    test_result = SingleMethodRunner.run_single_test(SystemOsTests, "test_verify_20_recent_apps")
    print test_result.wasSuccessful()
