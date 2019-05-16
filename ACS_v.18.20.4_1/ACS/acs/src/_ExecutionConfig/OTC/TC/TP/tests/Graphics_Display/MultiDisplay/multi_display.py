# -*- coding: utf-8 -*-

from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.common import multi_display, dbsetting, file_sys, adb32, launch_settings_am
from testlib.graphics.compare_pic_impl import compare_pic
from testlib.graphics.photos_impl import get_photo_implement
from testlib.util.common import g_common_obj


class MultiDisplay(UIATestBase):

    def setUp(self):
        super(MultiDisplay, self).setUp()
        self.d = g_common_obj.get_device()
        self.do_initiation = False

    def tearDown(self):
        super(MultiDisplay, self).tearDown()

    def test_2SimultaneousDisplays_HDMI1_HDMI2(self):
        multi_display.is_multi_displayed()
        croped_pri_image = multi_display.get_screenshot_multidisplay_croped(multi_display.PRIMARY_DISPLAY_ID)
        croped_ext_image = multi_display.get_screenshot_multidisplay_croped(multi_display.EXTERNAL_DISPLAY_ID)
        rms = compare_pic.compare_pic(croped_pri_image, croped_ext_image)
        assert rms < 2, "Not same screen between two displays."

    def test_Multiple_Display_UI_and_Instrument_Cluster_IC_dispaly(self):
        try:
            multi_display.is_multi_displayed()
            get_photo_implement()
            dbsetting.set_force_resizable_activities(enabled=True)
            multi_display.run_activities_multidisplay(multi_display.EXTERNAL_DISPLAY_ID,
                                                      "com.google.android.apps.photos",
                                                      ".home.HomeActivity")
            multi_display.is_show_different_picture()
            dbsetting.set_force_resizable_activities(enabled=False)
        except:
            raise Exception("Activity not started on external display.")
        finally:
            g_common_obj.adb_cmd_capture_msg("am force-stop com.google.android.apps.photos")

    def test_Multiple_Display_UI_and_Rear_Seat_Entertainment_RSE_dispaly(self):
        try:
            multi_display.is_multi_displayed()
            get_photo_implement()
            dbsetting.set_force_resizable_activities(enabled=True)
            multi_display.run_activities_multidisplay(multi_display.EXTERNAL_DISPLAY_ID,
                                                      "com.google.android.apps.photos",
                                                      ".home.HomeActivity")
            multi_display.is_show_different_picture()
            dbsetting.set_force_resizable_activities(enabled=False)
        except:
            raise Exception("Activity not started on external display.")
        finally:
            g_common_obj.adb_cmd_capture_msg("am force-stop com.google.android.apps.photos")

    def test_Multiple_Display_UI_and_Rear_Seat_Entertainment_RSE_touchscreen(self):
        try:
            multi_display.is_multi_displayed()
            get_photo_implement()
            dbsetting.set_force_resizable_activities(enabled=True)
            multi_display.run_activities_multidisplay(multi_display.EXTERNAL_DISPLAY_ID,
                                                      "com.google.android.apps.photos",
                                                      ".home.HomeActivity")
            multi_display.is_show_different_picture()
            dbsetting.set_force_resizable_activities(enabled=False)
        except:
            raise Exception("Activity not started on external display.")
        finally:
            g_common_obj.adb_cmd_capture_msg("am force-stop com.google.android.apps.photos")

    def test_Multi_display_display0_doesnot_rotation_display1_rotation_default(self):
        try:
            multi_display.is_multi_displayed()
            file_sys.switch_hwc_display('ROTATION', 'true')
            file_sys.switch_hwc_display('PHYSICAL_DISPLAY_ROTATION', '1:4')
            adb32.refresh_ui()
            launch_settings_am()
            multi_display.is_clone_mode()
            file_sys.init_hwc_display()
            self.do_initiation = multi_display.is_clone_mode()
            raise Exception('Need BKM for checking multi display orientation status.')
        except:
            raise Exception('Test failed, some problems occurred on above steps.')
        finally:
            if not self.do_initiation:
                file_sys.init_hwc_display()

    def test_Multi_display_display0_doesnot_rotation_display1_rotation_180(self):
        try:
            multi_display.is_multi_displayed()
            file_sys.switch_hwc_display('ROTATION', 'true')
            file_sys.switch_hwc_display('PHYSICAL_DISPLAY_ROTATION', '1:2')
            adb32.refresh_ui()
            launch_settings_am()
            multi_display.is_show_different_picture()
            file_sys.init_hwc_display()
            self.do_initiation = multi_display.is_clone_mode()
            raise Exception('Need BKM for checking multi display orientation status.')
        except:
            raise Exception('Test failed, some problems occurred on above steps.')
        finally:
            if not self.do_initiation:
                file_sys.init_hwc_display()

    def test_Two_Displays_Extended_Desktop_HDMI1_HDMI2(self):
        try:
            multi_display.is_multi_displayed()
            file_sys.switch_hwc_display('MOSAIC', 'true')
            file_sys.switch_hwc_display('MOSAIC_DISPLAY', '0+1+2')
            adb32.refresh_ui()
            launch_settings_am()
            multi_display.is_show_different_picture()
            file_sys.init_hwc_display()
            self.do_initiation = multi_display.is_clone_mode()
            raise Exception('Need BKM for checking multi display orientation status.')
        except:
            raise Exception('Test failed, some problems occurred on above steps.')
        finally:
            if not self.do_initiation:
                file_sys.init_hwc_display()

    def test_Multi_display_display0_doesnot_rotation_display1_rotation_270(self):
        try:
            multi_display.is_multi_displayed()
            file_sys.switch_hwc_display('ROTATION', 'true')
            file_sys.switch_hwc_display('PHYSICAL_DISPLAY_ROTATION', '1:3')
            adb32.refresh_ui()
            launch_settings_am()
            multi_display.is_show_different_picture()
            file_sys.init_hwc_display()
            self.do_initiation = multi_display.is_clone_mode()
            raise Exception('Need BKM for checking multi display orientation status.')
        except:
            raise Exception('Test failed, some problems occurred on above steps.')
        finally:
            if not self.do_initiation:
                file_sys.init_hwc_display()

    def test_Multi_display_doesnot_rotation_display1_rotation_90(self):
        try:
            multi_display.is_multi_displayed()
            file_sys.switch_hwc_display('ROTATION', 'true')
            file_sys.switch_hwc_display('PHYSICAL_DISPLAY_ROTATION', '1:1')
            adb32.refresh_ui()
            launch_settings_am()
            multi_display.is_show_different_picture()
            file_sys.init_hwc_display()
            self.do_initiation = multi_display.is_clone_mode()
            raise Exception('Need BKM for checking multi display orientation status.')
        except:
            raise Exception('Test failed, some problems occurred on above steps.')
        finally:
            if not self.do_initiation:
                file_sys.init_hwc_display()

    def test_Multi_display_rotation_180_display1_doesnot_rotation(self):
        try:
            multi_display.is_multi_displayed()
            file_sys.switch_hwc_display('ROTATION', 'true')
            file_sys.switch_hwc_display('PHYSICAL_DISPLAY_ROTATION', '0:2')
            adb32.refresh_ui()
            launch_settings_am()
            multi_display.is_show_different_picture()
            file_sys.init_hwc_display()
            self.do_initiation = multi_display.is_clone_mode()
            raise Exception('Need BKM for checking multi display orientation status.')
        except:
            raise Exception('Test failed, some problems occurred on above steps.')
        finally:
            if not self.do_initiation:
                file_sys.init_hwc_display()

    def test_Multi_display_rotation_270_display1_doesnot_rotation(self):
        try:
            multi_display.is_multi_displayed()
            file_sys.switch_hwc_display('ROTATION', 'true')
            file_sys.switch_hwc_display('PHYSICAL_DISPLAY_ROTATION', '0:3')
            adb32.refresh_ui()
            launch_settings_am()
            multi_display.is_show_different_picture()
            file_sys.init_hwc_display()
            self.do_initiation = multi_display.is_clone_mode()
            raise Exception('Need BKM for checking multi display orientation status.')
        except:
            raise Exception('Test failed, some problems occurred on above steps.')
        finally:
            if not self.do_initiation:
                file_sys.init_hwc_display()

    def test_Multi_display_rotation_90_display1_doesnot_rotation(self):
        try:
            multi_display.is_multi_displayed()
            file_sys.switch_hwc_display('ROTATION', 'true')
            file_sys.switch_hwc_display('PHYSICAL_DISPLAY_ROTATION', '0:1')
            adb32.refresh_ui()
            launch_settings_am()
            multi_display.is_show_different_picture()
            file_sys.init_hwc_display()
            self.do_initiation = multi_display.is_clone_mode()
            raise Exception('Need BKM for checking multi display orientation status.')
        except:
            raise Exception('Test failed, some problems occurred on above steps.')
        finally:
            if not self.do_initiation:
                file_sys.init_hwc_display()

    def test_Multi_display_rotation_default_display1_doesnot_rotation(self):
        try:
            multi_display.is_multi_displayed()
            file_sys.switch_hwc_display('ROTATION', 'true')
            file_sys.switch_hwc_display('PHYSICAL_DISPLAY_ROTATION', '0:4')
            adb32.refresh_ui()
            launch_settings_am()
            multi_display.is_clone_mode()
            file_sys.init_hwc_display()
            self.do_initiation = multi_display.is_clone_mode()
            raise Exception('Need BKM for checking multi display orientation status.')
        except:
            raise Exception('Test failed, some problems occurred on above steps.')
        finally:
            if not self.do_initiation:
                file_sys.init_hwc_display()
