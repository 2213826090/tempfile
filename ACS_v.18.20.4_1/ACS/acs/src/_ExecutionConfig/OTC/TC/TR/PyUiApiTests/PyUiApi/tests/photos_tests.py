from PyUiApi.app_utils.photos_utils import *
from PyUiApi.app_utils.chrome_utils import *
from PyUiApi.common.system_utils import *
from PyUiApi.app_utils.settings_utils import *
from threading import Thread
from PyUiApi.common.environment_utils import *


class PhotosTests(unittest.TestCase):
    color_white = (255, 255, 255)

    def setUp(self):
        self.assertTrue(SystemUtils.is_internet_connection_active())
        Photos.refresh_media_store()
        UiAutomatorUtils.unlock_screen()
        self.initial_orientation = d.orientation
        self.screenshot_utils = ScreenshotUtils()
        Photos.launch()

    def tearDown(self):
        self.log_before_cleanup()
        UiAutomatorUtils.close_all_tasks()
        if d.orientation != self.initial_orientation:
            d.orientation = self.initial_orientation
            d.freeze_rotation(False)
        self.screenshot_utils.remove_all_screenshots()

    def wait_for_media_store_refresh(self, timeout):
        time.sleep(timeout)

    def log_before_cleanup(self):
        result = self._resultForDoCleanups
        if not result.wasSuccessful():
            TestUtils.log_screenshot_and_dump()

    def test_crop_image(self):
        Photos.click_all_tab()
        Photos.click_first_photo()
        initial_details = Photos.get_media_details()
        d.press.back()
        Photos.edit_current_photo()
        Photos.select_crop_option()
        time.sleep(4)
        Photos.select_free_crop()
        self.screenshot_utils.take_screenshot()
        x, y = self.screenshot_utils.search_for_first_pixel_of_color_in_column(self.color_white,
                                                                               self.screenshot_utils.screen_width / 2)
        d.swipe(x, y, x, y + 100)
        time.sleep(2)
        Photos.apply_changes()
        Photos.select_done_edit()
        Photos.return_to_main_screen()
        Photos.click_first_photo()
        free_crop_details = Photos.get_media_details()
        d.press.back()
        Photos.edit_current_photo()
        Photos.select_crop_option()
        time.sleep(4)
        Photos.select_square_crop()
        self.screenshot_utils.take_screenshot()
        x, y = self.screenshot_utils.search_for_first_pixel_of_color_in_column(self.color_white,
                                                                               self.screenshot_utils.screen_width / 2)
        d.swipe(x, y, x, y + 100)
        time.sleep(2)
        Photos.apply_changes()
        Photos.select_done_edit()
        Photos.return_to_main_screen()
        Photos.click_first_photo()
        square_crop_details = Photos.get_media_details()
        print "initial details", initial_details
        print "free crop details", free_crop_details
        print "square crop details", square_crop_details
        initial_width, initial_height = Photos.get_size_from_details(initial_details)
        free_width, free_height = Photos.get_size_from_details(free_crop_details)
        square_width, square_height = Photos.get_size_from_details(square_crop_details)
        self.assertTrue(initial_height > free_height)
        self.assertTrue(free_height > square_height)
        # it sometimes happen that the square edges differ slightly
        self.assertTrue(abs(square_height - square_width) < 3)

    def test_timeline_media(self):
        Photos.click_all_tab()
        Photos.click_first_photo()
        first_photo_detail = Photos.get_media_details()
        Photos.return_to_main_screen()
        Photos.click_all_tab()
        photos = Photos.get_photos()
        self.assertTrue(len(photos) > 1)
        photos[len(photos) - 1].click()
        last_photo_detail = Photos.get_media_details()
        Photos.return_to_main_screen()
        first_photo_date = Photos.get_date_from_details(first_photo_detail)
        last_photo_date = Photos.get_date_from_details(last_photo_detail)
        # photos should be displayed latest first
        self.assertTrue(first_photo_date > last_photo_date)

    def test_cancel_changes(self):
        Photos.click_all_tab()
        Photos.click_first_photo()
        Photos.edit_current_photo()
        Photos.select_filter(PHOTOS_BLACK_AND_WHITE_FILTER_DESC)
        # check center picture to see if it is black and white
        self.screenshot_utils.take_screenshot()
        center_pixels = self.screenshot_utils.crop_center_square_pixels(200)
        for p in center_pixels:
            r, g, b = p
            self.assertTrue(r == g and g == b)
        Photos.cancel_changes()
        # check center picture to see if it is back to color
        self.screenshot_utils.take_screenshot()
        center_pixels = self.screenshot_utils.crop_center_square_pixels(200)
        color_detected = False
        for p in center_pixels:
            r, g, b = p
            if r != g or g != b or r != b:
                color_detected = True
                break
        self.assertTrue(color_detected)
        Photos.return_to_main_screen()

    def test_auto_filter(self):
        Photos.click_all_tab()
        Photos.click_first_photo()
        Photos.edit_current_photo()
        Photos.select_filter(PHOTOS_AUTO_FILTER_DESC)
        Photos.set_auto_filter_off()
        time.sleep(5)  # wait for filter to be applied
        self.screenshot_utils.take_screenshot()
        off_pixels = self.screenshot_utils.crop_center_square_pixels(500)
        Photos.set_auto_filter_normal()
        time.sleep(5)  # wait for filter to be applied
        self.screenshot_utils.take_screenshot()
        normal_pixels = self.screenshot_utils.crop_center_square_pixels(500)
        Photos.set_auto_filter_high()
        time.sleep(5)  # wait for filter to be applied
        self.screenshot_utils.take_screenshot()
        high_pixels = self.screenshot_utils.crop_center_square_pixels(500)
        self.assertTrue(off_pixels != normal_pixels and normal_pixels != high_pixels and off_pixels != high_pixels)

    def test_discard_changes(self):
        Photos.click_all_tab()
        Photos.click_first_photo()
        initial_details = Photos.get_media_details()
        d.press.back()
        Photos.edit_current_photo()
        Photos.select_filter(PHOTOS_AUTO_FILTER_DESC)
        Photos.set_auto_filter_high()
        time.sleep(5)  # wait for filter to be applied
        Photos.apply_changes()
        time.sleep(3)
        d.press.back()
        UiAutomatorUtils.click_view_with_text("Exit")
        Photos.return_to_main_screen()
        Photos.click_first_photo()
        final_details = Photos.get_media_details()
        self.assertTrue(initial_details == final_details)
        Photos.return_to_main_screen()

    def test_delete_photo(self):
        Photos.click_all_tab()
        Photos.click_first_photo()
        initial_details = Photos.get_media_details()
        d.press.back()
        Photos.delete_current_photo()
        Photos.return_to_main_screen()
        Photos.click_first_photo()
        final_details = Photos.get_media_details()
        self.assertFalse(initial_details == final_details)
        Photos.return_to_main_screen()
        Photos.open_navigation_drawer()
        Photos.open_trash()
        Photos.long_click_first_photo()
        Photos.restore_selected_photo()
        Photos.open_navigation_drawer()
        Photos.open_photos()
        Photos.click_first_photo()
        last_final_details = Photos.get_media_details()
        self.assertTrue(initial_details == last_final_details)
        Photos.return_to_main_screen()

    def test_add_looks(self):
        Photos.click_all_tab()
        Photos.click_first_photo()
        Photos.edit_current_photo()
        self.screenshot_utils.take_screenshot()
        original_pixels = self.screenshot_utils.crop_center_square_pixels(500)
        Photos.select_filter(PHOTOS_LOOKS_FILTER_DESC)
        for i in range(PHOTOS_LOOKS_FILTER_INSTANCES):
            Photos.select_looks_filter_instance(i)
            time.sleep(1)  # wait for filter to be applied
            self.screenshot_utils.take_screenshot()
            instance_pixels = self.screenshot_utils.crop_center_square_pixels(500)
            self.assertTrue(instance_pixels != original_pixels)
        Photos.return_to_main_screen()

    def test_add_filters(self):
        Photos.click_all_tab()
        Photos.click_first_photo()

        # Add the VINTAGE filter
        Photos.edit_current_photo()
        self.screenshot_utils.take_screenshot()
        original_pixels = self.screenshot_utils.crop_center_square_pixels(500)
        Photos.select_filter(PHOTOS_VINTAGE_FILTER_DESC)
        UiAutomatorUtils.click_view_with_text(PHOTOS_FILTER_STYLE_BUTTON_TXT)
        for i in range(1, PHOTOS_VINTAGE_FILTER_INSTANCES + 1):
            UiAutomatorUtils.click_view_with_text(str(i))
            time.sleep(1)  # wait for filter to be applied
            self.screenshot_utils.take_screenshot()
            instance_pixels = self.screenshot_utils.crop_center_square_pixels(500)
            self.assertTrue(original_pixels != instance_pixels)
        Photos.return_to_selected_photo()

        # Add the DRAMA filter
        Photos.edit_current_photo()
        Photos.select_filter(PHOTOS_DRAMA_FILTER_DESC)
        UiAutomatorUtils.click_view_with_text(PHOTOS_FILTER_STYLE_BUTTON_TXT)
        for i in PHOTOS_DRAMA_FILTER_STYLES_TXTS:
            UiAutomatorUtils.click_view_with_text(i)
            time.sleep(1)  # wait for filter to be applied
            self.screenshot_utils.take_screenshot()
            instance_pixels = self.screenshot_utils.crop_center_square_pixels(500)
            self.assertTrue(original_pixels != instance_pixels)
        Photos.return_to_selected_photo()

        # Add the BW filter
        Photos.edit_current_photo()
        Photos.select_filter(PHOTOS_BW_FILTER_DESC)
        UiAutomatorUtils.click_view_with_text(PHOTOS_FILTER_STYLE_BUTTON_TXT)
        for i in PHOTOS_BW_FILTER_STYLES_TXTS:
            UiAutomatorUtils.click_view_with_text(i)
            time.sleep(1)  # wait for filter to be applied
            self.screenshot_utils.take_screenshot()
            instance_pixels = self.screenshot_utils.crop_center_square_pixels(500)
            self.assertTrue(original_pixels != instance_pixels)
        Photos.return_to_selected_photo()

        # Add the HDR filter
        Photos.edit_current_photo()
        Photos.select_filter(PHOTOS_HDR_FILTER_DESC)
        UiAutomatorUtils.click_view_with_text(PHOTOS_FILTER_STYLE_BUTTON_TXT)
        for i in PHOTOS_HDR_FILTER_STYLES_TXTS:
            UiAutomatorUtils.click_view_with_text(i)
            time.sleep(1)  # wait for filter to be applied
            self.screenshot_utils.take_screenshot()
            instance_pixels = self.screenshot_utils.crop_center_square_pixels(500)
            self.assertTrue(original_pixels != instance_pixels)
        Photos.return_to_selected_photo()

        # Add the RETROLUX filter
        Photos.edit_current_photo()
        self.screenshot_utils.take_screenshot()
        original_pixels = self.screenshot_utils.crop_center_square_pixels(500)
        Photos.select_filter(PHOTOS_RETROLUX_FILTER_DESC)
        UiAutomatorUtils.click_view_with_text(PHOTOS_FILTER_STYLE_BUTTON_TXT)
        for i in range(1, PHOTOS_RETROLUX_FILTER_INSTANCES + 1):
            UiAutomatorUtils.click_view_with_text(str(i))
            time.sleep(1)  # wait for filter to be applied
            self.screenshot_utils.take_screenshot()
            instance_pixels = self.screenshot_utils.crop_center_square_pixels(500)
            self.assertTrue(original_pixels != instance_pixels)
        Photos.return_to_selected_photo()

        # Add the CENTER FOCUS filter
        Photos.edit_current_photo()
        Photos.select_filter(PHOTOS_CENTER_FOCUS_FILTER_DESC)
        UiAutomatorUtils.click_view_with_text(PHOTOS_FILTER_STYLE_BUTTON_TXT)
        for i in PHOTOS_CENTER_FOCUS_FILTER_STYLES_TXTS:
            UiAutomatorUtils.click_view_with_text(i)
            time.sleep(1)  # wait for filter to be applied
            self.screenshot_utils.take_screenshot()
            instance_pixels = self.screenshot_utils.crop_center_square_pixels(500)
            self.assertTrue(original_pixels != instance_pixels)
        Photos.return_to_selected_photo()

    def test_rotate_image(self):
        Photos.click_all_tab()
        Photos.click_first_photo()
        Photos.edit_current_photo()
        self.screenshot_utils.take_screenshot()
        original_pixels = self.screenshot_utils.crop_center_square_pixels(500)
        Photos.select_filter(PHOTOS_ROTATE_FILTER_DESC)

        # Rotate right 90 degrees
        UiAutomatorUtils.click_view_with_text(PHOTOS_ROTATE_RIGHT_BUTTON_TXT)
        time.sleep(1)
        self.screenshot_utils.take_screenshot()
        right90_pixels = self.screenshot_utils.crop_center_square_pixels(500)
        self.assertTrue(original_pixels != right90_pixels)

        # Rotate right another 90 degrees
        UiAutomatorUtils.click_view_with_text(PHOTOS_ROTATE_RIGHT_BUTTON_TXT)
        time.sleep(1)
        self.screenshot_utils.take_screenshot()
        right180_pixels = self.screenshot_utils.crop_center_square_pixels(500)
        self.assertTrue(original_pixels != right180_pixels and right90_pixels != right180_pixels)

        # Rotate left 90 degrees
        UiAutomatorUtils.click_view_with_text(PHOTOS_ROTATE_LEFT_BUTTON_TXT)
        time.sleep(1)
        self.screenshot_utils.take_screenshot()
        left_pixels = self.screenshot_utils.crop_center_square_pixels(500)
        self.assertTrue(original_pixels != left_pixels and right180_pixels != left_pixels)

        # Swipe up to rotate the image one degree at a time
        ScreenSwiper.swipe_up()
        time.sleep(1)
        self.screenshot_utils.take_screenshot()
        swiped_pixels = self.screenshot_utils.crop_center_square_pixels(500)
        self.assertTrue(original_pixels != swiped_pixels and left_pixels != swiped_pixels)
        Photos.return_to_main_screen()

    def test_add_frame(self):
        Photos.click_all_tab()
        Photos.click_first_photo()
        Photos.edit_current_photo()
        self.screenshot_utils.take_screenshot()
        original_pixels = self.screenshot_utils.crop_right_square_pixels(500)
        Photos.select_filter(PHOTOS_FRAME_FILTER_DESC)
        UiAutomatorUtils.click_view_with_text(PHOTOS_FRAME_STYLE_TXT)
        time.sleep(1)
        for i in range(1, PHOTOS_FRAME_STYLE_INSTANCES + 1):
            Photos.select_frame_instance(str(i))
            time.sleep(1)  # wait for filter to be applied
            self.screenshot_utils.take_screenshot()
            instance_pixels = self.screenshot_utils.crop_right_square_pixels(500)
            self.assertTrue(instance_pixels != original_pixels)
        Photos.return_to_main_screen()

    def test_prereq_sign_in(self):
        Photos.sign_in()
        Photos.open_menu()
        UiAutomatorUtils.click_view_with_text(PHOTOS_REFRESH_BUTTON_TXT)
        time.sleep(5)
        photos_list=Photos.get_photos()
        self.assertTrue(len(photos_list) >= 1)

    def test_copy_to_new_album(self):
        album_name = datetime.today().strftime('%d-%m-%Y-%H:%M')
        Photos.click_all_tab()
        Photos.copy_photo_to_new_album(album_name)
        Photos.open_navigation_drawer()
        Photos.open_albums()
        self.assertTrue(d(text=album_name).wait.exists(timeout=5000))
        UiAutomatorUtils.click_view_with_text(album_name)
        time.sleep(3)
        self.assertTrue(d(description=PHOTOS_ALBUM_FIRST_PHOTO_DESC).wait.exists(timeout=3000))
        Photos.open_menu()
        UiAutomatorUtils.click_view_with_text(PHOTOS_DELETE_ALBUM_BUTTON_TXT)
        UiAutomatorUtils.click_view_with_text(PHOTOS_DELETE_ALBUM_BUTTON_TXT)
        time.sleep(5)
        Photos.open_navigation_drawer()
        Photos.open_photos()

    def test_copy_to_existing_album(self):
        album_name = datetime.today().strftime('%d-%m-%Y-%H:%M')
        Photos.click_all_tab()
        Photos.copy_photo_to_new_album(album_name)
        Photos.click_all_tab()
        Photos.copy_photo_to_existing_album(album_name)
        Photos.open_navigation_drawer()
        Photos.open_albums()
        self.assertTrue(d(text=album_name).wait.exists(timeout=5000))
        UiAutomatorUtils.click_view_with_text(album_name)
        time.sleep(3)
        self.assertTrue(d(description=PHOTOS_ALBUM_FIRST_PHOTO_DESC).wait.exists(timeout=3000))
        self.assertTrue(d(description=PHOTOS_ALBUM_SECOND_PHOTO_DESC).wait.exists(timeout=3000))
        Photos.open_menu()
        UiAutomatorUtils.click_view_with_text(PHOTOS_DELETE_ALBUM_BUTTON_TXT)
        UiAutomatorUtils.click_view_with_text(PHOTOS_DELETE_ALBUM_BUTTON_TXT)
        time.sleep(5)
        Photos.open_navigation_drawer()
        Photos.open_photos()


class PhotosTestsM(PhotosTests):

    def setUp(self):
        self.assertTrue(SystemUtils.is_internet_connection_active())
        Photos.refresh_media_store(Environment.dcim_folder_path)
        UiAutomatorUtils.unlock_screen()
        self.initial_orientation = d.orientation
        self.screenshot_utils = ScreenshotUtils()
        Photos.launch()

    def test_crop_image(self):
        Photos.click_first_photo()
        self.assertTrue(d(resourceId=PHOTOS_PHOTO_DETAILS_OPTION_RESID).wait.exists(timeout=3000))
        self.assertTrue(d(resourceId=PHOTOS_PHOTO_EDIT_OPTION_RESID).wait.exists(timeout=3000))
        self.assertTrue(d(resourceId=PHOTOS_PHOTO_SHARE_OPTION_RESID).wait.exists(timeout=3000))
        self.assertTrue(d(resourceId=PHOTOS_PHOTO_TRASH_OPTION_RESID).wait.exists(timeout=3000))
        initial_details = Photos.get_media_details()
        d.press.back()
        Photos.edit_current_photo()
        Photos.select_crop_option()
        Photos.crop_current_photo(self.screenshot_utils, 100, 100)
        self.assertTrue(d(resourceId=PHOTOS_CROP_AND_ROTATE_RESET_RESID).wait.exists(timeout=3000))
        self.assertTrue(d(resourceId=PHOTOS_ROTATE_BUTTON_RESID).wait.exists(timeout=3000))
        Photos.accept_changes()
        Photos.save_changes()
        Photos.return_to_main_screen()
        Photos.click_first_photo()
        final_details = Photos.get_media_details()
        try:
            self.assertTrue(int(initial_details.width) > int(final_details.width) and
                            int(initial_details.height) > int(final_details.height))
        finally:
            Photos.return_to_main_screen()
            Photos.long_click_first_photo()
            Photos.move_to_trash()

    def test_search_for_photos(self):
        Photos.click_search_button()
        # categories not present in every version of photos
        # self.assertTrue(d(text=PHOTOS_SEARCH_CATEGORY_PLACES_TXT).wait.exists(timeout=3000))
        # self.assertTrue(d(text=PHOTOS_SEARCH_CATEGORY_TYPES_TXT).wait.exists(timeout=3000))
        d(resourceId=PHOTOS_SEARCH_BOX_RESID).set_text("photo")
        d.press.enter()
        self.assertTrue(d(resourceId=PHOTOS_PHOTO_TILE_RESID).wait.exists(timeout=3000))

    def get_current_photo_center_square_crop(self, square_side_size):
        self.screenshot_utils.take_screenshot()
        image_center = self.screenshot_utils.crop_center_square_pixels(square_side_size)
        return image_center

    def click_blue_wallpaper(self):
        self.screenshot_utils.take_screenshot()
        blue_pixels = self.screenshot_utils.get_all_pixels_of_color(PHOTOS_BLUE_WALLPAPER_COLOR, color_error=10)
        if len(blue_pixels) == 0:
            LOG.info("could not find any blue pixels in screenshot")
            return
        click_pixel = blue_pixels[len(blue_pixels)/2]
        AdbUtils.tap(click_pixel.x, click_pixel.y)
        time.sleep(2)

    def long_click_blue_wallpaper(self):
        self.screenshot_utils.take_screenshot()
        blue_pixels = self.screenshot_utils.get_all_pixels_of_color(PHOTOS_BLUE_WALLPAPER_COLOR, color_error=10)
        if len(blue_pixels) == 0:
            LOG.info("could not find any blue pixels in screenshot")
            return
        click_pixel = blue_pixels[len(blue_pixels)/2]
        AdbUtils.long_tap(click_pixel.x, click_pixel.y)
        time.sleep(2)

    def test_enhance_photo(self):
        Photos.click_first_photo()
        Photos.edit_current_photo()
        self.assertTrue(d(resourceId=PHOTOS_ADJUSTMENT_AUTO_RESID).wait.exists(timeout=3000))
        self.assertTrue(d(resourceId=PHOTOS_ADJUSTMENT_LIGHT_RESID).wait.exists(timeout=3000))
        self.assertTrue(d(resourceId=PHOTOS_ADJUSTMENT_COLOR_RESID).wait.exists(timeout=3000))
        self.assertTrue(d(resourceId=PHOTOS_ADJUSTMENT_POP_RESID).wait.exists(timeout=3000))
        self.assertTrue(d(resourceId=PHOTOS_ADJUSTMENT_VIGNETTE_RESID).wait.exists(timeout=3000))
        initial_image_center = self.get_current_photo_center_square_crop(200)
        d(resourceId=PHOTOS_ADJUSTMENT_AUTO_RESID).click()
        time.sleep(5)  # wait for the adjustments to take effect
        final_image_center = self.get_current_photo_center_square_crop(200)
        self.assertTrue(initial_image_center != final_image_center)
        Photos.cancel_photo_adjustments()
        photo_adjustment_resource_ids = [PHOTOS_ADJUSTMENT_LIGHT_RESID, PHOTOS_ADJUSTMENT_COLOR_RESID,
                                         PHOTOS_ADJUSTMENT_POP_RESID, PHOTOS_ADJUSTMENT_VIGNETTE_RESID]
        for resource_id in photo_adjustment_resource_ids:
            initial_image_center = self.get_current_photo_center_square_crop(300)
            d(resourceId=resource_id).click()
            self.assertTrue(d(resourceId=PHOTOS_ADJUSTMENT_SEEK_BAR_RESID).wait.exists(timeout=3000))
            Photos.select_adjustment_seek_bar_level(percent=90)
            time.sleep(5)  # wait for the adjustments to take effect
            final_image_center = self.get_current_photo_center_square_crop(300)
            self.assertTrue(initial_image_center != final_image_center,
                            "when applying adjustment %s, the result was the same as the initial image" % resource_id)
            Photos.cancel_photo_adjustments()

    def test_use_as_photo(self):
        self.wait_for_media_store_refresh(10)
        # Photos.click_first_photo()  # the first photo should be the blue_wallpaper.jpg file
        self.click_blue_wallpaper()
        Photos.set_current_photo_as_wallpaper()
        d.press.home()
        time.sleep(3)  # wait for the wallpaper to be displayed
        wallpaper_center_crop = self.get_current_photo_center_square_crop(100)
        blue_wallpaper_pixels_found = self.screenshot_utils\
            .get_nr_of_pixels_of_color_in_crop(PHOTOS_BLUE_WALLPAPER_COLOR, wallpaper_center_crop, color_error=10)
        LOG.info("found a nr. of %s blue wallpaper pixels" % str(blue_wallpaper_pixels_found))
        self.assertTrue(blue_wallpaper_pixels_found > 1000, "the blue wallpaper must have been set on the device")
        Settings.sync_app_with_google_account("Contacts", nr_of_clicks=6, wait_for_sync_timeout=5)
        Photos.launch()
        Photos.set_current_photo_as_contact_photo()
        Contacts.launch()
        Contacts.search(CONTACTS_AAA_SEARCH_TXT)
        Contacts.click_first_contact()
        time.sleep(3)  # wait for the first contact to be displayed
        contact_photo_crop = self.get_current_photo_center_square_crop(100)
        blue_contact_photo_pixels_found = self.screenshot_utils\
            .get_nr_of_pixels_of_color_in_crop(PHOTOS_BLUE_WALLPAPER_COLOR, contact_photo_crop, color_error=10)
        LOG.info("found a nr. of %s blue contact photo pixels" % str(blue_contact_photo_pixels_found))
        self.assertTrue(blue_contact_photo_pixels_found > 1000, "the contact photo must have been set on the device")
        # Photos.launch()
        # Photos.return_to_main_screen()
        # Photos.long_click_first_photo()
        # Photos.move_to_trash()

    def test_cancel_changes(self):
        Photos.click_first_photo()
        Photos.edit_current_photo()
        initial_photo = self.get_current_photo_center_square_crop(200)
        self.assertTrue(d(resourceId=PHOTOS_ADJUSTMENT_LIGHT_RESID).wait.exists(timeout=7000))
        d(resourceId=PHOTOS_ADJUSTMENT_LIGHT_RESID).click()
        self.assertTrue(d(resourceId=PHOTOS_ADJUSTMENT_SEEK_BAR_RESID).wait.exists(timeout=3000))
        Photos.select_adjustment_seek_bar_level(percent=90)
        time.sleep(3)  # wait for the adjustments to take effect
        adjusted_photo = self.get_current_photo_center_square_crop(100)
        Photos.cancel_photo_adjustments()
        time.sleep(3)
        canceled_adjustments_photo = self.get_current_photo_center_square_crop(200)
        self.assertTrue(initial_photo != adjusted_photo, "adjustments must appear on photo preview")
        self.assertTrue(adjusted_photo != canceled_adjustments_photo, "the original image must be displayed")

    def test_discard_changes(self):
        Photos.click_first_photo()
        time.sleep(2)
        initial_photo = self.get_current_photo_center_square_crop(200)
        Photos.edit_current_photo()
        self.assertTrue(d(resourceId=PHOTOS_ADJUSTMENT_LIGHT_RESID).wait.exists(timeout=3000))
        d(resourceId=PHOTOS_ADJUSTMENT_LIGHT_RESID).click()
        self.assertTrue(d(resourceId=PHOTOS_ADJUSTMENT_SEEK_BAR_RESID).wait.exists(timeout=3000))
        Photos.select_adjustment_seek_bar_level(percent=90)
        time.sleep(3)  # wait for the adjustments to take effect
        adjusted_photo = self.get_current_photo_center_square_crop(100)
        d.press.back()
        self.assertTrue(d(text=PHOTOS_DISCARD_CHANGES_BUTTON_TXT).wait.exists(timeout=3000))
        time.sleep(2)
        d(text=PHOTOS_DISCARD_CHANGES_BUTTON_TXT).click()
        time.sleep(2)
        discarded_changes_photo = self.get_current_photo_center_square_crop(200)
        self.assertTrue(initial_photo != adjusted_photo, "adjustments must appear on photo preview")
        self.assertTrue(initial_photo == discarded_changes_photo, "the original image must be displayed")

    def test_compare_images(self):
        Photos.click_first_photo()
        Photos.edit_current_photo()
        initial_photo = self.get_current_photo_center_square_crop(200)
        self.assertTrue(d(resourceId=PHOTOS_ADJUSTMENT_LIGHT_RESID).wait.exists(timeout=3000))
        d(resourceId=PHOTOS_ADJUSTMENT_LIGHT_RESID).click()
        self.assertTrue(d(resourceId=PHOTOS_ADJUSTMENT_SEEK_BAR_RESID).wait.exists(timeout=3000))
        Photos.select_adjustment_seek_bar_level(percent=90)
        time.sleep(3)  # wait for the adjustments to take effect
        adjusted_photo = self.get_current_photo_center_square_crop(100)

        def long_click_on_image(image_view, click_time_in_seconds):
            UiAutomatorUtils.long_click(image_view, press_time_in_seconds=click_time_in_seconds)

        self.long_clicked_photo = None

        def get_current_photo_crop(delay_time_in_seconds):
            time.sleep(delay_time_in_seconds)
            self.long_clicked_photo = self.get_current_photo_center_square_crop(100)

        adjusted_image_view = d(resourceId=PHOTOS_PHOTO_PREVIEW_RESID)
        long_click_thread = Thread(target=long_click_on_image, args=(adjusted_image_view, 15,))
        screenshot_thread = Thread(target=get_current_photo_crop, args=(7,))
        long_click_thread.start()
        screenshot_thread.start()
        screenshot_thread.join()
        long_click_thread.join()
        self.assertTrue(initial_photo != adjusted_photo)
        self.assertTrue(adjusted_photo != self.long_clicked_photo,
                        "long clicking on the adjusted image should preview the image without adjustments")

    def test_apply_filter_to_photo(self):
        Photos.click_first_photo()
        first_photo = self.get_current_photo_center_square_crop(200)
        Photos.edit_current_photo()
        initial_photo = self.get_current_photo_center_square_crop(200)
        Photos.select_filter_option()
        Photos.select_filter(PHOTOS_FILTER_OPTIONS_DESC[1])
        time.sleep(3)  # wait for filter to take effect
        filter_photo = self.get_current_photo_center_square_crop(200)
        self.assertTrue(initial_photo != filter_photo)
        for filter_description in PHOTOS_FILTER_OPTIONS_DESC:
            self.assertTrue(Photos.select_filter(filter_description))
        Photos.accept_changes()
        Photos.save_changes()
        Photos.return_to_main_screen()
        Photos.click_first_photo()
        filtered_photo = self.get_current_photo_center_square_crop(200)
        self.assertTrue(first_photo != filtered_photo, "The photo with the selected filter should have been saved")
        Photos.return_to_main_screen()
        Photos.long_click_first_photo()
        Photos.move_to_trash()

    def test_shared_links(self):
        Photos.open_shared_links()
        Photos.delete_all_shared_links()
        Photos.return_to_main_screen()
        Photos.long_click_first_photo()
        Photos.click_share_button()
        UiAutomatorUtils.refresh_uiautomator_view_hierarchy()
        shared_link = False
        for alternative in PHOTOS_SHARE_LINK_ALTERNATIVES_TXT:
            if d(text=alternative).wait.exists(timeout=3000):
                d(text=alternative).click()
                shared_link = True
        self.assertTrue(shared_link, "must share link")
        time.sleep(2)
        d(textContains="Link copied to clipboard.").wait.exists(timeout=20000)
        Photos.return_to_main_screen()
        Photos.open_shared_links()
        self.assertTrue(d(resourceId=PHOTOS_SHARED_LINK_MENU_RESID).wait.exists(timeout=5000))

    def test_view_device_folders(self):
        self.wait_for_media_store_refresh(10)
        Photos.open_device_folders()
        self.assertTrue(d(resourceId=PHOTOS_PHOTO_TILE_RESID).wait.exists(timeout=5000))
        Photos.delete_first_photo_from_device_folders()

    def test_sync_device_photos(self):
        self.wait_for_media_store_refresh(10)
        Photos.open_device_folders()
        if d(descriptionContains=PHOTOS_AUTO_BACKUP_TOGGLE_ON_DESC).wait.exists(timeout=5000):
            d(descriptionContains=PHOTOS_AUTO_BACKUP_TOGGLE_ON_DESC).click()
        time.sleep(10)

    def test_delete_photo_movie(self):
        self.wait_for_media_store_refresh(10)
        UiAutomatorUtils.refresh_uiautomator_view_hierarchy()
        self.assertTrue(d(descriptionContains=PHOTOS_VIDEO_CONTENT_DESC).wait.exists(timeout=5000))
        self.assertTrue(d(descriptionContains=PHOTOS_PHOTO_CONTENT_DESC).wait.exists(timeout=2000))
        # delete_first_photo also deletes video files, so we remove the first 2 files
        self.long_click_blue_wallpaper()
        Photos.move_to_trash()
        Photos.long_click_first_video()
        Photos.move_to_trash()
        Photos.open_trash()
        UiAutomatorUtils.refresh_uiautomator_view_hierarchy()
        self.assertTrue(d(descriptionContains=PHOTOS_VIDEO_CONTENT_DESC).wait.exists(timeout=5000))
        self.assertTrue(d(descriptionContains=PHOTOS_PHOTO_CONTENT_DESC).wait.exists(timeout=2000))
        Photos.open_trash()
        Photos.empty_trash()
        UiAutomatorUtils.refresh_uiautomator_view_hierarchy()
        self.assertFalse(d(resourceId=PHOTOS_PHOTO_TILE_RESID))

    def test_open_in_browser_photolink(self):
        self.wait_for_media_store_refresh(10)
        Photos.open_shared_links()
        Photos.delete_all_shared_links()
        Photos.return_to_main_screen()
        self.long_click_blue_wallpaper()
        Photos.click_share_button()
        UiAutomatorUtils.refresh_uiautomator_view_hierarchy()
        shared_link = False
        for alternative in PHOTOS_SHARE_LINK_ALTERNATIVES_TXT:
            if d(text=alternative).wait.exists(timeout=3000):
                d(text=alternative).click()
                shared_link = True
        self.assertTrue(shared_link, "must share link")
        time.sleep(5)
        d(textContains="Link copied to clipboard.").wait.exists(timeout=20000)
        shared_link = ClipboardManager.get_clipboard_text()
        print "shared_link: " + shared_link
        Chrome.launch()
        Chrome.go_to_url(shared_link, timeout=20)
        self.screenshot_utils.take_screenshot()
        blue_pixels = self.screenshot_utils.get_all_pixels_of_color(PHOTOS_BLUE_WALLPAPER_COLOR, color_error=10)
        LOG.info("found %s blue pixels in screenshot" % str(len(blue_pixels)))
        self.assertTrue(len(blue_pixels) > 100, "there must be a preview of the bluewallpaper in the chrome page")


if __name__ == "__main__":
    test_result = SingleMethodRunner.run_single_test(PhotosTests, "test_discard_changes")
    print test_result.wasSuccessful()
