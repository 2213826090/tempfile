from PyUiApi.app_utils.contacts_utils import *
from string import Template

open_img_cmd = '''am start -t image/* -d file:///$FILE_PATH$'''
refresh_media_store_cmd = Template('''am broadcast -a android.intent.action.MEDIA_MOUNTED -d file:///$media_dir''')

DIMENSION_KEY = "Dimensions"
DATE_KEY = "Date taken"
FILTER_SCROLL_TIMES = 3


class PhotosL(object):
    @staticmethod
    def launch():
        UiAutomatorUtils.launch_app_from_apps_menu(PHOTOS_SHORTCUT_NAME)
        for i in range(20):
            time.sleep(1)
            if d(text=PHOTOS_ALL_TAB_TXT).wait.exists(timeout=1000):
                return
            if d(text="No thanks").wait.exists(timeout=1000):
                d(text="No thanks").click()
                return
            if d(text="Later").wait.exists(timeout=1000):
                d(text="Later").click()
                return

    @staticmethod
    def refresh_media_store(media_dir=None):
        if media_dir is None:
            media_dir = "sdcard"
        AdbUtils.run_adb_cmd(refresh_media_store_cmd.substitute(media_dir=media_dir))

    @staticmethod
    def click_all_tab():
        if d(text=PHOTOS_ALL_TAB_TXT).wait.exists(timeout=3000):
            d(text=PHOTOS_ALL_TAB_TXT).click()
        time.sleep(1)

    @staticmethod
    def click_highlights_tab():
        if d(text=PHOTOS_HIGHLIGHTS_TAB_TXT).wait.exists(timeout=3000):
            d(text=PHOTOS_HIGHLIGHTS_TAB_TXT).click()
        time.sleep(1)

    @staticmethod
    def sign_in():
        PhotosL.click_highlights_tab()
        if d(text=PHOTOS_SIGN_IN_BUTTON_TXT).wait.exists(timeout=3000):
            d(text=PHOTOS_SIGN_IN_BUTTON_TXT).click()
        time.sleep(1)
        PhotosL.click_all_tab()
        if d(text=PHOTOS_BACKUP_TITLE_TXT).wait.exists(timeout=10000) and d(
                text=PHOTOS_NO_THANKS_BUTTON_TXT).wait.exists(timeout=3000):
            d(text=PHOTOS_NO_THANKS_BUTTON_TXT).click()
        if d(text=PHOTOS_DRIVE_PHOTOS_OPTION_TXT).wait.exists(timeout=10000) and d(
                text=PHOTOS_NO_THANKS_BUTTON_TXT).wait.exists(timeout=3000):
            d(text=PHOTOS_NO_THANKS_BUTTON_TXT).click()
        if d(text=PHOTOS_MORE_PHOTOS_OPTION_TXT).wait.exists(timeout=10000) and d(
                text=PHOTOS_NO_THANKS_BUTTON_TXT).wait.exists(timeout=3000):
            d(text=PHOTOS_NO_THANKS_BUTTON_TXT).click()
        time.sleep(5)

    @staticmethod
    def click_first_photo():
        photos = []
        if d(description=PHOTOS_PHOTO_DESC).wait.exists(timeout=3000):
            photos = d(description=PHOTOS_PHOTO_DESC)
        else:
            tile = d(resourceId=PHOTOS_TILES_LIST_RESID)
            photos = tile.child(className=ANDROID_VIEW_CLASSNAME)
        if len(photos) > 0:
            photos[0].click()

    @staticmethod
    def long_click_first_photo():
        photos = []
        if d(description=PHOTOS_PHOTO_DESC).wait.exists(timeout=3000):
            photos = d(description=PHOTOS_PHOTO_DESC)
        else:
            tile = d(resourceId=PHOTOS_TILES_LIST_RESID)
            photos = tile.child(className=ANDROID_VIEW_CLASSNAME)
        if len(photos) > 0:
            photos[0].long_click()

    @staticmethod
    def long_click_first_video():
        UiAutomatorUtils.refresh_uiautomator_view_hierarchy()
        if d(descriptionContains=PHOTOS_VIDEO_DESC).wait.exists(timeout=3000):
            UiAutomatorUtils.long_click(d(descriptionContains=PHOTOS_VIDEO_DESC), 2)
        time.sleep(2)

    @staticmethod
    def copy_photo_to_new_album(album_name):
        PhotosL.long_click_first_photo()
        if d(resourceId=PHOTOS_COPY_TO_ALBUM_BUTTON_RESID).wait.exists(timeout=3000):
            d(resourceId=PHOTOS_COPY_TO_ALBUM_BUTTON_RESID).click()
        if d(text=PHOTOS_NEW_ALBUM_BUTTON_TXT).wait.exists(timeout=5000):
            d(text=PHOTOS_NEW_ALBUM_BUTTON_TXT).click()
        if d(text=PHOTOS_ALBUM_NAME_FIELD_TXT).wait.exists(timeout=5000):
            d(text=PHOTOS_ALBUM_NAME_FIELD_TXT).clear_text()
            d(text=PHOTOS_ALBUM_NAME_FIELD_TXT).set_text(album_name)
            d(text=POPUP_OK).click()
        d(text=PHOTOS_ALL_TAB_TXT).wait.exists(timeout=5000)

    @staticmethod
    def copy_photo_to_existing_album(album_name):
        PhotosL.long_click_first_photo()
        if d(resourceId=PHOTOS_COPY_TO_ALBUM_BUTTON_RESID).wait.exists(timeout=3000):
            d(resourceId=PHOTOS_COPY_TO_ALBUM_BUTTON_RESID).click()
        if d(text=album_name).wait.exists(timeout=5000):
            d(text=album_name).click()
        if d(text=PHOTOS_COPY_CONFIRM_BUTTON_TXT).wait.exists(timeout=5000):
            d(text=PHOTOS_COPY_CONFIRM_BUTTON_TXT).click()
        d(text=PHOTOS_ALL_TAB_TXT).wait.exists(timeout=5000)

    @staticmethod
    def get_photos():
        photos = None
        if d(description=PHOTOS_PHOTO_DESC).wait.exists(timeout=3000):
            photos = d(description=PHOTOS_PHOTO_DESC)
        return photos

    @staticmethod
    def edit_current_photo():
        if d(resourceId=PHOTOS_PHOTO_EDIT_OPTION_RESID).wait.exists(timeout=3000):
            d(resourceId=PHOTOS_PHOTO_EDIT_OPTION_RESID).click()
        time.sleep(1)

    @staticmethod
    def delete_current_photo():
        if d(resourceId=PHOTOS_DELETE_RESID).wait.exists(timeout=3000):
            d(resourceId=PHOTOS_DELETE_RESID).click()
        time.sleep(1)

    @staticmethod
    def delete_first_photo_from_device_folders():
        PhotosM.long_click_first_photo()
        if d(resourceId=PHOTOS_PHOTO_MORE_OPTIONS_RESID).wait.exists(timeout=3000):
            d(resourceId=PHOTOS_PHOTO_MORE_OPTIONS_RESID).click()
        time.sleep(1)
        if d(text=PHOTOS_DELETE_DEVICE_COPY_TXT).wait.exists(timeout=3000):
            d(text=PHOTOS_DELETE_DEVICE_COPY_TXT).click()
        time.sleep(1)
        if d(text=PHOTOS_DELETE_DEVICE_COPY_CONFIRM_TXT).wait.exists(timeout=3000):
            d(text=PHOTOS_DELETE_DEVICE_COPY_CONFIRM_TXT).click()
        time.sleep(1)

    @staticmethod
    def restore_selected_photo():
        if d(resourceId=PHOTOS_RESTORE_FROM_TRASH_BUTTON_RESID).wait.exists(timeout=3000):
            d(resourceId=PHOTOS_RESTORE_FROM_TRASH_BUTTON_RESID).click()
        time.sleep(1)

    @staticmethod
    def select_crop_option():
        if d(description=PHOTOS_CROP_DESC).wait.exists(timeout=3000):
            d(description=PHOTOS_CROP_DESC).click()
        time.sleep(1)

    @staticmethod
    def select_free_crop():
        if d(description=PHOTOS_CROP_FREE_TXT).wait.exists(timeout=3000):
            d(description=PHOTOS_CROP_FREE_TXT).click()
        elif d(text=PHOTOS_CROP_FREE_TXT).wait.exists(timeout=3000):
            d(text=PHOTOS_CROP_FREE_TXT).click()
        time.sleep(1)

    @staticmethod
    def select_square_crop():
        if d(description=PHOTOS_CROP_SQUARE_TXT).wait.exists(timeout=3000):
            d(description=PHOTOS_CROP_SQUARE_TXT).click()
        elif d(text=PHOTOS_CROP_SQUARE_TXT).wait.exists(timeout=3000):
            d(text=PHOTOS_CROP_SQUARE_TXT).click()
        time.sleep(1)

    @staticmethod
    def select_done_edit():
        if d(text=PHOTOS_DONE_BUTTON_TXT).wait.exists(timeout=3000):
            d(text=PHOTOS_DONE_BUTTON_TXT).click()
        time.sleep(1)

    @staticmethod
    def apply_changes():
        if d(resourceId=PHOTOS_APPLY_BUTTON_RESID).wait.exists(timeout=3000):
            d(resourceId=PHOTOS_APPLY_BUTTON_RESID).click()
        time.sleep(1)

    @staticmethod
    def cancel_changes():
        if d(resourceId=PHOTOS_CANCEL_BUTTON_RESID).wait.exists(timeout=3000):
            d(resourceId=PHOTOS_CANCEL_BUTTON_RESID).click()
        time.sleep(1)

    @staticmethod
    def return_to_main_screen():
        while not d(text=PHOTOS_ALL_TAB_TXT).wait.exists(timeout=3000):
            d.press.back()
        time.sleep(2)

    @staticmethod
    def return_to_selected_photo():
        while not d(resourceId=PHOTOS_EDIT_RESID).wait.exists(timeout=2000):
            d.press.back()
        time.sleep(2)

    @staticmethod
    def open_menu():
        if d(description=PHOTOS_OPTIONS_DESC).wait.exists(timeout=3000):
            d(description=PHOTOS_OPTIONS_DESC).click()
        time.sleep(1)

    @staticmethod
    def open_navigation_drawer():
        if d(description=PHOTOS_NAVIGATION_DRAWER_DESC).wait.exists(timeout=3000):
            d(description=PHOTOS_NAVIGATION_DRAWER_DESC).click()
        time.sleep(1)

    @staticmethod
    def open_trash():
        if d(text=PHOTOS_TRASH_BUTTON_TXT).wait.exists(timeout=3000):
            d(text=PHOTOS_TRASH_BUTTON_TXT).click()
        time.sleep(1)

    @staticmethod
    def open_photos():
        if d(text=PHOTOS_PHOTOS_BUTTON_TXT).wait.exists(timeout=3000):
            d(text=PHOTOS_PHOTOS_BUTTON_TXT).click()
        time.sleep(1)

    @staticmethod
    def open_albums():
        if d(text=PHOTOS_ALBUMS_BUTTON_TXT).wait.exists(timeout=3000):
            d(text=PHOTOS_ALBUMS_BUTTON_TXT).click()
        time.sleep(1)

    @staticmethod
    def open_media_details():
        PhotosL.open_menu()
        if d(text="Details").wait.exists(timeout=3000):
            d(text="Details").click()
        time.sleep(1)

    @staticmethod
    def select_filter(filter_description):
        if d(description=filter_description).wait.exists(timeout=3000):
            d(description=filter_description).click()
        elif d(resourceId=PHOTOS_FILTER_LIST_RESID).wait.exists(timeout=3000):
            for i in range(FILTER_SCROLL_TIMES):
                d(resourceId=PHOTOS_FILTER_LIST_RESID).scroll.horiz.forward()
                if d(description=filter_description).wait.exists(timeout=3000):
                    d(description=filter_description).click()
                    break
        time.sleep(2)

    @staticmethod
    def select_looks_filter_instance(instance):
        if d(resourceId=PHOTOS_PRESET_LIST_RESID).child(className=ANDROID_VIEW_CLASSNAME, index=instance).wait.exists(
                timeout=3000):
            d(resourceId=PHOTOS_PRESET_LIST_RESID).child(className=ANDROID_VIEW_CLASSNAME, index=instance).click()
        elif d(resourceId=PHOTOS_PRESET_LIST_RESID).wait.exists(timeout=3000):
            for i in range(FILTER_SCROLL_TIMES):
                d(resourceId=PHOTOS_PRESET_LIST_RESID).scroll.horiz.forward()
                if d(resourceId=PHOTOS_PRESET_LIST_RESID).child(className=ANDROID_VIEW_CLASSNAME,
                                                                index=instance).wait.exists(timeout=3000):
                    d(resourceId=PHOTOS_PRESET_LIST_RESID).child(className=ANDROID_VIEW_CLASSNAME,
                                                                 index=instance).click()
                    break
        time.sleep(2)

    @staticmethod
    def select_frame_instance(instance):
        if d(className=ANDROID_WIDGET_HORIZONTAL_SCROLL_VIEW).child(text=instance).wait.exists(timeout=3000):
            d(className=ANDROID_WIDGET_HORIZONTAL_SCROLL_VIEW).child(text=instance).click()
        elif d(className=ANDROID_WIDGET_HORIZONTAL_SCROLL_VIEW).wait.exists(timeout=3000):
            for i in range(FILTER_SCROLL_TIMES):
                d(className=ANDROID_WIDGET_HORIZONTAL_SCROLL_VIEW).scroll.horiz.forward()
                if d(className=ANDROID_WIDGET_HORIZONTAL_SCROLL_VIEW).child(text=instance).wait.exists(timeout=3000):
                    d(className=ANDROID_WIDGET_HORIZONTAL_SCROLL_VIEW).child(text=instance).click()
                    break
        time.sleep(2)

    @staticmethod
    def set_auto_filter_off():
        if d(resourceId=PHOTOS_AUTO_FILTER_OFF_BUTTON_RESID).wait.exists(timeout=3000):
            d(resourceId=PHOTOS_AUTO_FILTER_OFF_BUTTON_RESID).click()
        time.sleep(1)

    @staticmethod
    def set_auto_filter_normal():
        if d(resourceId=PHOTOS_AUTO_FILTER_NORMAL_BUTTON_RESID).wait.exists(timeout=3000):
            d(resourceId=PHOTOS_AUTO_FILTER_NORMAL_BUTTON_RESID).click()
        time.sleep(1)

    @staticmethod
    def set_auto_filter_high():
        if d(resourceId=PHOTOS_AUTO_FILTER_HIGH_BUTTON_RESID).wait.exists(timeout=3000):
            d(resourceId=PHOTOS_AUTO_FILTER_HIGH_BUTTON_RESID).click()
        time.sleep(1)

    @staticmethod
    def get_media_details():
        PhotosL.open_media_details()
        detail_labels = []
        detail_values = []
        details = {}
        if d(resourceId=PHOTOS_DETAIL_LABEL_RESID).wait.exists(timeout=3000):
            detail_labels = d(resourceId=PHOTOS_DETAIL_LABEL_RESID)
            detail_values = d(resourceId=PHOTOS_DETAIL_VALUE_RESID)
        for i in range(len(detail_labels)):
            label = detail_labels[i].info["text"]
            value = detail_values[i].info["text"]
            details[str(label)] = str(value)
        return details

    @staticmethod
    def get_size_from_details(details):
        size_str = details[DIMENSION_KEY]
        sizes = size_str.split('x')
        sizes = [int(x.strip()) for x in sizes]
        return tuple(sizes)

    @staticmethod
    def get_date_from_details(details):
        date_str = details[DATE_KEY]  # example "Mar 12,2015 12:21:58 PM"
        date = time.strptime(date_str, "%b %d, %Y %H:%M:%S %p")
        return date


class PhotosM(PhotosL):
    photos_launcher_string = "com.google.android.apps.photos/.home.HomeActivity"
    started_once_previously = False

    @staticmethod
    def launch():
        AdbUtils.start_activity_from_shell(PhotosM.photos_launcher_string)
        PhotosM.wait_for_app_launch()

    @staticmethod
    def launch_without_account():
        AdbUtils.start_activity_from_shell(PhotosM.photos_launcher_string)
        if d(descriptionContains=PHOTOS_SHOW_NAV_DRAWER_DESC).wait.exists(timeout=3000):
            pass
        else:
            vn = ViewNavigator()
            vn.navigate_views(["Get started", 1, "No thanks"])

    @staticmethod
    def wait_for_app_launch():
        for i in range(10):
            time.sleep(1)
            UiAutomatorUtils.refresh_uiautomator_view_hierarchy()
            if d(resourceId=PHOTOS_SEARCH_BUTTON_RESID).wait.exists(timeout=1000):
                LOG.info("Photos app started")
                PhotosM.started_once_previously = True
                return True
            if PhotosM.started_once_previously:
                if d(resourceId=PHOTOS_PHOTO_VIEW_RESID).wait.exists(timeout=1000):
                    return True
            # next button appears 5 times
            if d(resourceId=PHOTOS_NEXT_BUTTON_RESID).wait.exists(timeout=1000):
                d(resourceId=PHOTOS_NEXT_BUTTON_RESID).click()
                continue
            # continue button appears 2 times
            if d(resourceId=PHOTOS_CONTINUE_BUTTON_RESID).wait.exists(timeout=1000):
                d(resourceId=PHOTOS_CONTINUE_BUTTON_RESID).click()
                continue
            # get started button appears 1 time
            if d(resourceId=PHOTOS_GET_STARTED_BUTTON_RESID).wait.exists(timeout=1000):
                d(resourceId=PHOTOS_GET_STARTED_BUTTON_RESID).click()
                continue
            # sign in button to sync photos content with the cloud
            for alternative in PHOTOS_SIGN_IN_ALTERNATIVES_TXT:
                if d(textContains=alternative).wait.exists(timeout=1000):
                    d(textContains=alternative).click()

    @staticmethod
    def return_to_main_screen():
        wait_steps = 10
        while not d(resourceId=PHOTOS_SEARCH_BUTTON_RESID).wait.exists(timeout=3000):
            d.press.back()
            wait_steps -= 1
            if wait_steps < 0:
                return
        time.sleep(2)

    @staticmethod
    def click_first_photo():
        if d(resourceId=PHOTOS_PHOTO_TILE_RESID).wait.exists(timeout=5000):
            d(resourceId=PHOTOS_PHOTO_TILE_RESID)[0].click()
        time.sleep(1)

    @staticmethod
    def click_nth_photo(n):
        if d(resourceId=PHOTOS_PHOTO_TILE_RESID).wait.exists(timeout=5000) and \
                        n < d(resourceId=PHOTOS_PHOTO_TILE_RESID).count:
                d(resourceId=PHOTOS_PHOTO_TILE_RESID)[n].click()
        time.sleep(1)

    @staticmethod
    def long_click_first_photo():
        if d(resourceId=PHOTOS_PHOTO_TILE_RESID).wait.exists(timeout=5000):
            # normal long click does not always work
            # d(resourceId=PHOTOS_PHOTO_TILE_RESID)[0].long_click()
            UiAutomatorUtils.long_click(d(resourceId=PHOTOS_PHOTO_TILE_RESID)[0], 2)
        time.sleep(1)

    @staticmethod
    def move_to_trash():
        if d(resourceId=PHOTOS_MOVE_TO_TRASH_RESID).wait.exists(timeout=5000):
            d(resourceId=PHOTOS_MOVE_TO_TRASH_RESID).click()
        if d(textContains=PHOTOS_DELETE_SHARED_ITEMS_TXT).wait.exists(timeout=2000):
            d(textContains=PHOTOS_DELETE_SHARED_ITEMS_TXT).click()
        if d(textContains=PHOTOS_MOVE_TO_TRASH_TXT).wait.exists(timeout=1000):
            d(textContains=PHOTOS_MOVE_TO_TRASH_TXT).click()
        time.sleep(1)

    @staticmethod
    def open_media_details():
        if d(resourceId=PHOTOS_PHOTO_DETAILS_OPTION_RESID).wait.exists(timeout=3000):
            d(resourceId=PHOTOS_PHOTO_DETAILS_OPTION_RESID).click()
        time.sleep(1)

    @staticmethod
    def get_media_details():
        PhotosM.open_media_details()
        detail_labels = []
        detail_values = []
        if d(resourceId=PHOTOS_DETAIL_LABEL_RESID).wait.exists(timeout=3000):
            detail_labels = d(resourceId=PHOTOS_DETAIL_LABEL_RESID)
            detail_values = d(resourceId=PHOTOS_DETAIL_VALUE_RESID)
        labels = []
        values = []
        for i in range(len(detail_labels)):
            labels.append(detail_labels[i].info["text"])
        for i in range(len(detail_values)):
            values.append(detail_values[i].info["text"])
        info_labels = labels[len(labels) - len(values):]
        photo_details = PhotoDetails(info_labels, values)
        LOG.info("found following photo details: " + photo_details.get_details_as_string())
        return photo_details

    @staticmethod
    def select_crop_option():
        if d(resourceId=PHOTOS_CROP_BUTTON_RESID).wait.exists(timeout=3000):
            d(resourceId=PHOTOS_CROP_BUTTON_RESID).click()
        time.sleep(1)

    @staticmethod
    def crop_current_photo(screenshooter, crop_length_x, crop_length_y):
        """
        @type screenshooter: ScreenshotUtils
        """
        screenshooter.take_screenshot()
        d(resourceId=PHOTOS_CROP_OVERLAY_RESID).wait.exists(timeout=5000)
        bounds = Bounds(d(resourceId=PHOTOS_CROP_OVERLAY_RESID).info)
        crop_pixels = screenshooter.get_all_pixels_of_color(COLOR_WHITE, color_error=10, min_x=bounds.left,
                                                            min_y=bounds.top, max_x=bounds.right, max_y=bounds.bottom)
        crop_swipe_point = crop_pixels[0]
        LOG.info("found swipe point: %s %s" % (crop_swipe_point.x, crop_swipe_point.y))
        d.swipe(crop_swipe_point.x, crop_swipe_point.y, crop_swipe_point.x + crop_length_x,
                crop_swipe_point.y + crop_length_y)
        time.sleep(4)

    @staticmethod
    def accept_changes():
        if d(resourceId=PHOTOS_ACCEPT_CHANGES_RESID).wait.exists(timeout=3000):
            d(resourceId=PHOTOS_ACCEPT_CHANGES_RESID).click()
        time.sleep(1)

    @staticmethod
    def cancel_changes():
        if d(resourceId=PHOTOS_CANCEL_CHANGES_RESID).wait.exists(timeout=3000):
            d(resourceId=PHOTOS_CANCEL_CHANGES_RESID).click()
        time.sleep(1)

    @staticmethod
    def save_changes():
        if d(resourceId=PHOTOS_SAVE_CHANGES_RESID).wait.exists(timeout=3000):
            d(resourceId=PHOTOS_SAVE_CHANGES_RESID).click()
        time.sleep(1)

    @staticmethod
    def click_search_button():
        if d(resourceId=PHOTOS_SEARCH_BUTTON_RESID).wait.exists(timeout=3000):
            d(resourceId=PHOTOS_SEARCH_BUTTON_RESID).click()
        time.sleep(1)

    @staticmethod
    def cancel_photo_adjustments():
        if d(resourceId=PHOTOS_CANCEL_CHANGES_RESID).wait.exists(timeout=3000):
            d(resourceId=PHOTOS_CANCEL_CHANGES_RESID).click()
        time.sleep(1)

    @staticmethod
    def select_adjustment_seek_bar_level(percent=50):
        if d(resourceId=PHOTOS_ADJUSTMENT_SEEK_BAR_RESID).wait.exists(timeout=3000):
            adjustment_bar_bounds = Bounds(d(resourceId=PHOTOS_ADJUSTMENT_SEEK_BAR_RESID).info)
            x = adjustment_bar_bounds.top + (adjustment_bar_bounds.bottom - adjustment_bar_bounds.top) / 2
            y = adjustment_bar_bounds.left + (adjustment_bar_bounds.right - adjustment_bar_bounds.left) * percent / 100.0
            LOG.info("clicking adjustment bar at %s %s" % (str(y), str(x)))
            d.click(y, x)

    @staticmethod
    def open_more_options_for_current_photo():
        if d(resourceId=PHOTOS_PHOTO_MORE_OPTIONS_RESID).wait.exists(timeout=3000):
            d(resourceId=PHOTOS_PHOTO_MORE_OPTIONS_RESID).click()
        time.sleep(1)

    @staticmethod
    def use_current_photo_as(use_as_option=None):
        PhotosM.open_more_options_for_current_photo()
        if d(text=PHOTOS_PHOTO_USE_AS_OPTION_TXT).wait.exists(timeout=3000):
            d(text=PHOTOS_PHOTO_USE_AS_OPTION_TXT).click()
        if use_as_option is not None and d(text=use_as_option).wait.exists(timeout=3000):
            d(text=use_as_option).click()
        time.sleep(1)

    @staticmethod
    def get_nr_of_photos():
        d(descriptionContains=PHOTOS_PHOTO_DESC).wait.exists(timeout=3000)
        return d(descriptionContains=PHOTOS_PHOTO_DESC).count

    @staticmethod
    def empty_trash():
        if d(resourceId=PHOTOS_PHOTO_MORE_OPTIONS_RESID).wait.exists(timeout=3000):
            d(resourceId=PHOTOS_PHOTO_MORE_OPTIONS_RESID).click()
        if d(text=PHOTOS_EMPTY_TRASH_OPTION_TXT).wait.exists(timeout=4000):
            d(text=PHOTOS_EMPTY_TRASH_OPTION_TXT).click()
        if d(text=PHOTOS_EMPTY_TRASH_CONFIRMATION_TXT).wait.exists(timeout=4000):
            d(text=PHOTOS_EMPTY_TRASH_CONFIRMATION_TXT).click()
        time.sleep(1)

    @staticmethod
    def set_current_photo_as_wallpaper():
        LOG.info("trying to click set wallpaper option")
        PhotosM.use_current_photo_as(PHOTOS_PHOTO_USE_AS_WALLPAPER_OPTION_TXT)
        if d(resourceId=PHOTOS_SET_WALLPAPER_BUTTON_RESID).wait.exists(timeout=3000):
            d(resourceId=PHOTOS_SET_WALLPAPER_BUTTON_RESID).click()
        time.sleep(1)
        # wait for the wallpaper to be set and screen to go back to image options view
        d(resourceId=PHOTOS_PHOTO_MORE_OPTIONS_RESID).wait.exists(timeout=5000)

    @staticmethod
    def set_current_photo_as_contact_photo():
        PhotosM.use_current_photo_as(PHOTOS_PHOTO_USE_AS_CONTACT_PHOTO_OPTION_TXT)
        Contacts.search(CONTACTS_AAA_SEARCH_TXT)
        Contacts.click_first_contact()
        PhotosM.save_changes()

    @staticmethod
    def select_filter_option():
        if d(resourceId=PHOTOS_FILTER_BUTTON_RESID).wait.exists(timeout=3000):
            d(resourceId=PHOTOS_FILTER_BUTTON_RESID).click()
        time.sleep(1)

    @staticmethod
    def scroll_filters_to_beginning():
        if d(resourceId=PHOTOS_FILTER_SCROLLER_RESID).wait.exists(timeout=3000):
            d(resourceId=PHOTOS_FILTER_SCROLLER_RESID).scroll.horiz.toBeginning()
        time.sleep(1)

    @staticmethod
    def click_share_button():
        if d(resourceId=PHOTOS_PHOTO_SHARE_BUTTON_RESID).wait.exists(timeout=3000):
            d(resourceId=PHOTOS_PHOTO_SHARE_BUTTON_RESID).click()
        time.sleep(1)

    @staticmethod
    def show_navigation_drawer():
        if d(description=PHOTOS_SHOW_NAV_DRAWER_DESC).wait.exists(timeout=3000):
            d(description=PHOTOS_SHOW_NAV_DRAWER_DESC).click()
        time.sleep(1)

    @staticmethod
    def open_shared_links():
        PhotosM.show_navigation_drawer()
        if d(text=PHOTOS_SHARED_LINKS_TXT).wait.exists(timeout=3000):
            d(text=PHOTOS_SHARED_LINKS_TXT).click()
        if d(text=PHOTOS_SHARED_ALBUMS_TXT).wait.exists(timeout=1000):
            d(text=PHOTOS_SHARED_ALBUMS_TXT).click()
        time.sleep(1)

    @staticmethod
    def open_device_folders():
        PhotosM.show_navigation_drawer()
        if d(text=PHOTOS_DEVICE_FOLDERS_TXT).wait.exists(timeout=3000):
            d(text=PHOTOS_DEVICE_FOLDERS_TXT).click()
        time.sleep(1)

    @staticmethod
    def open_trash():
        PhotosM.show_navigation_drawer()
        if d(text=PHOTOS_TRASH_OPTION_TXT).wait.exists(timeout=3000):
            d(text=PHOTOS_TRASH_OPTION_TXT).click()
        time.sleep(1)

    @staticmethod
    def delete_all_shared_links():
        while d(resourceId=PHOTOS_SHARED_LINK_MENU_RESID).wait.exists(timeout=10000):
            d(resourceId=PHOTOS_SHARED_LINK_MENU_RESID).click()
            if d(text=PHOTOS_DELETE_SHARED_LINK_OPTION_TXT).wait.exists(timeout=5000):
                d(text=PHOTOS_DELETE_SHARED_LINK_OPTION_TXT).click()
            UiAutomatorUtils.refresh_uiautomator_view_hierarchy()
            if d(text=PHOTOS_DELETE_SHARED_LINK_CONFIRMATION_TXT).wait.exists(timeout=5000):
                d(text=PHOTOS_DELETE_SHARED_LINK_CONFIRMATION_TXT).click()

    @staticmethod
    def select_filter(filter_name):
        filter_selected = False
        scroll_tries = 5
        while scroll_tries > 0:
            if d(description=filter_name).wait.exists(timeout=3000):
                d(description=filter_name).click()
                filter_selected = True
                break
            else:
                d(resourceId=PHOTOS_FILTER_SCROLLER_RESID).scroll.horiz.forward()
            scroll_tries -= 1
        time.sleep(2)
        return filter_selected


class PhotoDetails(object):
    megapix_regex = "([\.\d]+)MP"
    width_regex = "\d+\s*x\s*(\d+)"
    height_regex = "(\d+)\s*x\s*\d+"
    size_regex = "([\d\.]+)\s*.B"
    latitude_regex = "([-\d\.]+)\s*\,"
    longitude_regex = "\,\s*([-\d.]+)"
    fstop_regex = "f/([\d\.]+)"
    focal_length_regex = "([\d\.]+)\s*mm"
    iso_regex = "ISO\s*([\d]+)"

    def __init__(self, labels, values):
        self.date = None
        self.time = None
        self.name = None
        self.size = None
        self.height = None
        self.width = None
        self.megapixels_value = None
        self.iso = None
        self.fstop = None
        self.focal_length = None
        self.latitude = None
        self.longitude = None
        self.extract_info_from_labels(labels)
        self.extract_info_from_values(values)

    def extract_info_from_labels(self, labels):
        self.date = labels[0]
        self.name = labels[1]

    def extract_info_from_values(self, values):
        self.time = values[0]
        self.megapixels_value = re.findall(PhotoDetails.megapix_regex, values[1])[0]
        self.width = re.findall(PhotoDetails.width_regex, values[1])[0]
        self.height = re.findall(PhotoDetails.height_regex, values[1])[0]
        self.size = re.findall(PhotoDetails.size_regex, values[1])[0]
        if len(values) > 2:
            self.fstop = re.findall(PhotoDetails.fstop_regex, values[2])[0]
            self.focal_length = re.findall(PhotoDetails.focal_length_regex, values[2])[0]
            self.iso = re.findall(PhotoDetails.iso_regex, values[2])[0]
        if len(values) > 3:
            self.longitude = re.findall(PhotoDetails.longitude_regex, values[3])[0]
            self.latitude = re.findall(PhotoDetails.latitude_regex, values[3])[0]

    def get_details_as_string(self):
        return "\ndate: " + str(self.date) + " " + "\ntime: " + str(self.time) + \
               "\nname: " + str(self.name) + " " + "\nsize: " + str(self.size) + \
               "\nheight: " + str(self.height) + " " + "\nwidth: " + str(self.width) + \
               "\nmegapixels_value: " + str(self.megapixels_value) + " " + "\niso: " + str(self.iso) + \
               "\nfstop: " + str(self.fstop) + " " + "\nfocal_length: " + str(self.focal_length) + \
               "\nlatitude: " + str(self.latitude) + " " + "\nlongitude: " + str(self.longitude)


# provide support for old photos app
Photos = None
if ANDROID_VERSION is "M":
    Photos = PhotosM
elif ANDROID_VERSION is "L":
    Photos = PhotosL
else:
    Photos = PhotosM


if __name__ == "__main__":
    print Photos.select_filter("FRAMES")