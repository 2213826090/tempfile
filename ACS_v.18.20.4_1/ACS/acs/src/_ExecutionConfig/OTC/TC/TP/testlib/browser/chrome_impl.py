#coding=utf8=
from testlib.util.common import g_common_obj
import time


class ChromeImpl:
    """
        @summary: class for Chrome application Home UI
    """

    #--------- begin locator -------------
    class Locator(object):
        """
            Helper for locator UI Object
        """

        def __init__(self, device):
            self.d = device

        @property
        def btn_next(self):
            """ UI button next """
            return self.d(text="NEXT")

    def __init__(self, cfg):
        self.d = g_common_obj.get_device()
        self._locator = ChromeImpl.Locator(self.d)
        self.cfg = cfg
        self.dut = self.d

    def set_orientation_n(self):
        """
        @summary: set orientation as n
        """
        self.d.orientation = "n"

    chrome_pkg_name = "com.android.chrome"
    chrome_activity_name = ".Main"
    downloads_pkg_name = "com.android.providers.downloads.ui"
    downloads_activity_name = "com.android.providers.downloads.ui.DownloadList"

    def launch_by_am(self):
        g_common_obj.launch_app_am(ChromeImpl.chrome_pkg_name, \
            ChromeImpl.chrome_activity_name)

    def stop_by_am(self):
        g_common_obj.stop_app_am(ChromeImpl.chrome_pkg_name)

    def stop_loading(self):
        if (self.d(description="Stop page loading").exists):
            self.d(description="Stop page loading").click()

    def open_new_tab(self):
        time.sleep(1)
        while self.d(resourceId="com.android.chrome:id/tab_close_btn").exists:
            self.d(resourceId="com.android.chrome:id/tab_close_btn").click.wait(timeout=3000)
            time.sleep(1)
        self.d(description="New tab").click.wait(timeout=3000)

    def open_website(self, url):
        self.d(resourceId="com.android.chrome:id/url_bar").set_text(url)
        self.d.press("enter")

    def close_tab(self, tab_text):
        if self.d(text=tab_text, className="android.widget.TextView").exists:
            self.d(text=tab_text, className="android.widget.TextView").sibling(description="Close tab").click.wait(timeout=3000)
        else:
            self.d(description="Close tab").click.wait(timeout=3000)

    def open_compose_window(self):
        self.d(resourceId="com.google.android.gm:id/compose").\
            click.wait(timeout=3000)
        print "Open email compose window."

    def open_more_options_window(self):
        self.d(description="More options").click.wait(timeout=3000)

    def open_history_window(self):
        self.d(text="History", \
            className='android.widget.TextView').\
            click.wait(timeout=3000)
        #time.sleep 5 seconds to let browser retrieve all history
        time.sleep(5)

    def open_clear_browser_data_window(self):
        self.d(description="CLEAR BROWSING DATA...").\
            click.wait(timeout=3000)

    def clear_cache(self):
        if(not self.d(text="Clear the cache").checked):
            self.d(text="Clear the cache").click()

        self.d(text="Clear").click.wait(timeout=3000)

    def clear_browser_cache(self):
        self.open_more_options_window()
        self.open_history_window()
        self.open_clear_browser_data_window()
        self.clear_cache()
        self.d(description="Settings, Navigate up").click.wait(timeout=3000)
        self.close_tab("History")

    def launch_item_setting_from_app_gallery(self, itemtext):
        g_common_obj.launch_app_from_home_sc("Settings", "Apps")
        timeout = 50
        begin = time.time()
        while not self.d(text=itemtext).exists:
            self.d(scrollable=True).scroll.vert()
            assert(time.time() - begin > timeout)
            if self.d(text=itemtext).exists:
                break
        self.d(text=itemtext).click.wait()

    def get_location_status(self):
        enable = False
        if (self.d(text="ON", \
                className="android.widget.Switch").exists):
            print "Initialization, current location  status is ON"
            enable = True
        else:
            print "Initialization, current location  status is OFF"
        return enable

    def turn_location_icon_on(self):
        if (self.d(text="OFF", \
                className="android.widget.Switch").exists):
            self.d(text="OFF", \
                className="android.widget.Switch").click.wait(timeout=3000)
            if (self.d(text="Agree", \
                    className="android.widget.Button").exists):
                self.d(text="Agree", \
                    className="android.widget.Button")\
                    .click.wait(timeout=3000)
        time.sleep(3)
        print "Turn Location status from OFF to ON successfully."

    def turn_location_icon_off(self):
        if (self.d(text="ON", className="android.widget.Switch").exists):
            self.d(text="ON", \
                className="android.widget.Switch").click.wait(timeout=3000)
        time.sleep(3)
        print "Turn Location status from ON to OFF successfully."

    def register_chrome_watcher(self):
        self.d.watcher("CHROME_OK").when(text="OK", \
            className="android.widget.Button") \
            .click(text="OK", className="android.widget.Button")

    def unregister_chrome_watcher(self):
        self.d.watcher("CHROME_OK").remove()

    def check_chrome_watcher(self):
        return self.d.watcher("CHROME_OK").triggered

    def download_desc(self, desc):
        if self.d(description=desc).exists:
            rx = (self.d(description=desc).bounds["right"] + \
                    self.d(description=desc).bounds["left"]) / 2
            ry = (self.d(description=desc).bounds["top"] + \
                    self.d(description=desc).bounds["bottom"]) / 2
            self.d.drag(rx, ry, rx, ry)
            self.d(text="Save link").click()
            time.sleep(30)

    def start_downloads_by_am(self):
        g_common_obj.launch_app_am(ChromeImpl.downloads_pkg_name, \
            ChromeImpl.downloads_activity_name)

    def stop_downloads_by_am(self):
        g_common_obj.stop_app_am(ChromeImpl.downloads_pkg_name)

    def set_downloads_list_view(self):
        if self.d(description="More options").exists:
            self.d(description="More options").click()
            time.sleep(1)
            if self.d(text="List view", \
                    className="android.widget.TextView").exists:
                self.d(text="List view", \
                    className="android.widget.TextView").click()
                time.sleep(1)
            else:
                self.d.press.back()

    def open_download_image(self, imagetext):
        if self.d(text=imagetext).exists:
            self.d(text=imagetext).click()
            time.sleep(1)
            self.d(text="Photos", \
                className="android.widget.TextView").click()
            time.sleep(1)
        if self.d(text="Just once").exists:
            self.d(text="Just once").click()
            time.sleep(1)

    def delete_download_image(self, imagetext):
        self.start_downloads_by_am()
        time.sleep(1)
        if self.d(text=imagetext).exists:
            rx = (self.d(text=imagetext).bounds["right"] + \
                    self.d(text=imagetext).bounds["left"]) / 2
            ry = (self.d(text=imagetext).bounds["top"] + \
                    self.d(text=imagetext).bounds["bottom"]) / 2
            self.d.drag(rx, ry, rx, ry)
            if self.d(resourceId="com.android.documentsui:id/menu_delete").exists:
                self.d(resourceId="com.android.documentsui:id/menu_delete").click()
                time.sleep(1)
                self.d.press.back()
            self.d.press.home()

    def restore_wall_paper(self):
        itemtext = "Display"
        self.launch_item_setting_from_app_gallery(itemtext)
        time.sleep(1)
        self.d(text="Wallpaper", \
            className="android.widget.TextView").click()
        time.sleep(1)
        self.d(text="Wallpapers", \
            className="android.widget.TextView").click()
        time.sleep(1)
        self.d(text="Set wallpaper", \
            className="android.widget.Button").click()
        time.sleep(1)
        self.d.press.back()
        self.d.press.back()
        self.d.press.home()
