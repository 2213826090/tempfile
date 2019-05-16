# -*- coding: utf-8 -*-

from PyUiApi.app_utils.chrome_utils import *
from PyUiApi.app_utils.settings_utils import *
from PyUiApi.app_utils.widgets_utils import *
from PyUiApi.app_utils.clock_utils import *
from PyUiApi.app_utils.calendar_utils import *
from PyUiApi.app_utils.keyboard_utils import *
from PyUiApi.common.test_utils import *
from PyUiApi.common.shell_utils import *
from PyUiApi.common.status_bar import *
from PyUiApi.common.system_utils import *
from PyUiApi.app_utils.downloads_utils import *
from selenium.common.exceptions import WebDriverException
from selenium import webdriver
from selenium.webdriver import ActionChains
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait  # available since 2.4.0
from selenium.webdriver.support import expected_conditions as EC  # available since 2.26.0
from datetime import datetime
from threading import Thread
import re
import sys

reload(sys)
sys.setdefaultencoding("utf-8")

default_capabilities = {
  'chromeOptions': {
    'androidPackage': 'com.android.chrome',
  }
}

paste_test_app_name = "InputMethodTests"


class ChromeTests(unittest.TestCase):
    driver = None
    chromedriver_utils = None
    screen_shooter = None
    files_acces_allowed = False
    use_running_app = False

    def setUp(self):
        UiAutomatorUtils.unlock_screen()
        self.chromedriver_utils = ChromeDriverUtils()
        if self.chromedriver_utils.is_chromedriver_running():
            self.chromedriver_utils.kill_chromedriver()
        self.chromedriver_utils.resolve_dependencies()
        self.chromedriver_utils.start_chromedriver()
        self.driver = self.start_webdriver()
        self.assertIsNotNone(self.driver)
        self.grant_needed_permissions()
        self.screen_shooter = ScreenshotUtils()
        self.initial_orientation = d.orientation

    def start_webdriver(self):
        driver = None
        for i in range(40):
            time.sleep(2)
            try:
                default_capabilities["chromeOptions"]["androidDeviceSerial"]\
                    = dut_manager.active_uiautomator_device_serial
                driver = webdriver.Remote('http://localhost:9515', default_capabilities)
                break
            except WebDriverException as e:
                LOG.info("failed to start chromedriver, try: %s" % str(i))
        return driver

    def grant_needed_permissions(self):
        AdbUtils.grant_permission(CHROME_PACKAGE_NAME, PERMISSION_WRITE_EXTERNAL_STORAGE)

    def tearDown(self):
        try:
            self.log_before_cleanup()
            Chrome.close_all_tabs_and_count(self.driver)
            self.driver.quit()
            self.chromedriver_utils.stop_chromedriver()
        except:
            LOG.info("exception while shutting down Chrome")
        try:
            UiAutomatorUtils.close_all_tasks()
            self.screen_shooter.remove_all_screenshots()
        except:
            LOG.info("exception while task and scrennshot cleaning")
        if d.orientation != self.initial_orientation:
            d.orientation = self.initial_orientation
            d.freeze_rotation(False)
        if self.chromedriver_utils.is_chromedriver_running():
            self.chromedriver_utils.kill_chromedriver()

    def log_before_cleanup(self):
        result = self._resultForDoCleanups
        if not result.wasSuccessful():
            TestUtils.log_screenshot_and_dump()

    def download_file(self, url, create_new_file=False):
        LOG.info("making sure download dir exists: ")
        AdbUtils.create_dir("/sdcard/Download")
        LOG.info("downloading: " + url)
        self.driver.get(url)
        Chrome.clear_chrome_allow_access_files()
        if not create_new_file:
            LOG.info("replacing file")
            Chrome.clear_replace_files()
        else:
            LOG.info("creating new file")
            if d(text=CHROME_CREATE_NEW_FILE_TXT).wait.exists(timeout=3000):
                d(text=CHROME_CREATE_NEW_FILE_TXT).click()

    def test_alarm_clock_alert_when_clear_history(self):
        # get a few items in the history
        self.driver.get('http://google.com')
        self.driver.get('http://www.bing.com')
        self.driver.get('http://wikipedia.org')
        self.driver.get('http://reddit.com')
        # set the alarm to expire in a minute from now
        hh, mm, ss = ShellUtils.get_current_dut_time()
        try:
            alarm_time = Clock.set_new_alarm(hh, mm)
            Chrome.launch()
            self.driver.get('chrome://history/')
            hh, mm, ss = ShellUtils.get_current_dut_time()
            current_time = datetime(1, 1, 1, int(hh), int(mm), int(ss))
            time_delta = alarm_time - current_time
            time.sleep(time_delta.seconds + 15)
            StatusBar.open_notifications()
            self.assertTrue(d(text=CLOCK_ALARM_TXT).wait.exists(timeout=3000))
            self.assertTrue(d(text=CLOCK_DISMISS_ALARM_TXT).wait.exists(timeout=3000))
            d(text=CLOCK_DISMISS_ALARM_TXT).click()
            d.press.back()
            # Clear browsing data
            self.assertTrue(self.clear_browsing_data(), "could not clear browsing data")
            time.sleep(2)
            self.driver.get('chrome://history/')
            history_items = self.driver.find_elements(By.XPATH, '//button[@class="remove-entry custom-appearance"]')
            self.assertTrue(len(history_items) == 0)
        finally:
            Clock.delete_all_alarms()

    def test_calendar_alert_when_manage_history(self):
        # make sure we have some items in the Browser's history
        self.driver.get('http://wikipedia.org')
        self.driver.get('http://reddit.com')
        self.driver.get('http://bing.com')
        # set an calendar notification in a few minutes
        Calendar.create_calendar_event("chrome_test_event", "chrome_test_event", 70, 30)
        time.sleep(30)
        # access the browser's history
        Chrome.launch()
        self.driver.get('chrome://history/')
        history_items = self.driver.find_elements(By.XPATH, '//button[@class="remove-entry custom-appearance"]')
        self.assertTrue(len(history_items) >= 3)
        time.sleep(5)
        # Clear browsing data
        self.assertTrue(self.clear_browsing_data(), "could not clear browsing data")
        self.driver.get('chrome://history/')
        history_items = self.driver.find_elements(By.XPATH, '//button[@class="remove-entry custom-appearance"]')
        self.assertTrue(len(history_items) == 0)

    def test_chat_im(self):
        # login to google
        self.driver.get('https://accounts.google.com')
        username = "go4intel@gmail.com"
        password = "testing12345"
        self.driver.find_element(By.XPATH, '//input[@id="Email"]').send_keys(username)
        self.driver.find_element(By.XPATH, '//input[@id="next"]').click()
        time.sleep(2)
        self.driver.find_element(By.XPATH, '//input[@id="Passwd"]').send_keys(password)
        with WaitForPageLoad(self.driver):
            self.driver.find_element(By.XPATH, '//input[@id="signIn"]').click()
        time.sleep(2)
        self.driver.get("https://plus.google.com/hangouts")
        time.sleep(2)
        WebDriverWait(self.driver, 20)\
            .until(EC.presence_of_element_located((By.XPATH, '//input[@aria-label="New Conversation"]')))
        time.sleep(5)

    def retry_webelement_click(self, nr_of_tries, element_xpath):
        success = False
        for i in range(nr_of_tries):
            try:
                self.driver.find_element(By.XPATH, element_xpath).click()
                success = True
                break
            except WebDriverException as e:
                time.sleep(1)
        return success

    def test_remove_page_from_history(self):
        self.driver.get('http://google.com')
        self.driver.get('http://www.bing.com/')
        self.driver.get('chrome://history/')
        time.sleep(2)
        visited_pages = self.driver.find_elements(By.XPATH, '//div[@class="title"]')
        initial_visited_pages = len(visited_pages)
        print "initial nr. of pages: ", initial_visited_pages
        time.sleep(2)
        remove_handle = self.driver.find_element(By.XPATH, '//button[@class="remove-entry custom-appearance"]')
        remove_handle.click()
        time.sleep(2)
        visited_pages = self.driver.find_elements(By.XPATH, '//div[@class="title"]')
        final_visited_pages = len(visited_pages)
        print "final nr. of pages: ", final_visited_pages
        self.assertTrue(final_visited_pages < initial_visited_pages)

    def test_remove_website_from_most_visited(self):
        self.driver.get('http://google.com')
        self.driver.get('http://www.bing.com/')
        Chrome.open_new_tab()
        if d(resourceId=CHROME_MOST_VISITED_TITLE_RESID, text="Bing").wait.exists(timeout=3000):
            d(resourceId=CHROME_MOST_VISITED_TITLE_RESID, text="Bing").long_click()
        if d(text="Remove").wait.exists(timeout=3000):
            d(text="Remove").click()
        self.assertTrue(not d(resourceId=CHROME_MOST_VISITED_TITLE_RESID, text="Bing").exists)
        time.sleep(1)
        Chrome.close_current_tab()
        time.sleep(2)
        self.driver.get('chrome://history/')
        self.driver.find_element(By.XPATH, '//a[@title="Bing"]')

    def wait_for_element(self, seconds, element_xpath):
        for i in range(seconds):
            time.sleep(1)
            found = self.driver.find_elements(By.XPATH, element_xpath)
            if len(found) > 0:
                return True
        return False

    def test_open_page_from_most_visited(self):
        self.driver.get('http://google.com')
        self.driver.get('http://www.bing.com/')
        self.driver.get('http://google.com/')
        Chrome.open_new_tab()
        time.sleep(3)
        if d(resourceId=CHROME_MOST_VISITED_TITLE_RESID, text="Bing").wait.exists(timeout=10000):
            d(resourceId=CHROME_MOST_VISITED_TITLE_RESID, text="Bing").click()
        time.sleep(5)
        current_address = Chrome.get_current_tab_url()
        self.assertTrue("bing" in current_address)

    def test_check_block_popups(self):
        self.driver.get('http://popuptest.com/')
        Chrome.toggle_popups(enable=True)
        Chrome.return_to_main_screen()
        self.driver.get('http://popuptest.com/popuptest1.html')
        time.sleep(5)
        current_url = Chrome.get_current_tab_url()
        self.assertTrue("popuptest1" not in current_url)
        nr_tabs = Chrome.close_all_tabs_and_count(self.driver)
        self.assertTrue(nr_tabs > 4)
        UiAutomatorUtils.close_all_tasks()
        Chrome.launch()
        self.driver.switch_to.window(self.driver.window_handles[0])
        self.driver.get('http://popuptest.com/')
        Chrome.toggle_popups(enable=False)
        Chrome.return_to_main_screen()
        self.driver.get('http://popuptest.com/popuptest1.html')
        time.sleep(5)
        current_url = Chrome.get_current_tab_url()
        self.assertTrue("popuptest1" in current_url)
        nr_tabs = Chrome.close_all_tabs_and_count(self.driver)
        self.assertTrue(nr_tabs < 3)

    def test_check_cookies(self):
        # disable cookies and test
        self.driver.get('http://valeaplopului.com/chrome_tests/cookies_test.html')
        Chrome.toggle_cookies(enable=False)
        Chrome.return_to_main_screen()
        self.driver.get('http://valeaplopului.com/chrome_tests/cookies_test.html')
        initial_nr_of_cookies = int(self.driver.find_element(By.XPATH, "//div[@id='nrOfCookies']").text)
        for i in range(5):
            self.driver.find_element(By.XPATH, '//input[@id="makeCookie"]').click()
            time.sleep(1)
        self.driver.find_element(By.XPATH, "//input[@id='makeCookie']").click()
        final_nr_of_cookies = int(self.driver.find_element(By.XPATH, "//div[@id='nrOfCookies']").text)
        LOG.info("initial nr of cookies: %s vs. final nr of cookies: %s" %
                 (initial_nr_of_cookies, final_nr_of_cookies))
        self.assertTrue(initial_nr_of_cookies == final_nr_of_cookies)
        # enable cookies and test
        self.driver.get('http://valeaplopului.com/chrome_tests/cookies_test.html')
        Chrome.toggle_cookies(enable=True)
        Chrome.return_to_main_screen()
        self.driver.get('http://valeaplopului.com/chrome_tests/cookies_test.html')
        initial_nr_of_cookies = int(self.driver.find_element(By.XPATH, "//div[@id='nrOfCookies']").text)
        for i in range(5):
            self.driver.find_element(By.XPATH, '//input[@id="makeCookie"]').click()
            time.sleep(1)
        self.driver.find_element(By.XPATH, "//input[@id='makeCookie']").click()
        final_nr_of_cookies = int(self.driver.find_element(By.XPATH, "//div[@id='nrOfCookies']").text)
        LOG.info("initial nr of cookies: %s vs. final nr of cookies: %s" %
                 (initial_nr_of_cookies, final_nr_of_cookies))
        self.assertTrue(initial_nr_of_cookies < final_nr_of_cookies)

    def test_open_link_in_new_tab(self):
        self.driver.get('http://www.w3schools.com')
        action_chains = ActionChains(self.driver)
        initial_nr_of_tabs = len(self.driver.window_handles)
        links = self.driver.find_elements(By.XPATH, '//a')
        test_link = None
        for link in links:
            if "www.w3schools.com" in link.get_attribute('href'):
                test_link = link
        action_chains.context_click(test_link).perform()
        if d(text=CHROME_OPEN_LINK_IN_NEW_TAB).wait.exists(timeout=3000):
            d(text=CHROME_OPEN_LINK_IN_NEW_TAB).click()
        time.sleep(2)
        final_nr_of_tabs = len(self.driver.window_handles)
        self.assertTrue(final_nr_of_tabs > initial_nr_of_tabs)

    def test_context_menu(self):
        # test hyperlink context menu
        self.driver.get('http://www.w3schools.com/html/html_links.asp')
        action_chains = ActionChains(self.driver)
        links = self.driver.find_elements(By.XPATH, '//a')
        test_link = links[0]
        # long press link 1st try
        action_chains.context_click(test_link).perform()
        time.sleep(1)
        self.assertTrue(Chrome.is_context_menu_showing(), "context menu should be showing")
        d.press.back()
        self.assertTrue(not Chrome.is_context_menu_showing(), "context menu shouldn't be showing")
        # long press link 2nd try
        action_chains = ActionChains(self.driver)
        action_chains.context_click(test_link).perform()
        time.sleep(1)
        self.assertTrue(Chrome.is_context_menu_showing(), "context menu should be showing")
        d.press.back()
        self.assertTrue(not Chrome.is_context_menu_showing(), "context menu shouldn't be showing")
        # relaunch browser
        d.press.home()
        Chrome.launch()
        action_chains = ActionChains(self.driver)
        action_chains.context_click(test_link).perform()
        time.sleep(1)
        self.assertTrue(Chrome.is_context_menu_showing(), "context menu should be showing")
        d.press.back()
        self.assertTrue(not Chrome.is_context_menu_showing(), "context menu shouldn't be showing")
        # test image context menu
        self.driver.get('http://www.w3schools.com/html/html_images.asp')
        action_chains = ActionChains(self.driver)
        images = self.driver.find_elements(By.XPATH, '//img')
        test_image = images[0]
        action_chains = ActionChains(self.driver)
        action_chains.context_click(test_image).perform()
        time.sleep(1)
        self.assertTrue(Chrome.is_image_context_menu_showing(), "context menu should be showing")
        d.press.back()
        self.assertTrue(not Chrome.is_image_context_menu_showing(), "context menu shouldn't be showing")

    def test_open_many_tabs(self):
        self.driver.get('http://popuptest.com/')
        for i in range(30):
            Chrome.open_new_tab()
        opened_tabs = Chrome.close_all_tabs_and_count(self.driver)
        self.assertTrue(opened_tabs > 29)

    def test_display_image_when_clicked_in_the_webpage(self):
        self.driver.get('http://valeaplopului.com/chrome_tests/image_click.html')
        time.sleep(2)
        image = self.driver.find_element(By.XPATH, '//img')
        image.click()
        time.sleep(2)
        self.assertTrue(len(self.driver.window_handles) < 2)

    def test_save_bookmark_in_incognito_tab(self):
        self.driver.get('http://google.com')
        Chrome.close_current_tab()
        Chrome.open_new_incognito_tab()
        self.driver.switch_to.window(self.driver.window_handles[0])
        self.driver.get('http://reddit.com')
        Chrome.bookmark_current_page()
        Chrome.open_bookmarks()
        self.assertTrue(d(textStartsWith="reddit").wait.exists(timeout=3000))

    def test_history_items_sorted_by_time(self):
        self.driver.get('http://google.com')
        self.driver.get('http://www.bing.com/')
        self.driver.get('chrome://history/')
        time.sleep(2)
        history_days = self.driver.find_elements(By.XPATH, '//h3[@class="day"]')
        found_today = False
        for elem in history_days:
            print elem.text
            if "Today" in elem.text:
                found_today = True
        self.assertTrue(found_today)

    def test_web_worker(self):
        self.driver.get('http://html5demos.com/worker')
        toggle_worker = self.driver.find_element(By.XPATH, '//input[@id="toggleWorker"]')
        toggle_worker.click()
        time.sleep(10)
        toggle_worker.click()
        time.sleep(10)
        worker_msg = self.driver.find_element(By.XPATH, '//div[@id="status"]')
        print worker_msg.text
        self.assertTrue("Starting..." in worker_msg.text and "Stopping..." in worker_msg.text)

    def test_https_protocol(self):
        self.driver.get('https://accounts.google.com')
        username = "go4intel"
        password = "testing12345"
        self.driver.find_element(By.XPATH, '//input[@id="Email"]').send_keys(username)
        self.driver.find_element(By.XPATH, '//input[@id="next"]').click()
        time.sleep(2)
        self.driver.find_element(By.XPATH, '//input[@id="Passwd"]').send_keys(password)
        with WaitForPageLoad(self.driver):
            self.driver.find_element(By.XPATH, '//input[@id="signIn"]').click()
        time.sleep(2)

    def test_incognito_tab_history(self):
        self.driver.get('chrome://history/')
        time.sleep(2)
        visited_pages = self.driver.find_elements(By.XPATH, '//div[@class="title"]')
        initial_visited_pages = len(visited_pages)
        LOG.info("initial nr. of pages: " + str(initial_visited_pages), class_ref=self)
        time.sleep(2)
        remove_handles = self.driver.find_elements(By.XPATH, '//button[@class="remove-entry custom-appearance"]')
        for handle in remove_handles:
            handle.click()
        Chrome.close_all_tabs_and_count(self.driver)
        Chrome.open_new_incognito_tab()
        self.driver.switch_to.window(self.driver.window_handles[0])
        self.driver.get('http://reddit.com')
        self.driver.get('chrome://history/')
        visited_pages = self.driver.find_elements(By.XPATH, '//div[@class="title"]')
        final_visited_pages = len(visited_pages)
        self.assertTrue(final_visited_pages <= initial_visited_pages)

    def test_paste_text(self):
        test_page_background_color = (153, 1, 1)
        self.driver.get('http://valeaplopului.com/chrome_tests/paste_test.html')
        # change orientation to make sure the action menu is fully displayed on long click
        OrientationChanger.change_orientation('l')
        width, height = UiAutomatorUtils.get_screen_dims()
        d.long_click(width / 2, height / 2)
        self.screen_shooter.take_screenshot()
        background_pixels = self.screen_shooter.get_all_pixels_of_color(test_page_background_color, color_error=10)
        sorted_background_pixels = sorted(background_pixels, key=lambda pixel: int(pixel.x + pixel.y))
        first_pixel = sorted_background_pixels[0]
        last_pixel = sorted_background_pixels[-1]
        LOG.info("first background pixel: %s %s" % (str(first_pixel.x), str(first_pixel.y)))
        LOG.info("last background pixel: %s %s" % (str(last_pixel.x), str(last_pixel.y)))
        action_menu_pixels = self.screen_shooter.get_all_pixels_of_color(COLOR_WHITE, color_error=10,
                                                                         min_x=first_pixel.x, min_y=first_pixel.y,
                                                                         max_x=last_pixel.x, max_y=last_pixel.y)
        sorted_action_menu_pixels = sorted(action_menu_pixels, key=lambda pixel: int(pixel.x + pixel.y))
        first_pixel = sorted_action_menu_pixels[0]
        last_pixel = sorted_action_menu_pixels[-1]
        LOG.info("first action menu pixel: %s %s" % (str(first_pixel.x), str(first_pixel.y)))
        LOG.info("last action menu pixel: %s %s" % (str(last_pixel.x), str(last_pixel.y)))
        ActionMenu.copy(first_pixel.x, first_pixel.y, last_pixel.x, last_pixel.y)
        time.sleep(2)
        UiAutomatorUtils.launch_app_from_apps_menu(paste_test_app_name)
        if d(resourceId=API_TESTS_EDIT_RESID).wait.exists(timeout=3000):
            edit_text_bounds = Bounds(d(resourceId=API_TESTS_EDIT_RESID).info)
            UiAutomatorUtils.long_click(d(resourceId=API_TESTS_EDIT_RESID), 2)
            time.sleep(3)
            # click approximately where the PASTE dialog appears
            ActionMenu.click_paste(edit_text_bounds)
            time.sleep(1)
        pasted_text = d(resourceId=API_TESTS_EDIT_RESID).info["text"]
        self.assertTrue(len(pasted_text) > 30, "copy paste from browser did not work")

    def test_new_tab(self):
        self.driver.get('http://bing.com')
        initial_nr_of_tabs = len(self.driver.window_handles)
        Chrome.open_new_tab()
        time.sleep(2)
        final_nr_of_tabs = len(self.driver.window_handles)
        self.assertTrue(initial_nr_of_tabs < final_nr_of_tabs)
        # open many tabs to get the new tab button to the right
        for i in range(4):
            Chrome.open_new_tab()
        time.sleep(2)
        menu_button_bounds = Bounds(d(resourceId=CHROME_OPTION_RESID).info)
        # try to compute a point from the new tab button
        y = menu_button_bounds.top - (menu_button_bounds.bottom - menu_button_bounds.top) / 3
        x = menu_button_bounds.left
        initial_nr_of_tabs = len(self.driver.window_handles)
        print x, y
        d.click(x, y)
        time.sleep(3)
        final_nr_of_tabs = len(self.driver.window_handles)
        Chrome.close_all_tabs_and_count(self.driver)
        self.assertTrue(initial_nr_of_tabs < final_nr_of_tabs)

    def test_request_desktop_site(self):
        self.driver.get('http://support.mozilla.org')
        LOG.info("current url: " + self.driver.current_url, self)
        initial_url = self.driver.current_url
        Chrome.open_chrome_menu()
        if d(text=CHROME_REQ_DESKTOP_SITE_TXT).wait.exists(timeout=3000):
            d(text=CHROME_REQ_DESKTOP_SITE_TXT).click()
        time.sleep(3)
        self.driver.switch_to.window(self.driver.window_handles[0])
        self.driver.get('http://support.mozilla.org')
        LOG.info("current url: " + self.driver.current_url, self)
        final_url = self.driver.current_url
        Chrome.open_chrome_menu()
        if d(text=CHROME_REQ_DESKTOP_SITE_TXT).wait.exists(timeout=3000):
            d(text=CHROME_REQ_DESKTOP_SITE_TXT).click()
        self.assertTrue(initial_url != final_url, "request desktop site does not seem to work")

    def test_history_drop_down_list(self):
        self.driver.get('http://reddit.com')
        time.sleep(1)
        self.driver.get('http://wikipedia.com')
        time.sleep(1)
        self.driver.get('http://bing.com')
        time.sleep(1)
        if d(resourceId=CHROME_ADDRESS_BAR_RESID).wait.exists(timeout=3000):
            d(resourceId=CHROME_ADDRESS_BAR_RESID).click()
        time.sleep(3)
        d.press("delete")
        time.sleep(3)
        d.press(KEYCODE_H)
        time.sleep(3)
        history_item_1 = Chrome.get_current_tab_url()
        d.press("down")
        time.sleep(3)
        history_item_2 = Chrome.get_current_tab_url()
        d.press("down")
        time.sleep(3)
        history_item_3 = Chrome.get_current_tab_url()
        d.press("down")
        time.sleep(3)
        history_item_4 = Chrome.get_current_tab_url()
        aggregate_url = history_item_1 + " " + history_item_2 + " " + history_item_3 + " " + history_item_4
        LOG.info("urls in history: " + aggregate_url, self)
        self.assertTrue("bing" in aggregate_url and "wikipedia" in aggregate_url and "reddit" in aggregate_url,
                        "some sites could not be found in current history")

    def test_chrome_the_history(self):
        links = ["http://reddit.com", "http://wikipedia.com", "http://google.com",
                 "http://facebook.com", "http://bing.com"]
        for link in links:
            self.driver.get(link)
            time.sleep(2)
        self.driver.get('chrome://history/')
        history_links = self.driver.find_elements(By.XPATH, '//a')
        history_links_urls = [x.get_attribute('href') for x in history_links]
        link_names = [x.replace('.com', '').replace('http://', '') for x in links]

        # Navigate through History and view some records
        for name in link_names:
            is_in_history = False
            for url in history_links_urls:
                if name in url:
                    is_in_history = True
                    break
            self.assertTrue(is_in_history, "could not find " + str(url) + " in history")

        # Choose a record to render the page
        # self.screen_shooter.take_screenshot()
        # bing_icon_yellow = (255, 185, 0)
        # x, y = self.screen_shooter.search_for_first_pixel_of_color(bing_icon_yellow)
        # d.click(x, y)
        d(descriptionContains="Bing").wait.exists(timeout=3000)
        d(descriptionContains="Bing").click()
        time.sleep(4)
        self.assertTrue("bing" in Chrome.get_current_tab_url(), "could not find Bing site in history")

        # Return to History, clear a record
        self.driver.get('chrome://history/')
        first_history_item = self.driver.find_element(By.XPATH, '//li')
        initial_page_link = first_history_item.find_element(By.XPATH, '//a').get_attribute('href')
        remove_button = first_history_item.find_element(By.XPATH, '//button[@class="remove-entry custom-appearance"]')
        remove_button.click()
        time.sleep(2)
        first_history_item = self.driver.find_element(By.XPATH, '//li')
        final_page_link = first_history_item.find_element(By.XPATH, '//a').get_attribute('href')
        time.sleep(1)
        self.assertTrue(initial_page_link != final_page_link)

        # Clear browsing data
        self.assertTrue(self.clear_browsing_data(), "could not clear browsing data")
        time.sleep(2)
        Chrome.close_all_tabs_and_count(self.driver)
        Chrome.open_new_tab()
        self.driver.switch_to.window(self.driver.window_handles[0])
        self.driver.get('chrome://history/')
        history_items = self.driver.find_elements(By.XPATH, '//button[@class="remove-entry custom-appearance"]')
        self.assertTrue(len(history_items) == 0, "could not clear history")

    def clear_browsing_data(self):
        self.driver.get('chrome://history/')
        clear_history = self.driver.find_element(By.XPATH, '//button[@id="clear-browsing-data"]')
        clear_history.click()
        time.sleep(2)
        for clear_data_text in ["CLEAR DATA", "Clear data", "Clear"]:
            if d(text=clear_data_text).wait.exists(timeout=3000):
                return d(text=clear_data_text).click()
        return False

    def test_html5_video(self):
        self.driver.get('http://html5demos.com/video')
        print self.driver.execute_script('return document.getElementsByTagName("video")[0].currentSrc;')
        video_ready = False
        for i in range(20):
            time.sleep(1)
            ready_state = self.driver.execute_script('return document.getElementsByTagName("video")[0].readyState;')
            if int(ready_state) == 4:
                video_ready = True
                break
        self.assertTrue(video_ready)
        # check that the video is PAUSED
        self.assertTrue(self.driver.execute_script('return document.getElementsByTagName("video")[0].paused;'))
        # START the video by pressing the play button
        self.driver.find_element(By.XPATH, '//input[@id="play"]').click()
        time.sleep(3)
        # check that the video is NOT PAUSED
        self.assertTrue(not self.driver.execute_script('return document.getElementsByTagName("video")[0].paused;'))
        # click the play button againg to PAUSE the video
        self.driver.find_element(By.XPATH, '//input[@id="play"]').click()
        time.sleep(10)
        self.assertTrue(self.driver.execute_script('return document.getElementsByTagName("video")[0].paused;'))
        # START the video again
        self.driver.find_element(By.XPATH, '//input[@id="play"]').click()
        time.sleep(3)
        # activate the FULLSCREEN mode
        self.driver.find_element(By.XPATH, '//input[@value="fullscreen"]').click()
        time.sleep(3)
        # check that the video is NOT PAUSED
        self.assertTrue(not self.driver.execute_script('return document.getElementsByTagName("video")[0].paused;'))
        time.sleep(3)
        # deactivate the FULLSCREEN mode
        d.press.back()
        time.sleep(3)
        # check that the video is NOT PAUSED
        self.assertTrue(not self.driver.execute_script('return document.getElementsByTagName("video")[0].paused;'))
        current_time = self.driver.execute_script('return document.getElementsByTagName("video")[0].currentTime;')
        duration = self.driver.execute_script('return document.getElementsByTagName("video")[0].duration;')
        self.assertTrue(current_time >= 1)  # video has been playing for more than 15 seconds
        self.assertTrue(duration == 25)

    def test_add_bookmark_after_playing_video(self):
        bookmark_name = "After_Play_Video_Bookmark"
        self.driver.get('http://html5demos.com/video')
        video_ready = False
        for i in range(20):
            time.sleep(1)
            ready_state = self.driver.execute_script('return document.getElementsByTagName("video")[0].readyState;')
            if int(ready_state) == 4:
                video_ready = True
                break
        self.assertTrue(video_ready, "video did not become ready in time")
        # START the video by pressing the play button
        self.driver.find_element(By.XPATH, '//input[@id="play"]').click()
        time.sleep(3)
        # check that the video is NOT PAUSED
        self.assertTrue(not self.driver.execute_script('return document.getElementsByTagName("video")[0].paused;'),
                        "video did not start")
        # bookmark the page with the currently playing video
        Chrome.add_bookmark(bookmark_name)
        # open the bookmarks and try to load the video page
        Chrome.open_mobile_bookmarks()
        time.sleep(2)
        self.assertTrue(d(textStartsWith=bookmark_name).wait.exists(timeout=3000),
                        "could not find video page bookmark")
        d(textStartsWith=bookmark_name).click()
        time.sleep(3)
        # try to start the video from the bookmark loaded page
        for i in range(5):
            if len(self.driver.find_elements(By.XPATH, '//input[@id="play"]')) > 0:
                self.driver.find_element(By.XPATH, '//input[@id="play"]').click()
                time.sleep(3)
            if not self.driver.execute_script('return document.getElementsByTagName("video")[0].paused;'):
                LOG.info("video started after bookmark loaded")
                break
        # check that the video is NOT PAUSED
        self.assertTrue(not self.driver.execute_script('return document.getElementsByTagName("video")[0].paused;'),
                        "video did not start")

    def test_add_bookmark_to_specified_folder(self):
        test_link = "http://wikipedia.org"
        test_bookmark_name = "wikipedia"
        test_bookmark_folder_name = "test_folder"
        test_url_reference_string = "wikipedia.org"
        self.driver.get(test_link)
        Chrome.add_bookmark(name=test_bookmark_name, folder=test_bookmark_folder_name)
        Chrome.return_to_main_screen()
        Chrome.open_mobile_bookmarks()
        self.assertTrue(d(text=test_bookmark_folder_name).wait.exists(timeout=3000),
                        "could not find created bookmark folder")
        d(text=test_bookmark_folder_name).click()
        time.sleep(2)
        self.assertTrue(d(text=test_bookmark_name).wait.exists(timeout=3000), "could not find created bookmark")
        d(text=test_bookmark_name).click()
        time.sleep(2)
        self.assertTrue(test_url_reference_string in self.driver.current_url,
                        "bookmark click did not load the reference page")

    def test_add_bookmarks(self):
        test_link = "http://wikipedia.org"
        test_bookmark_name = "wikipedia"
        test_url_reference_string = "wikipedia.org"
        test_blank_string = ""
        self.driver.get(test_link)
        Chrome.add_bookmark(name=test_blank_string)
        # cannot create bookmark with no name, so the dialog remains the same
        UiAutomatorUtils.wait_for_compound_resource_id(CHROME_BOOKMARK_TITLE_INPUT_RESIDS)
        Chrome.return_to_main_screen()
        Chrome.add_bookmark(url=test_blank_string)
        # cannot create bookmark with no url, so the dialog remains the same
        UiAutomatorUtils.wait_for_compound_resource_id(CHROME_BOOKMARK_TITLE_INPUT_RESIDS)
        Chrome.return_to_main_screen()
        Chrome.add_bookmark(name=test_bookmark_name)
        Chrome.open_mobile_bookmarks()
        self.assertTrue(d(text=test_bookmark_name).wait.exists(timeout=3000))
        d(text=test_bookmark_name).click()
        time.sleep(2)
        self.assertTrue(test_url_reference_string in self.driver.current_url)

    def test_cancel_bookmark_folder_creation(self):
        test_link = "http://wikipedia.org"
        test_bookmark_folder_name = "test_cancel_folder_creation"
        self.driver.get(test_link)
        Chrome.add_bookmark(folder=test_bookmark_folder_name,
                            cancel_bookmark_creation=True, cancel_folder_creation=True)
        Chrome.return_to_main_screen()
        Chrome.open_mobile_bookmarks()
        self.assertFalse(d(text=test_bookmark_folder_name).wait.exists(timeout=3000))

    def test_create_bookmark_folder_with_long_name(self):
        test_link = "http://wikipedia.org"
        test_bookmark_folder_name = "test_very_long_folder_name_with_many_characters_which_should_work"
        self.driver.get(test_link)
        Chrome.add_bookmark(folder=test_bookmark_folder_name)
        Chrome.return_to_main_screen()
        Chrome.open_mobile_bookmarks()
        self.assertTrue(d(text=test_bookmark_folder_name).wait.exists(timeout=3000))

    def test_create_a_bookmark_folder(self):
        test_link = "http://wikipedia.org"
        test_bookmark_folder_name = "test_folder"
        self.driver.get(test_link)
        Chrome.add_bookmark(folder=test_bookmark_folder_name)
        Chrome.return_to_main_screen()
        Chrome.open_mobile_bookmarks()
        self.assertTrue(d(text=test_bookmark_folder_name).wait.exists(timeout=3000))

    def test_delete_bookmark_folder(self):
        test_link = "http://wikipedia.org"
        test_bookmark_folder_name = "test_folder"
        self.driver.get(test_link)
        Chrome.add_bookmark(folder=test_bookmark_folder_name)
        Chrome.return_to_main_screen()
        Chrome.open_mobile_bookmarks()
        self.assertTrue(d(text=test_bookmark_folder_name).wait.exists(timeout=3000))
        # long click bookmark folder name and delete
        Chrome.delete_bookmark(test_bookmark_folder_name)
        # deleted folder should not linger any further
        self.assertFalse(d(text=test_bookmark_folder_name).wait.exists(timeout=3000))

    def test_edit_bookmark(self):
        test_link = "http://wikipedia.org"
        test_bookmark_name = "wikipedia"
        test_renamed_bookmark_name = "wikipedia_renamed"
        self.driver.get(test_link)
        Chrome.add_bookmark(name=test_bookmark_name)
        Chrome.return_to_main_screen()
        Chrome.open_mobile_bookmarks()
        self.assertTrue(d(text=test_bookmark_name).wait.exists(timeout=3000))
        Chrome.edit_bookmark(test_bookmark_name, test_renamed_bookmark_name)
        time.sleep(2)
        self.assertTrue(d(text=test_renamed_bookmark_name).wait.exists(timeout=3000))
        Chrome.delete_bookmark(test_renamed_bookmark_name)
        self.assertFalse(d(text=test_renamed_bookmark_name).wait.exists(timeout=3000))

    def test_open_bookmark_in_current_window(self):
        test_link = "http://wikipedia.org"
        test_bookmark_name = "wikipedia"
        test_url_reference_string = "wikipedia.org"
        self.driver.get(test_link)
        initial_nr_of_tabs = len(self.driver.window_handles)
        Chrome.add_bookmark(name=test_bookmark_name)
        Chrome.return_to_main_screen()
        Chrome.open_mobile_bookmarks()
        self.assertTrue(d(text=test_bookmark_name).wait.exists(timeout=3000))
        d(text=test_bookmark_name).click()
        time.sleep(2)
        self.assertTrue(test_url_reference_string in self.driver.current_url)
        self.assertTrue(len(self.driver.window_handles) == initial_nr_of_tabs)

    def test_save_duplicate_name_bookmark(self):
        test_link = "http://wikipedia.org"
        test_bookmark_name = "wikipedia"
        test_renamed_bookmark_name = "wikipedia"
        test_url_reference_string = "wikipedia"
        self.driver.get(test_link)
        Chrome.add_bookmark(name=test_bookmark_name)
        Chrome.return_to_main_screen()
        Chrome.open_mobile_bookmarks()
        self.assertTrue(d(text=test_bookmark_name).wait.exists(timeout=3000))
        Chrome.edit_bookmark(test_bookmark_name, test_renamed_bookmark_name)
        time.sleep(2)
        self.assertTrue(d(text=test_renamed_bookmark_name).wait.exists(timeout=3000))
        d(text=test_renamed_bookmark_name).click()
        time.sleep(2)
        self.assertTrue(test_url_reference_string in self.driver.current_url)

    def test_add_bookmark_shortcut_to_homescreen(self):
        test_link = "http://wikipedia.org"
        test_bookmark_name = "wikipedia"
        self.driver.get(test_link)
        Chrome.open_chrome_menu()
        # click the add to homescreen menu option
        self.assertTrue(d(text=CHROME_ADD_TO_HOMESCREEN_TXT).wait.exists(timeout=3000))
        d(text=CHROME_ADD_TO_HOMESCREEN_TXT).click()
        time.sleep(2)
        UiAutomatorUtils.refresh_uiautomator_view_hierarchy()
        # enter a name for the screen shortcut
        self.assertTrue(d(resourceId=CHROME_ADD_TO_HOMESCREEN_TITLE_INPUT_RESID).wait.exists(timeout=3000))
        default_name = d(resourceId=CHROME_ADD_TO_HOMESCREEN_TITLE_INPUT_RESID).info["text"]
        LOG.info("default bookmark name: " + default_name)
        time.sleep(2)
        for char in default_name:
            d.press.right()
            d.press.delete()  # delete
            time.sleep(0.2)
        # add shortcut to homescreen
        self.assertTrue(d(resourceId=CHROME_ADD_TO_HOMESCREEN_TITLE_INPUT_RESID).wait.exists(timeout=3000),
                        "bookmark name edit box not found")
        UiAutomatorUtils.refresh_uiautomator_view_hierarchy()
        d(resourceId=CHROME_ADD_TO_HOMESCREEN_TITLE_INPUT_RESID).set_text(test_bookmark_name)
        d(text=CHROME_ADD_BUTTON_TXT).click()
        time.sleep(2)
        # close all apps
        UiAutomatorUtils.close_all_tasks()
        # open the homescreen shortcut and check chrome starts
        UiAutomatorUtils.open_homescreen_shortcut(test_bookmark_name)
        self.assertTrue(d(packageName=CHROME_PACKAGE_NAME).wait.exists(timeout=3000),
                        "could not open desktop shortcut bookmark")

    def test_input_selection_deletion(self):
        def get_input_text_value():
            return self.driver.execute_script('return document.getElementById("input_text").value;')

        def type_hello():
            d.press(KEYCODE_H)
            d.press(KEYCODE_E)
            d.press(KEYCODE_L)
            d.press(KEYCODE_L)
            d.press(KEYCODE_O)

        self.driver.get("http://valeaplopului.com/chrome_tests/input_text_test.html")
        self.driver.find_element(By.XPATH, '//input[@id="input_text"]').click()
        type_hello()
        time.sleep(1)
        input_text_value = get_input_text_value()
        self.assertTrue(input_text_value.lower() == "hello")
        d.press(KEYCODE_DEL)
        d.press(KEYCODE_DEL)
        time.sleep(1)
        input_text_value = get_input_text_value()
        self.assertTrue(input_text_value.lower() == "hel")

        self.driver.get("http://google.com")
        self.driver.find_element(By.XPATH, '//input[@id="lst-ib"]').click()
        type_hello()
        time.sleep(1)
        d.press(KEYCODE_ENTER)
        page_loaded = False
        for i in range(30):
            page_title = self.driver.execute_script('return document.title.toString();')
            if "hello" in page_title:
                page_loaded = True
                break
        self.assertTrue(page_loaded)
        time.sleep(2)
        Chrome.clear_obscuring_input_element()
        self.driver.find_element(By.XPATH, '//input[@id="lst-ib"]').click()
        type_hello()
        time.sleep(1)
        google_query = self.driver.execute_script('return document.getElementById("lst-ib").value;')
        self.assertTrue("hellohello" == google_query)
        d.press(KEYCODE_DEL)
        d.press(KEYCODE_DEL)
        time.sleep(1)
        google_query = self.driver.execute_script('return document.getElementById("lst-ib").value;')
        self.assertTrue("hellohel" == google_query)

    def test_play_streaming_audio(self):
        self.driver.get('http://valeaplopului.com/chrome_tests/audio_tests.html')
        # try to play the mp3 file
        self.driver.find_element(By.XPATH, '//input[@id="play_mp3"]').click()
        time.sleep(1)
        self.assertTrue(not self.driver.execute_script('return document.getElementById("mp3_audio").paused;'))
        played_audio = False
        for i in range(30):
            time.sleep(1)
            current_time = self.driver.execute_script('return document.getElementById("mp3_audio").currentTime;')
            if int(current_time) > 3:
                played_audio = True
                break
        self.assertTrue(played_audio)
        self.driver.find_element(By.XPATH, '//input[@id="pause_mp3"]').click()
        time.sleep(1)
        self.assertTrue(self.driver.execute_script('return document.getElementById("mp3_audio").paused;'))
        # try to play the wav file
        self.driver.find_element(By.XPATH, '//input[@id="play_wav"]').click()
        time.sleep(1)
        self.assertTrue(not self.driver.execute_script('return document.getElementById("wav_audio").paused;'))
        played_audio = False
        for i in range(30):
            time.sleep(1)
            current_time = self.driver.execute_script('return document.getElementById("wav_audio").currentTime;')
            if int(current_time) > 3:
                played_audio = True
                break
        self.assertTrue(played_audio)
        self.driver.find_element(By.XPATH, '//input[@id="pause_wav"]').click()
        time.sleep(1)
        self.assertTrue(self.driver.execute_script('return document.getElementById("wav_audio").paused;'))
        # try to play the ogg file - chrome does not yet support seekable audio ... so no play test
        self.driver.find_element(By.XPATH, '//input[@id="play_ogg"]').click()
        time.sleep(1)
        self.assertTrue(not self.driver.execute_script('return document.getElementById("ogg_audio").paused;'))
        self.driver.find_element(By.XPATH, '//input[@id="pause_ogg"]').click()
        time.sleep(1)
        self.assertTrue(self.driver.execute_script('return document.getElementById("ogg_audio").paused;'))

    def test_play_html5_audio(self):
        self.driver.get('http://valeaplopului.com/chrome_tests/html5_audio_test.html')
        # try to play the audio file
        self.driver.find_element(By.XPATH, '//input[@id="play_audio"]').click()
        time.sleep(1)
        self.assertTrue(not self.driver.execute_script('return document.getElementById("audio").paused;'))
        played_audio = False
        for i in range(30):
            time.sleep(1)
            current_time = self.driver.execute_script('return document.getElementById("audio").currentTime;')
            if int(current_time) > 3:
                played_audio = True
                break
        self.assertTrue(played_audio)
        self.driver.find_element(By.XPATH, '//input[@id="pause_audio"]').click()
        time.sleep(1)
        self.assertTrue(self.driver.execute_script('return document.getElementById("audio").paused;'))

    def get_webelement_center_coordinates(self, webelement_id):
        # get the page top and left from the browser UI info
        page_left, page_top = Chrome.get_html_page_top_left()
        print page_top, page_left
        self.assertTrue(page_top is not None and page_left is not None)
        top_script = 'return document.getElementById("$ID$").getBoundingClientRect()["top"];'
        left_script = 'return document.getElementById("$ID$").getBoundingClientRect()["left"];'
        width_script = 'return document.getElementById("$ID$").getBoundingClientRect()["width"];'
        height_script = 'return document.getElementById("$ID$").getBoundingClientRect()["height"];'
        # calculate the webelement dimensions
        element_top = self.driver.execute_script(top_script.replace('$ID$', webelement_id))
        element_left = self.driver.execute_script(left_script.replace('$ID$', webelement_id))
        element_width = self.driver.execute_script(width_script.replace('$ID$', webelement_id))
        element_height = self.driver.execute_script(height_script.replace('$ID$', webelement_id))
        print element_top, element_left, element_width, element_height
        center_x = page_left + element_left + element_width / 2
        center_y = page_top + element_top + element_height / 2
        return center_x, center_y

    def test_input_password(self):
        test_pass = "password"
        input_text_blue = (0, 204, 255)
        input_password_yellow = (255, 204, 0)
        self.driver.get("http://valeaplopului.com/chrome_tests/input_text_test.html")
        # go to the password field and input the test string
        self.driver.find_element(By.XPATH, '//input[@id="input_password"]').send_keys(test_pass)
        time.sleep(2)
        self.driver.find_element(By.XPATH, '//input[@id="input_text"]').send_keys(test_pass)
        time.sleep(2)
        pass_inputed_text = self.driver.execute_script('return document.getElementById("input_password").value;')
        self.assertTrue(test_pass in pass_inputed_text, "password not entered in text box")
        self.screen_shooter.take_screenshot()
        nr_of_text_color_pixels = self.screen_shooter.get_nr_of_pixels_of_color(input_text_blue)
        nr_of_pass_color_pixels = self.screen_shooter.get_nr_of_pixels_of_color(input_password_yellow)
        self.assertTrue(nr_of_pass_color_pixels != nr_of_text_color_pixels,
                        "password input seems incorrect")

    def test_text_scaling(self):
        self.driver.get("http://valeaplopului.com/chrome_tests/scaling_text_test.html")
        font1_color = (240, 128, 64)
        # font2_color = (160, 16, 176)
        # font3_color = (16, 80, 240)
        # a value of 5 means 70% text scaling
        Chrome.set_text_scaling(5)
        Chrome.return_to_main_screen()
        self.screen_shooter.take_screenshot()
        small_scaling_font1_nr_of_pixels = self.screen_shooter.get_nr_of_pixels_of_color(font1_color)

        # a value of 20 means 150% text scaling
        Chrome.set_text_scaling(20)
        Chrome.return_to_main_screen()
        self.screen_shooter.take_screenshot()
        large_scaling_font1_nr_of_pixels = self.screen_shooter.get_nr_of_pixels_of_color(font1_color)

        # a value of 11 means 100% text scaling
        Chrome.set_text_scaling(11)
        Chrome.return_to_main_screen()
        self.screen_shooter.take_screenshot()
        normal_scaling_font1_nr_of_pixels = self.screen_shooter.get_nr_of_pixels_of_color(font1_color)

        self.assertTrue(small_scaling_font1_nr_of_pixels != normal_scaling_font1_nr_of_pixels and
                        normal_scaling_font1_nr_of_pixels != large_scaling_font1_nr_of_pixels)

    def test_website_settings(self):
        cookie_value = "Cookie detected"
        self.driver.get("http://valeaplopului.com/chrome_tests/site_settings_test.html")
        # set a market cookie on the page
        self.driver.find_element(By.XPATH, '//input[@id="makeCookie"]').click()
        time.sleep(1)
        self.assertTrue(cookie_value in self.driver
                        .execute_script('return document.getElementById("cookieHolder").innerHTML;'))
        # clear the browsing data routine
        Chrome.clear_browsing_data()
        Chrome.return_to_main_screen()
        self.driver.get("http://valeaplopului.com/chrome_tests/site_settings_test.html")
        # the previously set cookie should have been cleared
        cookie_holder_value = self.driver.execute_script('return document.getElementById("cookieHolder").innerHTML;')
        LOG.info("cookie holder value: " + str(cookie_holder_value))
        self.assertTrue(cookie_holder_value is None or cookie_value not in cookie_holder_value, "cookies detected")

    def test_system_touch_zoom(self):
        self.driver.get("http://valeaplopului.com/chrome_tests/system_touch_zoom.html")
        font_color = (240, 128, 64)
        time.sleep(10)
        self.screen_shooter.take_screenshot()
        initial_nr_of_font_pixels = self.screen_shooter.get_nr_of_pixels_of_color(font_color,
                                                                                  color_error=20)
        # try to zoom in
        if d(className=CHROME_WEBKIT_CLASS).wait.exists(timeout=3000):
            d(className=CHROME_WEBKIT_CLASS).pinch.Out(percent=50)
        time.sleep(10)
        self.screen_shooter.take_screenshot()
        after_zoom_in_nr_of_font_pixels = self.screen_shooter.get_nr_of_pixels_of_color(font_color,
                                                                                        color_error=20)
        # try to zoom out
        if d(className=CHROME_WEBKIT_CLASS).wait.exists(timeout=3000):
            d(className=CHROME_WEBKIT_CLASS).pinch.In(percent=50)
        time.sleep(10)
        self.screen_shooter.take_screenshot()
        after_zoom_out_nr_of_font_pixels = self.screen_shooter.get_nr_of_pixels_of_color(font_color,
                                                                                         color_error=20)
        LOG.info("initial nr of pixels: %s , zoom in nr of pixels: %s , zoom out nr of pixels; %s" %
                 (initial_nr_of_font_pixels, after_zoom_in_nr_of_font_pixels,
                  after_zoom_out_nr_of_font_pixels))
        self.assertTrue(initial_nr_of_font_pixels != after_zoom_in_nr_of_font_pixels and
                        after_zoom_in_nr_of_font_pixels != after_zoom_out_nr_of_font_pixels)

    def test_check_force_enable_zoom(self):
        self.driver.get("http://valeaplopului.com/chrome_tests/force_zoom_test.html")
        font_color = (240, 128, 64)
        # try to zoom in
        if d(className=CHROME_WEBKIT_CLASS).wait.exists(timeout=3000):
            d(className=CHROME_WEBKIT_CLASS).pinch.Out(percent=50)
        self.screen_shooter.take_screenshot()
        initial_nr_of_font_pixels = self.screen_shooter.get_nr_of_pixels_of_color(font_color)
        # toggle force zoom
        Chrome.toggle_force_zoom()
        Chrome.return_to_main_screen()
        # try to zoom in
        if d(className=CHROME_WEBKIT_CLASS).wait.exists(timeout=3000):
            d(className=CHROME_WEBKIT_CLASS).pinch.Out(percent=50)
        time.sleep(2)
        # scroll to top page where our font lies
        self.driver.execute_script('window.scrollTo(0, 0);')
        time.sleep(2)
        self.screen_shooter.take_screenshot()
        final_nr_of_font_pixels = self.screen_shooter.get_nr_of_pixels_of_color(font_color)
        # toggle force zoom again to return to the initial state
        Chrome.toggle_force_zoom()
        Chrome.return_to_main_screen()
        self.assertTrue(initial_nr_of_font_pixels != final_nr_of_font_pixels, "zoom was not detected")

    def test_share_via_bluetooth(self):
        # choose a simple random page to share
        self.driver.get("http://valeaplopului.com/chrome_tests/force_zoom_test.html")
        Chrome.open_chrome_menu()
        vn = ViewNavigator()
        vn.nagivate_text(["Share", "Bluetooth"])
        if d(text=CHROME_TURN_ON_BLUETOOTH).wait.exists(timeout=3000):
            d(text=CHROME_TURN_ON_BLUETOOTH).click()
        choose_bluetooth_dialog_present = False
        if d(text="Choose Bluetooth device").wait.exists(timeout=15000):
            choose_bluetooth_dialog_present = True
        Chrome.return_to_main_screen()
        self.assertTrue(choose_bluetooth_dialog_present)

    def test_svg(self):
        svg_rect_color = (255, 0, 1)
        self.driver.get("http://valeaplopului.com/chrome_tests/svg_test.html")
        # see how many pixels are included in the rectangle on the page
        self.screen_shooter.take_screenshot()
        initial_nr_of_rect_pixels = self.screen_shooter.get_nr_of_pixels_of_color(svg_rect_color)
        # if svg works, the rectangle on the page is animated to grow
        time.sleep(10)
        self.screen_shooter.take_screenshot()
        final_nr_of_rect_pixels = self.screen_shooter.get_nr_of_pixels_of_color(svg_rect_color)
        # assert that the rectangle has really gotten bigger
        self.assertTrue(final_nr_of_rect_pixels > initial_nr_of_rect_pixels)

    def test_html5_webgl(self):
        self.driver.get("chrome://version")
        # self.assertTrue("--enable-webgl" in self.driver.page_source)
        # self.assertTrue("--ignore-gpu-blacklist" in self.driver.page_source)
        # self.assertTrue("--disable-gesture-requirement-for-media-playback" in self.driver.page_source)
        self.driver.get("https://www.khronos.org/registry/webgl/conformance-suites/1.0.2/webgl-conformance-tests"
                        ".html?skip=multisample-corruption.html,uniform-default-values.html")
        wait_for_tests_load = 50
        tests_loaded = False
        for i in range(wait_for_tests_load):
            time.sleep(2)
            not_loaded_yet = self.driver.execute_script('return document.getElementById("runTestsButton").disabled;')
            if not not_loaded_yet:
                tests_loaded = True
                break
        self.assertTrue(tests_loaded, "webgl tests did not load")
        self.driver.find_element(By.XPATH, '//input[@id="runTestsButton"]').click()
        for i in range(9):
            LOG.info("waiting for webgl tests ... " + str(900 - i * 100) + " seconds left to wait")
            time.sleep(100)
        webgl_tests_ended = False
        LOG.info("polling for results every 100 seconds for 15 times")
        for i in range(25):
            time.sleep(100)
            LOG.info("polling for results every 100 seconds for 15 times ... try nr.: " + str(i))
            results = self.driver.execute_script('return document.getElementById("fullresults").innerHTML;')
            if "passed" in results:
                nums = re.findall('\\d+', results)
                LOG.info(results)
                # passing criteria is more than 97% of tests should have passed
                self.assertTrue(float(nums[0]) / float(nums[1] > 0.97),
                                " there must be at least 97% pass rate")
                webgl_tests_ended = True
                break
        # check that the tests ended successfully
        self.assertTrue(webgl_tests_ended, "webgl tests are taking to long to complete")

    def test_input_after_page_zoomed(self):
        self.driver.get("http://valeaplopului.com/chrome_tests/input_text_test.html")
        if d(className=CHROME_WEBKIT_CLASS).wait.exists(timeout=3000):
            d(className=CHROME_WEBKIT_CLASS).pinch.Out(percent=50)
        time.sleep(2)
        pinch_in_text = "pinched_in"
        self.driver.find_element(By.XPATH, '//input[@id="input_text"]').send_keys(pinch_in_text)
        inputed_text = self.driver.execute_script('return document.getElementById("input_text").value;')
        self.assertTrue(pinch_in_text in inputed_text)
        Chrome.clear_obscuring_input_element()
        d(className=CHROME_WEBKIT_CLASS).pinch.In()
        time.sleep(2)
        pinch_out_text = "pinched_out"
        self.driver.find_element(By.XPATH, '//input[@id="input_text"]').send_keys(pinch_out_text)
        inputed_text = self.driver.execute_script('return document.getElementById("input_text").value;')
        self.assertTrue(pinch_out_text in inputed_text)

    def test_html5_web_storage(self):
        local_storage_data = "local_storage_data_string"
        session_storage_data = "session_storage_data_string"
        self.driver.get("http://html5demos.com/storage")
        # input information for the local and session storage
        self.driver.find_element(By.XPATH, '//input[@name="local"]').send_keys(local_storage_data)
        self.driver.find_element(By.XPATH, '//input[@name="session"]').send_keys(session_storage_data)
        # refreshed page should display the above inputed values for local and session storage
        self.driver.refresh()
        found_local_storage_item = False
        found_session_storage_item = False
        list_elements = self.driver.find_elements(By.XPATH, '//li')
        for i in range(len(list_elements)):
            if local_storage_data in list_elements[i].get_attribute('innerHTML'):
                found_local_storage_item = True
            if session_storage_data in list_elements[i].get_attribute('innerHTML'):
                found_session_storage_item = True
        self.assertTrue(found_local_storage_item)
        self.assertTrue(found_session_storage_item)
        # close the current tab and open the storage page in a new tab
        Chrome.close_all_tabs_and_count(self.driver)
        UiAutomatorUtils.close_all_tasks()
        Chrome.launch()
        Chrome.open_new_tab()
        self.driver.switch_to.window(self.driver.window_handles[0])
        self.driver.get("http://html5demos.com/storage")
        found_local_storage_item = False
        found_session_storage_item = False
        list_elements = self.driver.find_elements(By.XPATH, '//li')
        for i in range(len(list_elements)):
            if local_storage_data in list_elements[i].get_attribute('innerHTML'):
                found_local_storage_item = True
            if session_storage_data in list_elements[i].get_attribute('innerHTML'):
                found_session_storage_item = True
        self.assertTrue(found_local_storage_item)
        self.assertTrue(not found_session_storage_item)  # session storage should be empty in a new tab
        time.sleep(2)

    def test_css_validation(self):
        # open the validator page with the uri to be validated packed in the parameters
        self.driver.get("https://jigsaw.w3.org/css-validator/validator?uri=http%3A%2F%2Fwww.w3schools.com%2" +
                        "Fstdtheme.css&profile=css3&usermedium=all&warning=1&vextwarning=&lang=en")
        css_section = self.driver.find_element(By.XPATH, '//div[@id="css"]')
        css_elements = css_section.find_elements(By.XPATH, '//div[@class="selector"]')
        self.assertTrue(len(css_elements) > 100)  # there should be in fact 185 css elements
        # success is no longer tested at this time
        # success_message = "Congratulations! No Error Found."
        # self.assertTrue(success_message in self.driver.page_source)
        time.sleep(5)

    def test_click_url_when_page_loading(self):
        # firstly load a page with large picture ressources that take a long time to load
        self.driver.get('http://google.com')
        Chrome.go_to_url("http://valeaplopului.com/chrome_tests/before_page_load_test.html", timeout=None,
                         wait_for_page_to_load=False)
        self.driver.switch_to.window(self.driver.window_handles[0])
        url = self.driver.current_url
        # assert that we have the right page
        self.assertTrue("before_page_load_test.html" in url)
        # click on a link. At this point the loading of the ressources is nowhere nearly finished
        self.driver.find_element(By.XPATH, '//a[@id="link_to_other_page"]').click()
        time.sleep(2)
        # assert that clicking the link on the loading page gets us to the new page
        url = self.driver.current_url
        self.assertTrue("before_page_load_test.html" not in url)

    def test_download_from_incognito_tab(self):
        self.driver.get('http://google.com')
        # check that a new incognito tab is open
        self.screen_shooter.take_screenshot()
        # take screenshot of a normal tab
        normal_tab_image_stat = self.screen_shooter.get_current_screenshot_stat()
        Chrome.close_all_tabs_and_count(self.driver)
        Chrome.open_new_incognito_tab()
        self.screen_shooter.take_screenshot()
        # take screenshot of an incognito tab
        incognito_tab_image_stat = self.screen_shooter.get_current_screenshot_stat()
        # normal tab has mostly light pixels
        nr, ng, nb = normal_tab_image_stat.median
        # incognito tab has mostly dark pixels
        ir, ig, ib = incognito_tab_image_stat.median
        # hence, the difference betweem their median value
        self.assertTrue(nr > 2 * ir and ng > 2 + ig and nb > 2 * ib)

        # download file
        self.driver.switch_to.window(self.driver.window_handles[0])
        self.download_file('http://valeaplopului.com/chrome_tests/download_test_1.tmp')
        Downloads.launch()
        download_info = Downloads.get_download_files_info()
        # check if we have some downloaded files
        self.assertTrue(download_info is not None)
        file_finished_download = False
        # poll wait for the file to download
        for i in range(20):
            time.sleep(8)
            download_info = Downloads.get_download_files_info()
            last_downloaded_file_info = download_info[0]
            self.assertTrue("download_test_1" in last_downloaded_file_info.name)
            LOG.info(last_downloaded_file_info.name + " " + last_downloaded_file_info.summary)
            # while the file is being downloaded
            if "in progress" not in last_downloaded_file_info.summary.lower():
                file_finished_download = True
                break
        self.assertTrue(file_finished_download)
        Downloads.delete_downloaded_files()

    def test_sorting_files(self):
        try:
            # download 2 different files at a 1 minute interval (different date)
            self.download_file('http://valeaplopului.com/chrome_tests/download_test_1.tmp')
            time.sleep(60)
            self.download_file('http://valeaplopului.com/chrome_tests/download_test_4.tmp')
            time.sleep(10)
            Downloads.launch()
            # test name sorting
            Downloads.sort_downloads(DOWNLOADS_SORT_OPTION_NAME_TXT)
            download_info = Downloads.get_download_files_info()
            for i in range(len(download_info) - 1):
                LOG.info("comparing: " + download_info[i].name + " -to- " + download_info[i + 1].name)
                self.assertTrue(download_info[i].name.lower() <= download_info[i + 1].name.lower())

            # test size sorting
            Downloads.sort_downloads(DOWNLOADS_SORT_OPTION_SIZE_TXT)
            download_info = Downloads.get_download_files_info()
            size_ordered = None
            for i in range(len(download_info) - 1):
                size_ordered = True
                larger_file_size = download_info[i].size.strip()
                smaler_file_size = download_info[i + 1].size.strip()
                LOG.info("comparing sizes: " + larger_file_size + " -to- " + smaler_file_size)
                larger_file_unit = [x for x in larger_file_size if x.isalpha()]
                smaler_file_unit = [x for x in smaler_file_size if x.isalpha()]
                larger_file_unit = ''.join(larger_file_unit)
                smaler_file_unit = ''.join(smaler_file_unit)
                LOG.info("comparing units: " + larger_file_unit + " -to- " + smaler_file_unit)
                unit_ordering = {'GB': 5, 'MB': 4, 'KB': 3, 'B': 2}
                if larger_file_unit != smaler_file_unit:
                    self.assertTrue(unit_ordering[larger_file_unit] > unit_ordering[smaler_file_unit])
                    continue
                larger_file_value = float(larger_file_size.replace(larger_file_unit, ''))
                smaler_file_value = float(smaler_file_size.replace(smaler_file_unit, ''))
                self.assertTrue(larger_file_value >= smaler_file_value)
            self.assertTrue(size_ordered)

            # test date sorting
            Downloads.sort_downloads(DOWNLOADS_SORT_OPTION_DATE_TXT)
            download_info = Downloads.get_download_files_info()
            first_date = download_info[0].date.strip()
            second_date = download_info[1].date.strip()
            LOG.info("comparing dates: " + first_date + " -to- " + second_date)
            # empty date value downloads will be at the end of the list, so give them a comparable value
            if first_date == '':
                first_date = '23:59'
            if second_date == '':
                second_date = '23:59'
            try:
                later_date = datetime.strptime(first_date, '%I:%M %p')
            except ValueError:
                later_date = datetime.strptime(first_date, '%H:%M')
            try:
                earlier_date = datetime.strptime(second_date, '%I:%M %p')
            except ValueError:
                earlier_date = datetime.strptime(second_date, '%H:%M')
            self.assertTrue(later_date.hour * 60 + later_date.minute >= earlier_date.hour * 60 + earlier_date.minute)
        finally:
            Downloads.delete_downloaded_files()

    def test_exit_while_downloading(self):
        self.download_file('http://valeaplopului.com/chrome_tests/download_test_5.tmp')
        time.sleep(2)
        UiAutomatorUtils.close_all_tasks()
        Downloads.launch()
        download_info = Downloads.get_download_files_info()
        download_found = False
        for info in download_info:
            if "download_test_5" in info.name:
                download_found = True
        Downloads.delete_downloaded_files()
        self.assertTrue(download_found, "could not delete downloaded files")

    def test_download_file_larger_than_freespace(self):
        self.download_file('http://speedtest.tele2.net/100GB.zip')
        time.sleep(2)
        Downloads.launch()
        download_infos = Downloads.get_download_files_info()
        for info in download_infos:
            if "100GB" in info.name:
                self.assertTrue("Unsuccessful" in info.summary,
                                "did not find fail download status in downloads app")
        Downloads.delete_downloaded_files()

    def test_retry_delete_download_file(self):
        self.download_file('http://speedtest.tele2.net/100GB.zip')
        time.sleep(2)
        Downloads.launch()
        self.assertTrue(d(resourceId="android:id/title").wait.exists(timeout=3000),
                        "did not find downloads app")
        initial_downloaded_files = len(d(resourceId="android:id/title"))
        self.assertTrue(d(textStartsWith="100GB").wait.exists(timeout=4000),
                        "did not find failed download file")
        if d(textStartsWith="100GB").count == 1:
            d(textStartsWith="100GB").click()
        else:
            d(textStartsWith="100GB")[0].click()
        time.sleep(1)
        self.assertTrue(d(text="Retry").wait.exists(timeout=3000),
                        "did not find retry download button")
        d(text="Retry").click()
        time.sleep(1)
        self.assertTrue(d(textStartsWith="100GB").wait.exists(timeout=3000),
                        "did not find failed download file")
        d(textStartsWith="100GB").click()
        self.assertTrue(d(text="Delete").wait.exists(timeout=3000))
        d(text="Delete").click()
        time.sleep(2)
        if d(resourceId="android:id/title").exists:
            final_downloaded_files = len(d(resourceId="android:id/title"))
            self.assertTrue(final_downloaded_files < initial_downloaded_files,
                            "could not delete failed download file")
        Downloads.delete_downloaded_files()

    def test_cancel_multiple_downloads(self):
        self.download_file('http://valeaplopului.com/chrome_tests/download_test_5.tmp')
        self.download_file('http://valeaplopului.com/chrome_tests/download_test_4.tmp')
        self.download_file('http://valeaplopului.com/chrome_tests/download_test_3.tmp')
        Downloads.launch()
        self.assertTrue(d(resourceId="android:id/title").wait.exists(timeout=3000),
                        "downloads app not found")
        initial_nr_of_downloaded_files = len(d(resourceId="android:id/title"))
        Downloads.delete_downloaded_files()
        final_nr_of_downloaded_files = len(d(resourceId="android:id/title"))
        self.assertTrue(initial_nr_of_downloaded_files > final_nr_of_downloaded_files,
                        "could not delete downloaded files")

    def test_download_through_mirrors(self):
        try:
            # open office mirror download
            self.download_file('http://sourceforge.net/projects/openofficeorg.mirror/files/4.0.0/binaries/en-US/'
                               'Apache_OpenOffice_4.0.0_Win_x86_install_en-US.exe/download')
            # wait for the download to start
            time.sleep(7)
            # put DUT to sleep
            d.press.power()
            # wait a little
            time.sleep(5)
            # turn the device back on
            browser_in_focus = False
            d.press.power()
            if d(packageName=CHROME_PACKAGE_NAME).wait.exists(timeout=3000):
                browser_in_focus = True
                UiAutomatorUtils.unlock_screen()
            if d(packageName=CHROME_PACKAGE_NAME).wait.exists(timeout=3000):
                browser_in_focus = True
            Chrome.clear_chrome_popups()
            time.sleep(5)
            # check if openoffice is in the Downloads
            Downloads.launch()
            download_info = Downloads.get_download_files_info()
            found = False
            for info in download_info:
                if "openoffice" in info.name.lower():
                    found = True
                    break
            self.assertTrue(browser_in_focus, "did not find browser in focus")
            self.assertTrue(found, "mirror downloaded file not found")
        finally:
            Downloads.delete_downloaded_files()

    def test_multiple_download_task(self):
        # make sure there are no files in the download manager
        Downloads.delete_downloaded_files()
        self.download_file('http://valeaplopului.com/chrome_tests/download_test_5.tmp')
        time.sleep(2)
        self.download_file('http://valeaplopului.com/chrome_tests/download_test_4.tmp')
        time.sleep(20)
        # check if files are all in the download manager
        Downloads.launch()
        download_info = Downloads.get_download_files_info()
        # all 3 files must be in the download manager
        self.assertTrue(len(download_info) >= 2, "did not find multiple download files")
        downloads_finished_successfully = False
        for i in range(50):
            time.sleep(9)
            download_info = Downloads.get_download_files_info()
            downloads_finished_successfully = True
            for info in download_info:
                if "in progress" in info.summary.lower():
                    downloads_finished_successfully = False
                elif "unsuccessful" in info.summary.lower():
                    downloads_finished_successfully = False
                    break
            if downloads_finished_successfully:
                break
        # Cleanup
        Downloads.delete_downloaded_files()
        self.assertTrue(downloads_finished_successfully, "some downloads did not finish successfuly")

    def test_clear_download_notifications_expand_view(self):
        Downloads.delete_downloaded_files()
        Chrome.launch()
        StatusBar.open_notifications()
        initial_nr_of_notifications = StatusBar.get_nr_of_notifications()
        print initial_nr_of_notifications
        d.press.back()
        # download a small file that finishes quickly
        self.driver.get('http://valeaplopului.com/chrome_tests/img/single_color_image.jpg')
        self.assertTrue(d(className=CHROME_WEBKIT_CLASS).wait.exists(timeout=3000))
        d(className=CHROME_WEBKIT_CLASS).long_click()
        self.assertTrue(d(text="Save image").wait.exists(timeout=3000))
        d(text="Save image").click()
        time.sleep(2)
        Chrome.clear_chrome_popups()
        # download 2 files that will take some time to complete
        self.download_file('http://valeaplopului.com/chrome_tests/download_test_5.tmp')
        time.sleep(2)
        self.download_file('http://valeaplopului.com/chrome_tests/download_test_4.tmp')
        time.sleep(2)
        StatusBar.open_notifications()
        time.sleep(2)
        post_download_nr_of_notifications = StatusBar.get_nr_of_notifications()
        print post_download_nr_of_notifications
        try:
            self.assertTrue(post_download_nr_of_notifications > initial_nr_of_notifications)
            self.assertTrue(d(textContains="single_color_image").wait.exists(timeout=2000))
            if d(resourceId=STATUS_BAR_DISMISS_RESID).wait.exists(timeout=2000):
                d(resourceId=STATUS_BAR_DISMISS_RESID).click()
            time.sleep(4)
            StatusBar.close_notifications()
            StatusBar.open_notifications()
            self.assertTrue(not d(textContains="single_color_image").exists,
                            "single_color_image download notiication was not dismissed")
            final_nr_of_notifications = StatusBar.get_nr_of_notifications()
            self.assertTrue(final_nr_of_notifications < post_download_nr_of_notifications)
            d.press.back()
        finally:
            # cleanup
            Downloads.delete_downloaded_files()

    def test_tba_download_files(self):
        try:
            self.driver.get('http://valeaplopului.com/chrome_tests/img/single_color_image.jpg')
            self.assertTrue(d(className=CHROME_WEBKIT_CLASS).wait.exists(timeout=3000))
            d(className=CHROME_WEBKIT_CLASS).long_click()
            self.assertTrue(d(text="Save image").wait.exists(timeout=3000), "save image dialog did not show")
            d(text="Save image").click()
            time.sleep(2)
            Chrome.clear_chrome_popups()
            StatusBar.open_notifications()
            self.assertTrue(d(textStartsWith="single_color_image").wait.exists(timeout=3000),
                            "download notification not visible")
            d(textStartsWith="single_color_image").click()
            if d(text="Photos").wait.exists(timeout=3000):
                d(text="Photos").click()
                d(text="Photos").click()
            self.assertTrue(d(packageName=GMS_PHOTOS_PACKAGE).wait.exists(timeout=3000) or
                            d(packageName=PHOTOS_PACKAGE_NAME).wait.exists(timeout=3000),
                            "downloaded image could not be opened")
            self.screen_shooter.take_screenshot()
            # define the color of the image downloaded (it is a solid color image)
            test_image_color = (200, 101, 0)
            nr = self.screen_shooter.get_nr_of_pixels_of_color(test_image_color)
            self.assertTrue(nr > 0, "downloaded image is not visible in image viewing app")
            Chrome.launch()
            self.driver.get('http://valeaplopului.com/chrome_tests/download_test_3.tmp')
            time.sleep(2)
            StatusBar.open_notifications()
            self.assertTrue(d(textStartsWith="download_test_3").wait.exists(timeout=3000))
            d(textStartsWith="download_test_3").click()
            self.assertTrue(d(packageName=DOWNLOADS_PACKAGE_NAME).wait.exists(timeout=7000))
        finally:
            Downloads.delete_downloaded_files()

    def test_clear_all_downloads_before_finished(self):
        # make sure there are no files in the download manager
        Downloads.delete_downloaded_files()
        self.download_file('http://valeaplopului.com/chrome_tests/download_test_5.tmp')
        time.sleep(2)
        self.download_file('http://valeaplopului.com/chrome_tests/download_test_4.tmp')
        time.sleep(2)
        self.download_file('http://valeaplopului.com/chrome_tests/download_test_3.tmp')
        time.sleep(2)
        # check if files are all in the download manager
        Downloads.launch()
        download_info = Downloads.get_download_files_info()
        # all 3 files must be in the download manager
        self.assertTrue(len(download_info) >= 3)
        Downloads.delete_downloaded_files()
        d.press.home()
        Downloads.launch()
        download_info = Downloads.get_download_files_info()
        if download_info is not None:
            self.assertTrue(len(download_info) < 3)

    def test_most_visited_page_funct(self):
        wait_for_most_visited_tiles = 20
        # Clear history
        self.assertTrue(self.clear_browsing_data(), "could not clear browsing data")
        Chrome.return_to_main_screen()
        self.driver.get('http://reddit.com')
        Chrome.close_current_tab()
        Chrome.open_new_tab()
        self.driver.switch_to.window(self.driver.window_handles[0])
        self.driver.get('http://wikipedia.org')
        time.sleep(10)
        Chrome.open_new_tab()
        self.assertTrue(d(resourceId=CHROME_MOST_VISITED_TITLE_RESID).wait.exists(timeout=3000),
                        "could not find most visited tiles section")
        most_visited_tiles = None
        for i in range(wait_for_most_visited_tiles):
            time.sleep(10)
            most_visited_tiles = d(resourceId=CHROME_MOST_VISITED_TITLE_RESID)
            if most_visited_tiles.count >= 2:
                break
        self.assertTrue(most_visited_tiles.count >= 2)
        self.assertTrue('reddit' in most_visited_tiles[0].info['text'].lower(),
                        "could not find reddit in most visited tiles")
        self.assertTrue('wikipedia' in most_visited_tiles[1].info['text'].lower(),
                        "could not find reddit in most visited tiles")
        time.sleep(2)
        Chrome.close_current_tab()
        Chrome.close_current_tab()
        Chrome.open_new_tab()
        self.driver.switch_to.window(self.driver.window_handles[0])
        self.driver.get('http://wikipedia.org')
        time.sleep(10)
        Chrome.open_new_tab()
        self.assertTrue(d(resourceId=CHROME_MOST_VISITED_TITLE_RESID).wait.exists(timeout=3000),
                        "could not find most visited tiles section")
        most_visited_tiles = d(resourceId=CHROME_MOST_VISITED_TITLE_RESID)
        for i in range(wait_for_most_visited_tiles):
            time.sleep(10)
            most_visited_tiles = d(resourceId=CHROME_MOST_VISITED_TITLE_RESID)
            if 'wikipedia' in most_visited_tiles[0].info['text'].lower():
                break
            Chrome.close_current_tab()
            Chrome.close_current_tab()
            Chrome.open_new_tab()
            self.driver.switch_to.window(self.driver.window_handles[0])
            self.driver.get('http://wikipedia.org')
            time.sleep(1)
            Chrome.open_new_tab()
        self.assertTrue('wikipedia' in most_visited_tiles[0].info['text'].lower(),
                        "wikipedia did not change position in most visited tiles after another visit")
        time.sleep(1)
        Chrome.close_all_tabs_and_count(self.driver)

    def test_tba_test_enable_javascript(self):
        self.driver.get('https://www.webkit.org/perf/sunspider/sunspider.html')
        Chrome.toggle_javascript(enable=False)
        Chrome.return_to_main_screen()
        self.driver.get("https://www.webkit.org/perf/sunspider-1.0.2/sunspider-1.0.2/driver.html")
        element_present = self.wait_for_element(50, '//pre[@id="console"]')
        self.assertFalse(element_present)
        Chrome.toggle_javascript(enable=True)
        Chrome.return_to_main_screen()
        self.driver.get("https://www.webkit.org/perf/sunspider-1.0.2/sunspider-1.0.2/driver.html")
        self.wait_for_element(60, '//pre[@id="console"]')
        console = self.driver.find_element(By.XPATH, '//pre[@id="console"]')
        console_text = console.get_attribute('innerHTML')
        print console_text

    def test_share_file_while_downloading_in_progress(self):
        try:
            # make sure there are no files in the download manager
            Downloads.delete_downloaded_files()
            # download file and try to open it
            self.download_file('http://valeaplopului.com/chrome_tests/download_test_5.tmp')
            time.sleep(20)
            Downloads.launch()
            # try to open downloaded file
            if d(resourceId="android:id/title").wait.exists(timeout=2000):
                d(resourceId="android:id/title").click()
            # download another file
            self.download_file('http://valeaplopului.com/chrome_tests/download_test_4.tmp')
            time.sleep(2)
            # try to share it
            if d(resourceId="android:id/title").wait.exists(timeout=2000):
                d(resourceId="android:id/title").long_click()
            ActionMenu.push_long_click_share()
            self.assertTrue(d(text="Share via").wait.exists(timeout=3000),
                            "could not find share file option")
            d.press.back()
            # Cleanup
        finally:
            Downloads.delete_downloaded_files()

    def test_details_of_downloaded_file(self):
        # define the color of the image downloaded (it is a solid color image)
        test_image_color = (200, 101, 0)
        date_min_size = 3
        try:
            # download the image
            self.driver.get('http://valeaplopului.com/chrome_tests/img/single_color_image.jpg')
            self.assertTrue(d(className=CHROME_WEBKIT_CLASS).wait.exists(timeout=3000))
            d(className=CHROME_WEBKIT_CLASS).long_click()
            self.assertTrue(d(text="Save image").wait.exists(timeout=3000))
            d(text="Save image").click()
            time.sleep(2)
            Chrome.clear_chrome_popups()
            # check download details for different orientations
            OrientationChanger.change_orientation("n")
            time.sleep(2)
            Downloads.launch()
            donwload_info = Downloads.get_download_files_info()
            self.assertTrue("single_color_image" in donwload_info[0].name)
            self.assertTrue("184" in donwload_info[0].size)
            self.assertTrue("Unsuccessful" not in donwload_info[0].summary)
            self.assertTrue(":" in donwload_info[0].date and len(donwload_info[0].date) > date_min_size)
            # check that changing orientation preserves the information displayed
            OrientationChanger.change_orientation("l")
            time.sleep(2)
            donwload_info = Downloads.get_download_files_info()
            self.assertTrue("single_color_image" in donwload_info[0].name,
                            "image name not available in download details")
            self.assertTrue("184" in donwload_info[0].size, "image size not available in download details")
            self.assertTrue("Unsuccessful" not in donwload_info[0].summary,
                            "image summary not available in download details")
            self.assertTrue(":" in donwload_info[0].date and len(donwload_info[0].date) > date_min_size,
                            "image download date not available in download details")
            time.sleep(2)
            # the icon of the downloaded file must be a thumbnail of the downloaded image
            self.screen_shooter.take_screenshot()
            nr = self.screen_shooter.get_nr_of_pixels_of_color(test_image_color)
            # so there must be a rectangular image of this color somewhere on the screen
            self.assertTrue(nr > 0, "downloaded image preview not visible")
        finally:
            Downloads.delete_downloaded_files()

    def test_tab_switch_mode(self):
        # open 10 normal tabs
        for i in range(10):
            Chrome.open_new_tab()
        self.assertTrue(len(self.driver.window_handles) >= 10)
        # open 4 aditional incognito tabs
        for i in range(4):
            Chrome.open_new_incognito_tab()
            self.assertTrue(Chrome.is_incognito_tab_in_focus(self.screen_shooter))
        self.assertTrue(len(self.driver.window_handles) >= 14)
        incognito_tab_value = True
        for i in range(5):
            Chrome.tap_incognito_mode_switcher()
            current_is_incognito_tab = Chrome.is_incognito_tab_in_focus(self.screen_shooter)
            self.assertTrue(current_is_incognito_tab is not incognito_tab_value)
            incognito_tab_value = not incognito_tab_value
        # close the tabs
        Chrome.close_all_tabs_and_count(self.driver)

    def test_edit_user_defined_folder(self):
        bookmark_folder_name = "test_folder"
        modified_bookmark_folder_name = "test_folder_modified"
        self.driver.get('http://reddit.com')
        Chrome.add_bookmark(folder=bookmark_folder_name)
        time.sleep(2)
        Chrome.open_mobile_bookmarks()
        self.assertTrue(d(text=bookmark_folder_name).wait.exists(timeout=3000))
        Chrome.edit_bookmark_folder(bookmark_folder_name, modified_bookmark_folder_name)
        self.assertTrue(d(text=modified_bookmark_folder_name).wait.exists(timeout=3000))
        time.sleep(1)
        # delete created folder
        Chrome.delete_bookmark(modified_bookmark_folder_name)

    def test_edit_bookmark_portrait_landscape(self):
        test_bookmark_name = "test_bookmark"
        modified_bookmark_name = "modified_bookmark"
        self.driver.get('http://wikipedia.org')
        Chrome.add_bookmark(name=test_bookmark_name)
        # change orientation
        OrientationChanger.change_orientation('n')
        Chrome.open_mobile_bookmarks()
        self.assertTrue(d(text=test_bookmark_name).wait.exists(timeout=3000),
                        "could not find created bookmark")
        # d(text=test_bookmark_name).long_click()  # does not always work
        Chrome.edit_bookmark(test_bookmark_name, modified_bookmark_name)
        # change orientation
        OrientationChanger.change_orientation('l')
        self.assertTrue(d(textContains=modified_bookmark_name).wait.exists(timeout=3000),
                        "bookmark name was not modified")
        # d(textContains=modified_bookmark_name).long_click()
        Chrome.edit_bookmark(modified_bookmark_name, test_bookmark_name)
        self.assertTrue(d(textContains=test_bookmark_name).wait.exists(timeout=3000),
                        "modified bookmark was not found")
        # delete created bookmark
        Chrome.delete_bookmark(test_bookmark_name)
        time.sleep(10)
        self.assertFalse(d(textContains=test_bookmark_name).exists,
                         "modified bookmark could not be deleted")

    def test_check_history_after_incognito(self):
        Chrome.close_current_tab()
        Chrome.open_new_incognito_tab()
        self.driver.switch_to.window(self.driver.window_handles[0])
        self.driver.get('http://www.kappa.ro')
        self.driver.get('chrome://history')
        found = False
        missing = False
        # list of elements in history should have 0 items
        history_list = self.driver.find_elements(By.XPATH, '//div[@class="title"]')
        if len(history_list) == 1:
                missing = True
        print missing
        # is list is empty, then proceed with checking history without incognito
        if missing:
            Chrome.open_new_tab()
            self.driver.switch_to.window(self.driver.window_handles[1])
            self.driver.get('http://www.emag.ro')
            self.driver.get('chrome://history')
            history_list = self.driver.find_elements(By.XPATH, '//div[@class="title"]')
            if len(history_list) == 2:
                found = True
        print found
        self.assertTrue(missing and found)

    def test_new_incognito_tab(self):
        # open a normal tab
        self.driver.get('http://reddit.com')
        # open incognito tab and check if it is dark
        Chrome.open_new_incognito_tab()
        self.assertTrue(Chrome.is_incognito_tab_in_focus(self.screen_shooter))
        # check some incognito tab specific views
        time.sleep(3)
        self.assertTrue(d(textContains="incognito").wait.exists(timeout=3000))
        UiAutomatorUtils.refresh_uiautomator_view_hierarchy()
        self.assertTrue(d(text=CHROME_INCOGNITO_LEARN_MORE_TXT).wait.exists(timeout=3000))
        UiAutomatorUtils.refresh_uiautomator_view_hierarchy()
        # switch to normal tab and check the switch with the url value
        incognito_tab_url = Chrome.get_current_tab_url()
        Chrome.tap_incognito_mode_switcher()
        current_tab_url = Chrome.get_current_tab_url()
        self.assertTrue(incognito_tab_url != current_tab_url)

    def test_bookmark_long_name_and_url(self):
        long_url = "http://valeaplopului.com/chrome_tests/very_long_url_that_will_help_test_the_chrome_browser" + \
                   "_bookmark_manager_this_is_it_and_a_bottle_of_rum.html"
        long_name = "This_is_a_very_long_bookmark_name_that_shall_be_used_to_store_the_very_long_url_above_in_the_" + \
                    "chrome_browser_bookmark_manager"
        self.driver.get(long_url)
        time.sleep(1)
        Chrome.add_bookmark(long_name)
        Chrome.open_mobile_bookmarks()
        self.assertTrue(d(text=long_name).wait.exists(timeout=3000))
        d(text=long_name).click()
        time.sleep(2)
        url = Chrome.get_current_tab_url()
        self.assertTrue(url in long_url)

    def test_download_when_dut_locked(self):
        download_file_url = "http://valeaplopului.com/chrome_tests/download_test_5.tmp"
        # make sure screen lock is enabled
        Settings.enable_screen_lock("Swipe")
        d.press.home()
        try:
            Downloads.launch()
            self.download_file(download_file_url)
            time.sleep(2)
            download_info = Downloads.get_download_files_info()
            initial_status = download_info[0].summary
            # lock the DUT
            d.press.power()
            time.sleep(4)
            d.press.power()
            # let the download progress while screen is locked
            time.sleep(10)
            UiAutomatorUtils.unlock_screen()
            Downloads.launch()
            download_info = Downloads.get_download_files_info()
            final_status = download_info[0].summary
            print final_status, initial_status
            self.assertTrue(final_status != initial_status)
            self.assertTrue("Unsuccessful" not in final_status)
        finally:
            Downloads.delete_downloaded_files()
            Settings.enable_screen_lock("None")

    def test_download_when_dut_screen_sleeping(self):
        download_file_url = "http://valeaplopului.com/chrome_tests/download_test_2.tmp"
        Downloads.delete_downloaded_files()
        try:
            self.download_file(download_file_url)
            time.sleep(3)
            Downloads.launch()
            download_info = Downloads.get_download_files_info()
            initial_status = download_info[0].summary
            # lock the DUT
            d.press.power()
            # let the download progress while screen is locked
            time.sleep(10)
            UiAutomatorUtils.unlock_screen()
            UiAutomatorUtils.clear_obscuring_element()
            Downloads.launch()
            download_info = Downloads.get_download_files_info()
            final_status = download_info[0].summary
            print final_status, initial_status
            self.assertTrue(final_status != initial_status)
            self.assertTrue("Unsuccessful" not in final_status)
        finally:
            Downloads.delete_downloaded_files()

    def test_disconnect_reconnect_wifi_while_downloadinng(self):
        download_file_url_1 = "http://valeaplopului.com/chrome_tests/download_test_2.tmp"
        try:
            self.download_file(download_file_url_1)
            Settings.disable_wifi()
            Settings.enable_wifi()
            time.sleep(7)
            Downloads.launch()
            self.assertTrue(d(resourceId=DOWNLOADS_FILE_TITLE_RESID).wait.exists(timeout=3000),
                            "could not find file that was being downloaded")
            d(text="Unsuccessful").wait.exists(timeout=30000)
            d(resourceId=DOWNLOADS_FILE_TITLE_RESID)[0].click()
            if d(text="Retry").wait.exists(timeout=3000):
                d(text="Retry").click()
                time.sleep(2)
                download_info = Downloads.get_download_files_info()
                self.assertTrue("Unsuccessful" not in download_info[0].summary)
                time.sleep(2)
            elif d(text="Queued").wait.exists(timeout=30000):
                    download_info = Downloads.get_download_files_info()
                    self.assertTrue("Queued" not in download_info[0].summary)
        finally:
            Downloads.delete_downloaded_files()

    def test_autofill_forms(self):
        account_creation_link = "https://accounts.google.com/SignUp?continue=https%3A%2F%2Fwww.google.co.jp%2F%3Fgfe" \
                                "_rd%3Dcr%26ei%3Dqch2VeUL7_3zB62GitAN&hl=en"
        Chrome.enter_autofill_forms_info()
        self.driver.get(account_creation_link)
        self.driver.find_element(By.XPATH, '//input[@name="RecoveryEmailAddress"]').click()
        time.sleep(2)
        # select the new autofill data to fill the form
        d.press.up()
        time.sleep(1)
        d.press.up()
        time.sleep(1)
        d.press.up()
        time.sleep(1)
        d.press.right()
        d.press.right()
        time.sleep(1)
        d.press(KEYCODE_SPACE)
        time.sleep(1)
        autofilled_email = self.driver.execute_script('return document.getElementById("RecoveryEmailAddress").value;')
        print autofilled_email
        self.assertTrue("test@test.com" in autofilled_email)
        time.sleep(2)

    def test_download_same_name_files(self):
        try:
            download_file_url = "http://valeaplopului.com/chrome_tests/download_test_5.tmp"
            download_file_name = "download_test_5"
            self.download_file(download_file_url)
            time.sleep(1)
            self.download_file(download_file_url, create_new_file=True)
            time.sleep(5)
            ls_downloads = "ls /sdcard/Download"
            download_folder_ls = AdbUtils.run_adb_cmd(ls_downloads)
            LOG.info("download folder contents: " + download_folder_ls)
            nr_of_occurences = download_folder_ls.count(download_file_name)
            self.assertTrue(nr_of_occurences > 1, "Could not find duplicate file names")
        finally:
            Downloads.delete_downloaded_files()

    def test_check_home_button(self):
        Chrome.launch()
        # try to get a tab to make sure chrome is in the foreground
        Chrome.verify_open_tab_or_make_new_tab()
        d.press.home()
        time.sleep(2)
        self.assertTrue(not d(packageName=CHROME_PACKAGE_NAME).exists)

    def test_tba_enter_url_address(self):
        self.driver.get('http://google.com')
        if not d(resourceId=CHROME_ADDRESS_BAR_RESID).wait.exists(timeout=3000):
            ScreenSwiper.swipe_down()
        self.assertTrue("google" in d(resourceId=CHROME_ADDRESS_BAR_RESID).info["text"])

    def test_tba_navigation(self):
        self.driver.get('http://wikipedia.org')
        self.driver.get('http://google.com')
        Chrome.navigation_back()
        time.sleep(5)
        self.assertTrue("wikipedia" in self.driver.current_url)
        Chrome.navigation_fwd()
        time.sleep(5)
        self.assertTrue("google" in self.driver.current_url)
        Chrome.refresh_page()
        time.sleep(5)
        self.assertTrue("google" in self.driver.current_url)

    def test_tba_print_option(self):
        self.driver.get('http://google.com')
        try:
            Chrome.open_chrome_menu()
            d(textStartsWith=CHROME_PRINT_OPTION).wait.exists(timeout=3000)
            d(textStartsWith=CHROME_PRINT_OPTION).click()
            d(packageName=PRINT_SPOOLER_PACKAGE_NAME).wait.exists(timeout=7000)
        finally:
            Chrome.return_to_main_screen()

    def test_tba_search_google_for_image(self):
        self.driver.get('https://upload.wikimedia.org/wikipedia/commons/d/da/The_Parthenon_in_Athens.jpg')
        width, height = UiAutomatorUtils.get_screen_dims()
        d.long_click(width/2, height/2)
        searched = False
        if d(text="Search Google for this image").wait.exists(timeout=3000):
            d(text="Search Google for this image").click()
            searched = True
        elif d(text="Web search").exists:
            d(text="Web search").click()
            searched = True
        self.assertTrue(searched, "was not able to search for image")

    def test_check_find_in_page(self):
        self.driver.get("http://ro.wikipedia.org/wiki/B%C4%83icoi")
        find_status = Chrome.find_text_in_current_page("este")
        self.assertTrue("0/0" not in find_status)

    def test_full_screen_browsing(self):
        self.driver.get('http://reddit.com')
        ScreenSwiper.swipe_up()
        ScreenSwiper.swipe_up()
        self.assertTrue(not d(resourceId=CHROME_ADDRESS_BAR_RESID).exists, "fullscreen browsing was not activated")

    def test_help_and_feedback(self):
        self.driver.get('http://google.com')
        Chrome.open_chrome_menu()
        d(textStartsWith=CHROME_HELP_TXT).wait.exists(timeout=3000)
        d(textStartsWith=CHROME_HELP_TXT).click()
        help_and_feedback_test = False
        if d(textContains=CHROME_HELP_TXT).wait.exists(timeout=3000) and \
                (d(textContains=CHROME_FEEDBACK_TXT).wait.exists(timeout=1000) or
                 d(textContains=CHROME_FEEDBACK_TXT.lower()).wait.exists(timeout=1000)):
            help_and_feedback_test = True
        self.assertTrue(help_and_feedback_test, "could not find help and feedback section")

    def test_launch_portrait_landscape(self):
        test_urls = ["http://wikipedia.org", "http://reddit.com", "http://w3schools.com"]
        OrientationChanger.change_orientation("n")
        for url in test_urls:
            self.driver.get(url)
        OrientationChanger.change_orientation("r")
        for url in test_urls:
            self.driver.get(url)

    def test_remove_a_page_from_history(self):
        self.driver.get('http://google.com')
        self.driver.get('http://www.bing.com/')
        self.driver.get('chrome://history/')
        time.sleep(2)
        visited_pages = self.driver.find_elements(By.XPATH, '//div[@class="title"]')
        initial_visited_pages = len(visited_pages)
        print "initial nr. of pages: ", initial_visited_pages
        time.sleep(2)
        remove_handle = self.driver.find_element(By.XPATH, '//button[@class="remove-entry custom-appearance"]')
        remove_handle.click()
        time.sleep(2)
        visited_pages = self.driver.find_elements(By.XPATH, '//div[@class="title"]')
        final_visited_pages = len(visited_pages)
        print "final nr. of pages: ", final_visited_pages
        self.assertTrue(final_visited_pages < initial_visited_pages, "page was not removed from history")

    def click_middle_of_colored_rectangle(self, color):
        self.screen_shooter.take_screenshot()
        input_text_pixels = self.screen_shooter\
            .get_all_pixels_of_color(color, color_error=20)
        aprox_middle_pixel = input_text_pixels[len(input_text_pixels) / 2]
        input_text_coord_x = aprox_middle_pixel.x
        input_text_coord_y = aprox_middle_pixel.y
        LOG.info("found box at: %s %s" % (input_text_coord_x, input_text_coord_y))
        d.click(input_text_coord_x, input_text_coord_y)

    def test_pinyin_available_in_browser(self):
        keyboard_changer = KeyboardChanger()
        try:
            keyboard_changer.change_keyboard(PINYIN_KEYBOARD_NAME)
            Chrome.launch()
            self.driver.get("http://valeaplopului.com/chrome_tests/input_text_test.html")
            time.sleep(3)
            self.driver.find_element(By.XPATH, '//input[@id="input_text"]')
            # click an input text box to get the keyboards option
            self.click_middle_of_colored_rectangle(CHROME_INPUT_TEXT_BACKGROUND_COLOR)
            time.sleep(5)  # wait a while for the keyboard to pop-up
            self.screen_shooter.take_screenshot()
            keyboard_icon_x, keyboard_icon_y = self.screen_shooter.\
                get_first_pixel_of_color_from_ref_with_direction(CHROME_KEYBOARD_SWITCH_ICON_LOWER_BAR_COLOR,
                                                                 (self.screen_shooter.screen_width - 3,
                                                                  self.screen_shooter.screen_height - 3),
                                                                 (-1, -1), color_error=30)
            LOG.info("found keyboard icon at: %s %s" % (keyboard_icon_x, keyboard_icon_y))
            AdbUtils.tap(keyboard_icon_x, keyboard_icon_y)
            self.assertTrue(d(text=PINYIN_KEYBOARD_NAME).wait.exists(timeout=5000), "pinyin IME not found")
            self.assertTrue(d(text=GOOGLE_KEYBOARD_NAME).wait.exists(timeout=1000), "latin IME not found")
        finally:
            keyboard_changer.change_keyboard(GOOGLE_KEYBOARD_NAME)

    def test_japanese_ime_search(self):
        keyboard_changer = KeyboardChanger()
        try:
            keyboard_changer.change_keyboard(JAPANESE_KEYBOARD_NAME)
            d.press.home()
            time.sleep(2)
            ktopx, ktopy, kwidth, kheight = KeyboardGeometry.get_keyboard_geometry()
            japanese_keyboard = JapaneseKeyboard("Japan_IM", int(ktopx), int(ktopy), int(kwidth), int(kheight))
            LOG.info("got keyboard geometry: " + ktopx + " " + ktopy + " " + kwidth + " " + kheight)
            start_search_url = "http://valeaplopului.com/chrome_tests/google_search.html"
            self.driver.get(start_search_url)
            # click an input text box to get the keyboards option
            Chrome.launch()
            time.sleep(2)
            self.click_middle_of_colored_rectangle(CHROME_INPUT_TEXT_BACKGROUND_COLOR)
            time.sleep(5)  # wait a while for the keyboard to pop-up
            japanese_keyboard.press_key("ka")
            japanese_keyboard.press_key("ta")
            japanese_keyboard.press_key("ka")
            japanese_keyboard.press_key("na")
            time.sleep(2)
            self.driver.find_element(By.XPATH, '//button[@id="searchButton"]').click()
            time.sleep(2)  # wait for page to load
            page_source = self.driver.page_source
            self.assertTrue("か" in page_source, "could not find the か kana")
            self.assertTrue("た" in page_source, "could not find the た kana")
            self.assertTrue("な" in page_source, "could not find the な kana")
        finally:
            keyboard_changer.change_keyboard(GOOGLE_KEYBOARD_NAME)

    def test_add_bookmarks_widget_to_homescreen(self):
        Chrome.delete_current_bookmarks()
        test_urls = ["http://wikipedia.org", "http://reddit.com", "http://w3schools.com"]
        for url in test_urls:
            self.driver.get(url)
            time.sleep(1)
            Chrome.bookmark_current_page()
        WidgetsUtils.launch_widget_chooser()
        WidgetsUtils.scroll_to_widget(CHROME_BOOKMARK_WIDGET_NAME)
        time.sleep(2)
        self.assertTrue(d(textContains=CHROME_BOOKMARK_WIDGET_NAME).wait.exists(timeout=2000))
        UiAutomatorUtils.long_click(d(textContains=CHROME_BOOKMARK_WIDGET_NAME), 3)
        d.press.home()
        bookmarks_reference_titles = ["chools", "reddit", "ikipedia"]  # some bookmarks may start with CAPS
        for title in bookmarks_reference_titles:
            self.assertTrue(d(textContains=title).wait.exists(timeout=1000),
                            "bookmark must be visible in the homescreen widget")

    def test_map_location(self):
        Settings.enable_location()
        map_url = "http://valeaplopului.com/chrome_tests/google_map.html"
        wiki_url = 'http://wikipedia.org'
        Chrome.launch()
        self.driver.get(map_url)
        self.assertTrue(d(textContains=NOTIFICATIONS_USE_DEVICE_LOCATION_TXT).wait.exists(timeout=5000))
        self.assertTrue(d(textContains=NOTIFICATIONS_ALLOW_LOCATION_TXT).wait.exists(timeout=5000))
        self.assertTrue(d(textContains=NOTIFICATIONS_BLOCK_LOCATION_TXT).wait.exists(timeout=5000))
        self.driver.get(wiki_url)
        time.sleep(5)
        self.assertFalse(d(textContains=NOTIFICATIONS_USE_DEVICE_LOCATION_TXT).wait.exists(timeout=5000))
        self.driver.get(map_url)
        self.assertTrue(d(textContains=NOTIFICATIONS_USE_DEVICE_LOCATION_TXT).wait.exists(timeout=5000))
        d(text=NOTIFICATIONS_ALLOW_LOCATION_TXT).click()
        for i in range(20):
            time.sleep(1)
            long_element = self.driver.find_element(By.XPATH, '//div[@id="long"]')
            long_value = long_element.get_attribute('innerHTML')
            if len(long_value) > 0:
                break
        lat_element = self.driver.find_element(By.XPATH, '//div[@id="lat"]')
        lat_value = lat_element.get_attribute('innerHTML')
        long_element = self.driver.find_element(By.XPATH, '//div[@id="long"]')
        long_value = long_element.get_attribute('innerHTML')
        self.assertTrue(-180 <= float(long_value) <= 180)
        self.assertTrue(0 <= float(lat_value) <= 90)

    url_list = []

    def browse_pages(self):
        for url in ChromeTests.url_list:
            self.driver.get(url)
            time.sleep(3)


class ChromeSupportTests(ChromeTests):

    def setUp(self):
        UiAutomatorUtils.unlock_screen()
        Chrome.launch()
        ChromeTests.use_running_app = True
        super(ChromeSupportTests, self).setUp()

    def start_webdriver(self):
        print "new start web driver"
        driver_container = []
        driver = None

        def start_chrome():
            chrome_launched = 0
            for i in range(10):
                if chrome_launched == 2:
                    break
                time.sleep(1)
                if d(packageName=CHROME_PACKAGE_NAME, className=CHROME_FRAME_LAYOUT_CLASS).exists:
                    pass
                else:
                    Chrome.launch()
                    chrome_launched += 1

        def fire_up_webdriver(local_host, capabilities):
            driver_container.append(webdriver.Remote(local_host, capabilities))

        for i in range(40):
            time.sleep(2)
            try:
                default_capabilities["chromeOptions"]["androidDeviceSerial"]\
                    = dut_manager.active_uiautomator_device_serial
                if ChromeTests.use_running_app:
                    default_capabilities["chromeOptions"]["androidUseRunningApp"] = True
                    print str(default_capabilities)
                # driver = webdriver.Remote('http://localhost:9515', default_capabilities)
                webdriver_thread = Thread(target=fire_up_webdriver, args=('http://localhost:9515', default_capabilities,))
                chrome_thread = Thread(target=start_chrome)
                webdriver_thread.start()
                chrome_thread.start()
                chrome_thread.join()
                webdriver_thread.join()
                driver = driver_container.pop()
                break
            except WebDriverException as e:
                LOG.info("failed to start chromedriver, try: %s" % str(i))
        return driver

    def test_add_bookmark(self):
        test_link = "http://en.wikipedia.org/wiki/Selenium"
        test_bookmark_name = "Selenium - Wikipedia, the free encyclopedia"
        test_url_reference_string = "wikipedia.org"
        self.driver.get(test_link)
        Chrome.add_bookmark(name=test_bookmark_name)
        Chrome.open_mobile_bookmarks()
        self.assertTrue(d(text=test_bookmark_name).wait.exists(timeout=3000))
        d(text=test_bookmark_name).click()
        time.sleep(2)
        self.assertTrue(test_url_reference_string in self.driver.current_url)

    def test_page_loaded(self):
        test_bookmark_name = "Selenium - Wikipedia, the free encyclopedia"
        Chrome.open_mobile_bookmarks()
        self.assertTrue(d(text=test_bookmark_name).wait.exists(timeout=3000))
        d(text=test_bookmark_name).click()
        time.sleep(2)
        self.driver.implicitly_wait(10)
        self.assertTrue("This article is about the chemical element.  For the software testing framework, see"
                        in self.driver.page_source)


if __name__ == "__main__":
    test_result = SingleMethodRunner.run_single_test(ChromeTests, "test_check_cookies")
