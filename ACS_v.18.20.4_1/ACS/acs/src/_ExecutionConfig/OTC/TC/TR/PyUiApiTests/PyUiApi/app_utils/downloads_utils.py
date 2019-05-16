from PyUiApi.common.uiautomator_utils import *

adb_clear_download_dir = "rm -rf /sdcard/Download/*"


class DownloadsM(object):
    @staticmethod
    def launch():
        if d(packageName=DOWNLOADS_PACKAGE_NAME, className=DOWNLOADS_FRAME_LAYOUT_CLASS).exists:
            pass
        else:
            UiAutomatorUtils.launch_app_from_apps_menu(DOWNLOADS_SHORTCUT_NAME)

    @staticmethod
    def get_download_files_info():
        DownloadsM.launch()
        if d(resourceId=DOWNLOADS_FILE_TITLE_RESID).wait.exists(timeout=3000):
            # ensure that we are in the list view mode
            if d(resourceId=DOWNLOADS_FILE_SUMMARY_RESID).wait.exists(timeout=3000) or\
               d(description=DOWNLOADS_GRID_VIEW_DESC).wait.exists(timeout=1000):
                    pass
            elif d(description=DOWNLOADS_LIST_VIEW_DESC).wait.exists(timeout=2000):
                d(description=DOWNLOADS_LIST_VIEW_DESC).click()
            elif d(description=DOWNLOADS_MORE_OPTIONS_DESC).wait.exists(timeout=2000):
                d(description=DOWNLOADS_MORE_OPTIONS_DESC).click()
                if d(text=DOWNLOADS_LIST_VIEW_DESC).wait.exists(timeout=3000):
                    d(text=DOWNLOADS_LIST_VIEW_DESC).click()
            nr_of_files = d(resourceId=DOWNLOADS_FILE_TITLE_RESID).count
            if nr_of_files > 1:
                download_files_info = []
                for i in range(nr_of_files):
                    file_name = Info.get_text(d(resourceId=DOWNLOADS_FILE_TITLE_RESID)[i])
                    file_size = Info.get_text(d(resourceId=DOWNLOADS_FILE_SIZE_RESID)[i])
                    file_summary = Info.get_text(d(resourceId=DOWNLOADS_FILE_SUMMARY_RESID)[i])
                    file_date = Info.get_text(d(resourceId=DOWNLOADS_FILE_DATE_RESID)[i])
                    download_files_info.append(FileDownloadInfo(file_name, size=file_size, summary=file_summary,
                                                                date=file_date))
                return download_files_info
            elif nr_of_files == 1:
                file_name = Info.get_text(d(resourceId=DOWNLOADS_FILE_TITLE_RESID))
                file_size = Info.get_text(d(resourceId=DOWNLOADS_FILE_SIZE_RESID))
                file_summary = Info.get_text(d(resourceId=DOWNLOADS_FILE_SUMMARY_RESID))
                file_date = Info.get_text(d(resourceId=DOWNLOADS_FILE_DATE_RESID))
                return [FileDownloadInfo(file_name, size=file_size, summary=file_summary, date=file_date)]
            else:
                return None

    @staticmethod
    def sort_downloads(criterion):
        if d(description=DOWNLOADS_SORT_OPTION_DESC).wait.exists(timeout=3000):
            d(description=DOWNLOADS_SORT_OPTION_DESC).click()
            if d(text=criterion).wait.exists(timeout=3000):
                d(text=criterion).click()
        time.sleep(2)

    @staticmethod
    def delete_downloaded_files():
        if not d(packageName="com.android.documentsui").wait.exists(timeout=2000):
            DownloadsM.launch()
        try:
            if d(resourceId="android:id/title").wait.exists(timeout=2000):
                files = d(resourceId="android:id/title")
                nr_of_files = len(files)
                if nr_of_files > 0:
                    files[0].long_click()
                    time.sleep(1)
                    for i in range(nr_of_files - 1):
                        files[i + 1].click()
                ActionMenu.delete()

        except:
            print "error deleting downloaded files from UI"
            LOG.info("error deleting downloaded files from UI")
        AdbUtils.run_adb_cmd(adb_clear_download_dir)


class DownloadsN(DownloadsM):
    @staticmethod
    def launch():
        DownloadsM.launch()
        if d(text=DOWNLOADS_RETRY_TXT).wait.exists(timeout=2000):
            d(text=DOWNLOADS_RETRY_TXT).click()
        time.sleep(2)
        if not d(description=DOWNLOADS_SHOW_ROOTS_DESC).wait.exists(timeout=3000):
            d.press.back()
        return DownloadsN.ensure_downloads_view()

    @staticmethod
    def ensure_downloads_view():
        if d(description=DOWNLOADS_SHOW_ROOTS_DESC).wait.exists(timeout=3000):
            view = d(description=DOWNLOADS_SHOW_ROOTS_DESC).right(className=ANDROID_WIDGET_TEXT_VIEW)
            if Info.get_text(view) == DOWNLOADS_VIEW_TXT:
                return True
            else:
                d(description=DOWNLOADS_SHOW_ROOTS_DESC).click()
                if d(resourceId=TITLE_ELEMENT_RESID, text=DOWNLOADS_VIEW_TXT).wait.exists(timeout=3000):
                    d(resourceId=TITLE_ELEMENT_RESID, text=DOWNLOADS_VIEW_TXT).click()
                    return True
        return False

    @staticmethod
    def delete_downloaded_files():
        DownloadsN.launch()
        DownloadsM.delete_downloaded_files()
        if d(text=POPUP_OK).wait.exists(timeout=3000):
            d(text=POPUP_OK).click()

    @staticmethod
    def get_download_files_info():
        DownloadsN.launch()
        DownloadsN.prepare_downloads_layout()
        DownloadsN.ensure_downloads_size_shown()
        return DownloadsN.retrieve_downloads_info()

    @staticmethod
    def ensure_downloads_size_shown():
        if d(resourceId=DOWNLOADS_FILE_SIZE_RESID).wait.exists(timeout=3000):
            return
        else:
            d.press.menu()
            if d(text=DOWNLOADS_SHOW_FILE_SIZE_TXT).wait.exists(timeout=3000):
                d(text=DOWNLOADS_SHOW_FILE_SIZE_TXT).click()
            else:
                d.press.back()

    @staticmethod
    def prepare_downloads_layout():
        if d(resourceId=DOWNLOADS_FILE_TITLE_RESID).wait.exists(timeout=3000):
            # ensure that we are in the list view mode
            if d(resourceId=DOWNLOADS_FILE_SUMMARY_RESID).wait.exists(timeout=3000) or\
               d(description=DOWNLOADS_GRID_VIEW_DESC).wait.exists(timeout=1000):
                    pass
            elif d(description=DOWNLOADS_LIST_VIEW_DESC).wait.exists(timeout=2000):
                d(description=DOWNLOADS_LIST_VIEW_DESC).click()
            elif d(description=DOWNLOADS_MORE_OPTIONS_DESC).wait.exists(timeout=2000):
                d(description=DOWNLOADS_MORE_OPTIONS_DESC).click()
                if d(text=DOWNLOADS_LIST_VIEW_DESC).wait.exists(timeout=3000):
                    d(text=DOWNLOADS_LIST_VIEW_DESC).click()

    @staticmethod
    def retrieve_downloads_info():
        nr_of_files = d(resourceId=DOWNLOADS_FILE_TITLE_RESID).count
        if nr_of_files > 1:
            download_files_info = []
            for i in range(nr_of_files):
                file_name = Info.get_text(d(resourceId=DOWNLOADS_FILE_TITLE_RESID)[i])
                file_size = Info.get_text(d(resourceId=DOWNLOADS_FILE_SIZE_RESID)[i])
                file_summary = Info.get_text(d(resourceId=DOWNLOADS_FILE_SUMMARY_RESID)[i])
                file_date = Info.get_text(d(resourceId=DOWNLOADS_FILE_DATE_RESID)[i])
                download_files_info.append(FileDownloadInfo(file_name, size=file_size, summary=file_summary,
                                                            date=file_date))
            return download_files_info
        elif nr_of_files == 1:
            file_name = Info.get_text(d(resourceId=DOWNLOADS_FILE_TITLE_RESID))
            file_size = Info.get_text(d(resourceId=DOWNLOADS_FILE_SIZE_RESID))
            file_summary = Info.get_text(d(resourceId=DOWNLOADS_FILE_SUMMARY_RESID))
            file_date = Info.get_text(d(resourceId=DOWNLOADS_FILE_DATE_RESID))
            return [FileDownloadInfo(file_name, size=file_size, summary=file_summary, date=file_date)]
        else:
            return None


class FileDownloadInfo(object):
    name = None
    size = None
    date = None
    summary = None

    def __init__(self, name, size=None, summary=None, date=None):
        self.name = name
        self.size = size
        self.date = date
        self.summary = summary

if ANDROID_VERSION is "M":
    Downloads = DownloadsM
elif ANDROID_VERSION is "N":
    Downloads = DownloadsN
else:
    Downloads = DownloadsM
