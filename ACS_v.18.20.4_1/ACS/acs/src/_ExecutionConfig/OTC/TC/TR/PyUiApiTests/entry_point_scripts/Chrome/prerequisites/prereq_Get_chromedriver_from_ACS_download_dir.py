from _prerequisites import *
from PyUiApi.common.shell_utils import *
import sys

framework_path = None

for p in sys.path:
    if "PyUiApi" in p:
        framework_path = p

acs_download_dir = os.path.join(os.path.expanduser("~/"), ".acs")
chromedriver_pattern = "*chromedriver*"

chrome_utils_file_pattern = "chrome_utils*"

chromedriver_download_path = ShellUtils.find_files_matching_pattern(acs_download_dir, chromedriver_pattern)
print chromedriver_download_path

chrome_utils_path = ShellUtils.find_files_matching_pattern(framework_path, chrome_utils_file_pattern)
print chrome_utils_path

shutil.copyfile(chromedriver_download_path, os.path.join(os.path.dirname(chrome_utils_path), "chromedriver"))

legacy_chrome_path = ShellUtils.find_files_matching_pattern(os.path.join(framework_path, ".."),
                                                            chrome_utils_file_pattern)
print legacy_chrome_path
shutil.copyfile(chromedriver_download_path, os.path.join(os.path.dirname(legacy_chrome_path), "chromedriver"))

print "PASS"
