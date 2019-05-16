from PyUiApi.multi_dut_support.dut_manager import *

# share some important variables across modules
import __builtin__
if not hasattr(__builtin__, 'dut_manager'):
    __builtin__.dut_manager = DutManager()
if not hasattr(__builtin__, 'd'):
    __builtin__.d = __builtin__.dut_manager.active_uiautomator_device

