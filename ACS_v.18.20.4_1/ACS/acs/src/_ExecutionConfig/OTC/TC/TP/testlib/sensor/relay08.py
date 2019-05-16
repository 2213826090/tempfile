import serial
import time
import sys
import os

NOSERUNNER = "NOSERUNNER"

def set_relay_status(relay, num_str, status):
    #assert os.path.exists(relay), "Relay not exists!"
    if not num_str:
        #open the serial
        ser = serial.Serial(relay, 9600)
        if status == 0:
            ser.write('\xFE\x05\x00\x00\xFF\x00\x98\x35')
        else:
            ser.write('\xFE\x05\x00\x00\x00\x00\xD9\xC5')
        time.sleep(1)
        #close the device
        ser.close()
    else:
        if 0 == status:
            status = 100
        else:
            status = 110
        ser = None
        for n in range(10):
            try:
                ser = serial.Serial(port = relay, baudrate = 19200, stopbits = 2)
                break
            except serial.serialutil.SerialException:
                time.sleep(5)
        if ser:
            for i in num_str:
                ser.write(chr(int(i) + status))
                time.sleep(0.001)
            ser.close()
        else:
            assert False, "Could not open port %s" % relay

# compatible with relay08 and old relay
def set_relay_status_compatible(relay, status):
    relay_list = relay.split(':')
    if len(relay_list) == 1:
        set_relay_status(relay_list[0], None, status)
    else:
        set_relay_status(relay_list[0], relay_list[1], status)

# define class to operate relay
class NomalRelay(object):

    def __init__(self, relay_usb, relay_power):
        self.relay_usb = relay_usb
        self.relay_power = relay_power

    def usb_connect(self):
        print "[info]--- connect usb:", self.relay_usb
        set_relay_status_compatible(self.relay_usb, 1)

    def usb_disconnect(self):
        print "[info]--- disconnect usb:", self.relay_usb
        set_relay_status_compatible(self.relay_usb, 0)

    def press_power_key(self, duration=4):
        print "[info]--- press power key:", self.relay_power
        set_relay_status_compatible(self.relay_power, 0)
        time.sleep(duration)
        set_relay_status_compatible(self.relay_power, 1)

# define class to operate relay by ACS
class NomalRelay08ACS(object):

    def __init__(self, relay08):
        self.relay08 = relay08
        if self.relay08.get_bench_params().get_param_value("SwitchOnOff"):
            self.power = self.relay08.get_bench_params().get_param_value("SwitchOnOff")
        else:
            self.power = None
        #self.usb = self.relay08.get_bench_params().get_param_value("RelayCutter")

    def usb_connect(self):
        self.relay08.disable_line(self.usb)

    def usb_disconnect(self):
        self.relay08.enable_line(self.usb)

    def press_power_key(self, duration = 4):
        self.relay08.enable_line(self.power)
        time.sleep(duration)
        self.relay08.disable_line(self.power)

def get_relay_obj():
    if os.environ.get(NOSERUNNER) is None:
        from acs_test_scripts.Equipment.EquipmentManager import EquipmentManager
        io_card = EquipmentManager().get_io_cards()["IO_CARD"]
        return NomalRelay08ACS(io_card)
    else:
        from testlib.common.map import DeviceMap
        mapObj = DeviceMap()
        usb_relay = mapObj.getValue("cutter_relay")
        power_relay = mapObj.getValue("power_relay")
        return NomalRelay(usb_relay, power_relay)

# define 3-way-cutter class
class ThreeWayCutter(object):

    def __init__(self, three_way_cutter):
        self.three_way_cutter = three_way_cutter
        self.cutter = "1" # set 0 to disconnect
        self.cdp = "567" # set 0 to connect
        self.dcp = "234" # set 0 to connect

    def set_usb_line_status(self, status):
        set_relay_status(self.three_way_cutter, self.cutter, status)

    def enable_sdp_charging(self, switching = 2, post_switch = 2):
        set_relay_status(self.three_way_cutter, self.cutter, 0)
        time.sleep(switching)
        set_relay_status(self.three_way_cutter, self.cutter + self.dcp + self.cdp, 1)
        time.sleep(post_switch)

    def enable_cdp_charging(self, switching = 2, post_switch = 2):
        set_relay_status(self.three_way_cutter, self.cutter + self.cdp, 0)
        time.sleep(switching)
        set_relay_status(self.three_way_cutter, self.cutter + self.dcp, 1)
        time.sleep(post_switch)

    def enable_dcp_charging(self, switching = 2, post_switch = 2):
        set_relay_status(self.three_way_cutter, self.cutter + self.dcp, 0)
        time.sleep(switching)
        set_relay_status(self.three_way_cutter, self.cutter + self.cdp, 1)
        time.sleep(post_switch)

def get_three_way_cutter():
    if os.environ.get(NOSERUNNER):
        from testlib.common.map import DeviceMap
        mapObj = DeviceMap()
        three_way_cutter = mapObj.getValue("three_way_cutter")
        print "[info]---get three way cutter by map.conf:", three_way_cutter
    else:
        from acs_test_scripts.Equipment.EquipmentManager import EquipmentManager
        io_card = EquipmentManager().get_io_cards()["IO_CARD"]
        three_way_cutter = io_card.get_bench_params().get_param_value("ThreeChargingModeSwitchCutter")
        print "[info]---get three way cutter by Bench:", three_way_cutter
    return ThreeWayCutter(three_way_cutter)


if __name__ == "__main__":
    assert len(sys.argv) == 3, "Wrong args!"
    status = int(sys.argv[2])
    set_relay_status_compatible(sys.argv[1], status)

