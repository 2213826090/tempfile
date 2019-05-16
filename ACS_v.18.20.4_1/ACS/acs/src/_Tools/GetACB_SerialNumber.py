import ctypes

usb_dio_lib = ctypes.cdll.LoadLibrary("AIOUSB.dll")


def get_serial_number():
    """
    Get serial number of all connected devices
    """
    devices = usb_dio_lib.GetDevices()
    print("\n*************** Searching for USB DIO Serial Numbers ***************")
    if devices != 0:
        device_index = 0
        serial_number = ctypes.c_uint64()
        while device_index < 31:
            if (1 << device_index) & devices:

                # Get one device, this is the one
                print("\n    -----------------------------------")
                print("    USB DIO Device: %s" % str(device_index))
                usb_dio_lib.GetDeviceSerialNumber(device_index, ctypes.byref(serial_number))
                print("    USB DIO SN: %s" % serial_number.value)
                print("    -----------------------------------\n")
            device_index += 1

    else:
        print ("\nDevices not found or driver not installed properly.\n")

    print("********************************************************************")

if __name__ == "__main__":

    get_serial_number()
    print("")
