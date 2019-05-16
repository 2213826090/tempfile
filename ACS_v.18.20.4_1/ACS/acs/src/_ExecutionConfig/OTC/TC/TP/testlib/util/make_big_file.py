from testlib.util.common import g_common_obj

def create_file(size = 5):
    count = 1024 * size
    command = "dd if=/dev/zero of=/mnt/sdcard/" + str(size) + "Mbigfile bs=1024 count=" + str(count)
    result = g_common_obj.adb_cmd_capture_msg(command, 1200)
    print "The result of create big file :" + result
    if result.find(str(count)) != -1:
        return True , "/mnt/sdcard/" + str(size) + "Mbigfile"
    return False,"Not fill the file into device"

def create_file_no_space():
    command = "dd if=/dev/zero of=/mnt/sdcard/bigfile"
    result = g_common_obj.adb_cmd_capture_msg(command, 1200)
    print "The result of create big file till momery full :" + result
    if result.find("No space left on device") != -1:
        return True , "/mnt/sdcard/bigfile"
    return False,"Not fill the file into device"

def fill_no_space_except(size = 5):
    result , msg = create_file(size)
    if result:
        result1 , msg1 = create_file_no_space()
        if result1:
            g_common_obj.adb_cmd_capture_msg("rm -rf " + msg)
            return True , "/mnt/sdcard/bigfile"
        else:
            return result1 , msg1
    else:
        return result, msg

