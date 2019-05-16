import time
import uuid
from random import randint
import filecmp
from iterative_copy_lib import *

VERDICT = SUCCESS
OUTPUT = ""

# get the parameters
dev_memory_path = TC_PARAMETERS("DEVICE_MEMORY_PATH")
sdcard_memory_path = TC_PARAMETERS("SDCARD_MEMORY_PATH")
iterations = int(TC_PARAMETERS("ITERATIONS"))

# create the temp folders
gen_path = str(uuid.uuid4())

# local directory
os.mkdir(gen_path)

# on device dirs
dev_memory_path += "/" + gen_path
exec_status, output = DEVICE.run_cmd("adb shell mkdir {0}".format(dev_memory_path), 3)

sdcard_memory_path += "/" + gen_path
exec_status, output = DEVICE.run_cmd("adb shell mkdir {0}".format(sdcard_memory_path), 3)


for i in range(0, iterations):
    # generate the file name and file size
    file_name = str(uuid.uuid4())
    file_size = randint(1024,102400) # size between 1k and 100k

    # generate file on dev_memory_path
    gen_random_binary_file(gen_path + "/" + file_name, file_size)

    # copy the file to sdcard_memory_path
    exec_status, output = DEVICE.run_cmd("adb push {0} {1}".format(gen_path + "/" + file_name, dev_memory_path), 3)
    exec_status, output = DEVICE.run_cmd("adb shell cp {0} {1}".format(dev_memory_path + "/" + file_name,\
                                        sdcard_memory_path + "/" + file_name), 3)

    # check the content of the file
    exec_status, output = DEVICE.run_cmd("adb pull {0} {1}".format(sdcard_memory_path + "/" + file_name,\
                                        gen_path + "/" + file_name + "_from_sdcard"), 3)
    # compare the two files
    if not filecmp.cmp(gen_path + "/" + file_name, gen_path + "/" + file_name + "_from_sdcard"):
        VERDICT = FAILURE
        OUTPUT += "The files were differ after copy."

    # delete the files from the PC machine
    os.remove(gen_path + "/" + file_name)
    os.remove(gen_path + "/" + file_name + "_from_sdcard")

# delete the directories/files from device and PC
DEVICE.run_cmd("adb shell rm -r {0}".format(dev_memory_path), 3)
DEVICE.run_cmd("adb shell rm -r {0}".format(sdcard_memory_path), 3)
os.rmdir(gen_path)

