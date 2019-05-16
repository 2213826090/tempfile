# get the processes on the device
exec_status, output = DEVICE.run_cmd("adb shell ps -C hald", 3)  # @UndefinedVariable

# get the process owner
user = ""
if len(output.split("\n")) > 1:
    user = output.split("\n")[1].split()[0]

# check the owner
if exec_status == SUCCESS:
    if user == 'hal':
        VERDICT = SUCCESS
    else:
        VERDICT = FAILURE
        OUTPUT = "The hald process owner is not 'hal' but: {0}".format(user)
elif exec_status == FAILURE:
    VERDICT = FAILURE
    PRINT_ERROR("ECHO FAILURE: %s" % output)
else:
    PRINT_DEBUG("UNKNOWN STATUS")
    VERDICT = FAILURE
OUTPUT = output
