import subprocess

device_resolution = TC_PARAMETERS("RESOLUTION")
process = subprocess.Popen("adb shell dumpsys | grep \"DisplayDeviceInfo\"", stdout=subprocess.PIPE, shell=True)
(output, error) = process.communicate()
resolution = output.split("{")[-1].split("}")[0].split(":")[-1].split(",")[0].replace(" ", "")
if resolution in device_resolution:
    VERDICT = SUCCESS
else:
    VERDICT = FAILURE
OUTPUT = output