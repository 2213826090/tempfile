import subprocess

process = subprocess.Popen("adb shell cat /d/dri/0/i915_display_info", stdout=subprocess.PIPE, shell=True)
(output, error) = process.communicate()
if "DSI" in output:
    VERDICT = SUCCESS
else:
    VERDICT = FAILURE
OUTPUT = output