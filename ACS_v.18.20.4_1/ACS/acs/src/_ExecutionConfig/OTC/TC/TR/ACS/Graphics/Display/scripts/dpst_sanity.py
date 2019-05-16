import subprocess

process = subprocess.Popen("adb shell cat /sys/kernel/debug/dri/0/i915_capabilities",
                           stdout=subprocess.PIPE, shell=True)
(output, error) = process.communicate()
new_output = output.split("\n")
for line in new_output:
    if "dpst" in line.rstrip("\r"):
        output = line.rstrip("\r")
        if "yes" in output.split(":")[-1].replace(" ", ""):
            VERDICT = SUCCESS
        else:
            VERDICT = FAILURE
        break
OUTPUT = output