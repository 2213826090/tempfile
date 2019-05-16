echo $$ > pid.txt
echo "num cpu_temp board_temp cpu_freq  power gpu_freq">$1
i=0;
while (( 1 ))
do
    i=$(($i+1))
    CMD_TEMP0=`cat /sys/class/thermal/thermal_zone0/temp |sed 's/000/00000/'`
    CMD_TEMP1=`cat /sys/class/thermal/thermal_zone1/temp |sed 's/000/00000/'`
    CMD_FREQ=`cat /sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_cur_freq`
    CMD_POWER=`cat /sys/class/powercap/intel-rapl:0/constraint_0_power_limit_uw`
    CMD_GPU=`cat /d/dri/0/i915_frequency_info |grep -i 'Current freq' |sed 's/Current freq://g' |sed 's/ MHz/0000/'`
    echo " $i  $CMD_TEMP0  $CMD_TEMP1  $CMD_FREQ   $CMD_POWER   $CMD_GPU" >>$1
    sleep 2
done
