echo $$ > pid.txt
#now=$(date +"%m_%d_%Y_%H_%M_%S")
#File="out_cpu_gpu_cur_"$now".csv"
File=$1
inter=","
CPU_DIR="/sys/devices/system/cpu"
cd $CPU_DIR
cpus=($(ls -d cpu?))
cd -
echo -n no"$inter" > $File
for cpu in ${cpus[@]}
do
    echo -n $cpu$inter >> $File
done
echo cpu_max${inter}gpu_act${inter}gpu_req >> $File

no=0
while true
do
    echo -n $no$inter >> $File
    no=$((no+1))
    max=0
    for cpu in ${cpus[@]}
    do
        cpu_cur="$CPU_DIR/${cpu}/cpufreq/scaling_cur_freq"
        freq=$(cat $cpu_cur | sed 's/[^0-9]//g')
        freq=$(($freq/1000))
        if [ $freq -gt $max ]
        then
            max=$freq
        fi
        echo -n $freq$inter >> $File
    done
    echo -n $max$inter >> $File

    gpu_node="/d/dri/0/i915_frequency_info"
    act_freq=`cat $gpu_node | grep "Actual freq"  | sed 's/[^0-9]//g'`
    req_freq=`cat $gpu_node | grep "Current freq" | sed 's/[^0-9]//g'`
    echo -n $act_freq$inter >> $File
    echo $req_freq >> $File
    sleep 0.1
done
