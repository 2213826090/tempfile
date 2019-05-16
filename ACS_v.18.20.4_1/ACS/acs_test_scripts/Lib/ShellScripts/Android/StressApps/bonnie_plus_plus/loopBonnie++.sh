#!/system/bin/sh

echo -e "\n\n"
echo "Usage: ./loopBonnie++.sh <minute> <size in MB> <target devices...>  "
echo "Example: ./loopBonnie++.sh 1440 2048 /dev/block/mmcblk0p9 /dev/block/mmcblk1p1"

script_args=( "$@" )
dir=`dirname $0`
log_file=$dir/logfile.log
fail_log_file=$dir/fail.log
pass_log_file=$dir/pass.log

rm -r $dir/*.log 2> /dev/null
echo "$# arguments provided: ${script_args[@]}"
# Check to make sure our devices exist and populate our variables
for i in `seq 2 $(($#-1))`
do
    echo "Checking if device '${script_args[$i]}' exists ..."
    if [ ! -a ${script_args[$i]} ]
    then
        echo "Device ${script_args[$i]} does not exist. Skipping this device." | tee -a $log_file
        continue
    fi
    echo "Device exists. Adding it to the list of devices to test."
    devices[$(($i-2))]=${script_args[$i]}
done

for i in `seq 0 9`
do
    if [ -d $dir/cvbonnie++$i ]; then
        umount $dir/cvbonnie++$i 2> /dev/null
        wait
        rm -r $dir/cvbonnie++$i 2> /dev/null
    fi
done

##### Functions
Bonnie(){
    echo "   Running bonnie++ in Target:$1 Size:$(($2))M:1024K Memory:$(($2/2))M" | tee -a $log_file
    $dir/bonnie++ -d $1 -s $2M:1024K -r $(($2/2))M -u 0 >> $3 2>&1 $4
    if [ $? -ne 0 ]; then
        echo "Error messages dectected!! $(date)" | tee -a $log_file
        cp $log_file $fail_log_file
        $dir/killer.sh
        exit 1
    fi
}

BonnieC2(){
    echo "   Running bonnie++ in Target:$1 Size:$(($2))M Memory:$(($2/2))M Concurrency:2" | tee -a $log_file
    $dir/bonnie++ -d $1 -c 2 -s $2M:1024k -r $(($2/2))M -u 0 >> $3 2>&1 $4
    if [ $? -ne 0 ]; then
        echo "Error messages dectected!! $(date)" | tee -a $log_file
        cp $log_file $fail_log_file
        $dir/killer.sh
        exit 1
    fi
}

Error_checking(){
    if [ -f $dir/$1 ]; then
        failure=$(grep -ic "\<error\>\|\<fail\>\|\<failed\>" $1)
        if [ $failure -ne 0 ]; then
            echo "$failure error detected!! $(date)" | tee -a $log_file
            cp $log_file $fail_log_file
            $dir/killer.sh
            exit 1
        fi
    fi
}
#####

echo "Start Time: $(date)" | tee -a $log_file

if [ -z "$1" ]; then 
    echo "   No test duration assigned. Default duration 24 hours" | tee -a $log_file
    test_time=$((1440*60))
else
    echo "   Test duration entered. Run Bonnie++ for $1 minutes" | tee -a $log_file
    test_time=$(($1*60))
fi

if [ -z "$2" ]; then 
    echo "   No file size entered. Run Bonnie++ in 1024MB" | tee -a $log_file
    size=1024
else
    size=$2
fi  
    
if [ -z "$devices" ]; then 
    echo "   No target device entered. Bonnie++ test file store in current location" | tee -a $log_file
    mkdir $dir/cvbonnie++0 2> /dev/null
    mounts[0]=$dir/cvbonnie++0
else
    filesystem_types=( ext3 vfat ext2 ext4 )
    i=0
    for dev_path in ${devices[@]}
    do
        mkdir $dir/cvbonnie++$i 2> /dev/null
        sleep 2
        echo "   Attempting to mount device $dev_path at $dir/cvbonnie++$i. Auto-detecting filesystem type..." | tee -a $log_file
        for fs_type in "${filesystem_types[@]}"
        do
            mount_cmd="mount -t $fs_type $dev_path $dir/cvbonnie++$i"
            echo -n "     Attempting FS type $fs_type with command: $mount_cmd. "
            mount_cmd_output=`$mount_cmd 2>&1`
            echo "Output: $mount_cmd_output."
            if [ "$mount_cmd_output" == "" ]
            then
                echo "   Mount successful. Filesystem Type: $fs_type"
                mounts[$i]=$dir/cvbonnie++$i
                break
            fi
        done
        i=$(($i+1))
    done
fi

# All loops/subloops should take roughly the same amount of time, so measure the
# time to execute the first loop and base decision on whether or not to execute
# each following loop on that.
typicalLoopTime=0

start_time_seconds=$(date +%s)
loop=1
while [ $((`date +%s`-$start_time_seconds)) -lt $test_time -a $(($test_time-(`date +%s`-$start_time_seconds))) -gt $typicalLoopTime ]
do
    echo " " | tee -a $log_file
    echo " " | tee -a $log_file
    echo `date` | tee -a $log_file
    echo "Test Time Remaining: $(($test_time-(`date +%s`-$start_time_seconds))) Seconds"
    echo "========= loop: $loop =========" | tee -a $log_file
    echo "   Test files generated in ./cvbonnie++[1-$(($#-2))]" | tee -a $log_file
    if [ $loop -eq 1 ]; then startLoopTime=$(date +%s); fi
    for mount in ${mounts[@]}
    do
        Bonnie $mount $size $mount.log &
    done
    wait
    if [ $loop -eq 1 ]; then typicalLoopTime=$((`date +%s`-$startLoopTime)); echo "Each loop takes roughly $typicalLoopTime seconds to complete"; fi
    # If we still have time left, do the second test
    if [ $((`date +%s`-$start_time_seconds)) -lt $test_time -a $(($test_time-(`date +%s`-$start_time_seconds))) -gt $typicalLoopTime ]
    then
        echo "   Test Time Remaining: $(($test_time-(`date +%s`-$start_time_seconds))) Seconds"
        for mount in ${mounts[@]}
        do
            BonnieC2 $mount $size $mount.log &
        done
        wait
    else
        echo "   Skipping BonnieC2 test. Out of time." | tee -a $log_file
    fi
    # Check for errors
    for mount in ${mounts[@]}
    do
        Error_checking $mount.log
    done
    loop=$(($loop+1))
done

echo "End Time: $(date)" | tee -a $log_file

# Unmount our mounts
for mount in ${mounts[@]}
do
    umount $dir/cvbonnie++$(($i-2)) 2> /dev/null
    wait
    rm -r $dir/cvbonnie++$(($i-2)) 2> /dev/null
done

# We passed! Copy our log file to pass.log to signify completion.
cp $log_file $pass_log_file
