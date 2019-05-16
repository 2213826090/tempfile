#!/system/bin/sh

echo -e "\n\n"
echo "Usage: ./loopIOZone.sh <minute> <size in MB> <target devices...>  "
echo "Example: ./loopIOZone.sh 1440 2048 /dev/block/mmcblk0p1 /dev/block/mmcblk0p2"

dir=`dirname $0`
script_args=( "$@" )

rm -r $dir/*.log 2> /dev/null
echo "$# arguments provided: ${script_args[@]}"
# Check to make sure our devices exist and populate our variables
for i in `seq 2 $(($#-1))`
do
    echo "Checking if device '${script_args[$i]}' exists ..."
    if [ ! -a ${script_args[$i]} ]
    then
        echo "Device ${script_args[$i]} does not exist. Skipping this device." | tee -a $dir/logfile.log
        continue
    fi
    echo "Device exists. Adding it to the list of devices to test."
    devices[$(($i-2))]=${script_args[$i]}
done

i=0
while [ $i -le 9 ]; do
    if [ -d $dir/cviozone$i ]; then
        echo "Umounting and removing $dir/cviozone$i"
        umount $dir/cviozone$i 2> /dev/null
        wait
        rm -r $dir/cviozone$i 2> /dev/null
    fi
    i=$(($i+1))
done

echo "Start Time: $(date)" | tee -a $dir/logfile.log

if [ -z "$1" ]; then
    echo "   No test duration assigned. Default duration 24 hours" | tee -a $dir/logfile.log
    test_time=$((1440*60))
else
    echo "   Test duration entered. Run IOZone for $1 minutes" | tee -a $dir/logfile.log
    test_time=$(($1*60))
fi

if [ -z "$2" ]; then
    echo "   No file size entered. Run IOZone in 1024MB" | tee -a $dir/logfile.log
    size=1024
else
    size=$2
fi

if [ -z "$3" ]; then
    echo "   No target device entered. IOZone test file store in current location" | tee -a $dir/logfile.log
else
    location=""
    filesystem_types=( ext3 vfat ext2 ext4 )
    i=3
    for dev_path in ${devices[@]}
    do
        mkdir $dir/cviozone$(($i-2)) 2> /dev/null
        sleep 2
        echo "   Attempting to mount device $dev_path to ./cviozone$(($i-2))" | tee -a $dir/logfile.log
        for fs_type in "${filesystem_types[@]}"
        do
            mount_cmd="mount -t $fs_type $dev_path $dir/cviozone$(($i-2))"
            echo -n "     Attempting FS type $fs_type with command: $mount_cmd. "
            mount_cmd_output=`$mount_cmd 2>&1`
            echo "Output: $mount_cmd_output."
            if [ "$mount_cmd_output" == "" ]
            then
                echo "   Mount successful. Filesystem Type: $fs_type"
                mounts[$i]=$dir/cviozone$(($i-2))
                break
            fi
        done
        location="$location $dir/cviozone$(($i-2))/f$(($i-2))"
        i=$(($i+1))
    done
fi

tested_time=0
starting=$(date -u +%s)

loop=1
while [ $tested_time -lt $test_time ]
do
    echo " " | tee -a $dir/logfile.log
    echo " " | tee -a $dir/logfile.log
    echo `date` | tee -a $dir/logfile.log
    echo "========= loop: $loop =========" | tee -a $dir/logfile.log

    i=1
    while [ $tested_time -lt $test_time -a $i -le 12 ]; do
        echo "Getting ready to run test $i"
        if [ -z $3 ]; then
            echo "   Test files generated in current location" | tee -a $dir/logfile.log
            echo "   Running IOZone Size:$(($size))M Block:8K Test:$i..."
            $dir/iozone -i 0 -i $i -l 1 -u 1 -r 8k -s $(($size))M >> $dir/logfile.log 2>&1
            if [ $? -ne 0 ]; then
                echo "Error messages dectected!! $(date)" | tee -a $dir/logfile.log
                cat $dir/logfile.log >> $dir/fail.log
                $dir/killer.sh
                exit 1
            fi
        else
            echo "   Test files generated in $location" | tee -a $dir/logfile.log
            echo "   Running IOZone Size:$(($size))M Block:8K Process:$(($#-2)) Test:$i..."
            $dir/iozone -i 0 -i $i -l $(($#-2)) -u $(($#-2)) -r 8k -s $(($size))M -F $location >> $dir/logfile.log 2>&1
            if [ $? -ne 0 ]; then
                echo "Error messages dectected!! $(date)" | tee -a $dir/logfile.log
                cat $dir/logfile.log >> $dir/fail.log
                $dir/killer.sh
                exit 1
            fi
        fi
        wait

        ending=$(date -u +%s)
        tested_time=$(($ending-$starting))
        echo "\nTime elapsed after test $i: $tested_time \n"
        i=$(($i+1))
    done

    if [ -f $dir/logfile.log ]; then
        failure=$(grep -ic "\<error\>\|\<fail\>\|\<failed\>" $dir/logfile.log)
        if [ $failure -ne 0 ]; then
            echo "$failure error detected!! $(date)" | tee -a $dir/logfile.log
            cat $dir/logfile.log >> $dir/fail.log
            $dir/killer.sh
            exit 1
        fi
        
    fi

    loop=$(($loop+1))
done

echo "End Time: $(date)" | tee -a $dir/logfile.log

i=3
while [ $i -le $# ]; do
    umount $dir/cviozone$(($i-2)) 2> /dev/null
    wait
    rm -r $dir/cviozone$(($i-2)) 2> /dev/null
    i=$(($i+1))
done

cat $dir/logfile.log >> $dir/pass.log
