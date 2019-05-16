#!/system/bin/sh

# This script is written specially for Android OS. Automount for storage devices is not supported in this script. Tester need to manually mount all storage devices to be tested before the start of test. This script can be run before/after tester mount the storage devices but should be run before start of test.

# Usage: ./cv_storage_transfer.sh [OPTION]... [MINUTE] [DEVICE]...
# Example: ./cv_storage_transfer.sh -c -d -m 1440 /dev/block/mmcblk0p1 /dev/block/mmcblk0p2
#  -a   all
#  -c    concurrency
#  -d    dd-ing
#  -m    many2one
#  -o    one2many
#  -p    compress
#  -r    token ring
#  -t    touchmount
#  -h    help
# More details in test spec.

FAIL_LOG_FILE=fail.log

function usage() {
cat <<EOF

 Usage: ./cv_storage_transfer.sh [OPTION]... [MINUTE] [DEVICE]...
 Example: ./cv_storage_transfer.sh -c -d -m 1440 /dev/block/mmcblk0p1 /dev/block/mmcblk0p2

   -a   all
   -c    concurrency
   -d    dd-ing
   -m    many2one
   -o    one2many
   -p    compress
   -r    token ring
   -t    touchmount
   -h    help

EOF
exit 0
}

checktime(){
    ending=$(date -u +%s)
    tested_time=$(($ending-$starting))
    if [ $tested_time -ge $test_time ]; then
        break
    fi
}

##### Functions
clear_content(){
    echo "        Clearing all contents from previous test in the device..."
    size=0
    sdloop=1
    while [ $sdloop -le $stloop ]; do
        rm -r $stdir/cv$sdloop/tempfile$sdloop 2> /dev/null
        mkdir $stdir/cv$sdloop/tempfile$sdloop 2> /dev/null

        spaces=$(busybox df -m | grep cv$sdloop | awk '{ print($4) }')

        if [ $size -eq 0 ]; then
            size=$spaces
        elif [ $size -ge $spaces ]; then
            size=$spaces
        fi

        sdloop=$(($sdloop+1))
    done
    sync
}

set_size_rank(){

    if [ $size -ge 4000 ]; then
        size_rank=1
    elif [ $size -ge 2000 ]; then
        size_rank=2
    elif [ $size -ge 1000 ]; then
        size_rank=4
    elif [ $size -ge 512 ]; then
        size_rank=8
    else
        echo "Concurrency: One of the devices does not have enough space.\nIt has $(size) MB and at least 512 MB is needed, $(date)" | tee -a $dir/$FAIL_LOG_FILE
        exit 1
    fi
}

concurrency=0
compress=0
dding=0
many2one=0
one2many=0
tokenring=0
touchmount=0

while getopts ":acdhmoprt" opt; do
    case $opt in
        a) concurrency=1
           compress=1
           dding=1
           many2one=1
           one2many=1
           tokenring=1
           touchmount=1 ;;
        c) concurrency=1 ;;
        d) dding=1 ;;
        m) many2one=1 ;;
        o) one2many=1 ;;
        p) compress=1 ;;
        r) tokenring=1 ;;
        t) touchmount=1 ;;
        h) usage ;;
        \?) echo "./cv_storage_transfer.sh: invalid option -- '-$OPTARG'"; usage ;;
    esac
done

shift $(($OPTIND-1))

dir=`dirname $0`

# Storage Device initialize: Start ~
rm -r $dir/*.log 2> /dev/null
rm -r $dir/devices.txt 2> /dev/null
rm -r $dir/check_if_fail/* 2> /dev/null

i=1
while [ $i -le 9 ]; do
    if [ -d $dir/cv$i ]; then
        umount $dir/cv$i 2> /dev/null
        wait
        rm -r $dir/cv$i 2> /dev/null
    fi
    i=$(($i+1))
done
sync

echo "Start Time: $(date)" | tee -a $dir/logfile.log

if [ -z "$1" ]; then 
    echo "   No test duration assigned. Default duration 72 hours" | tee -a $dir/logfile.log
    test_time=$((72*3600))
else
    echo "   Test duration entered. Run cv_storage_transfer for $1 minute(s)" | tee -a $dir/logfile.log
    test_time=$(($1*60))
fi

if [ -z "$2" ]; then 
    echo "   No target device entered." | tee -a $dir/logfile.log
    usage
    exit 0
else
    i=1
    for storage_device in "$@"; do
        if [ $i -gt 1 ]; then
            mkdir $dir/cv$(($i-1)) 2> /dev/null
            sleep 2
            
            echo "   Working on $storage_device device mounted to ./cv$(($i-1))" | tee -a $dir/logfile.log
            mount -t ext3 $storage_device $dir/cv$(($i-1)) 2> /dev/null
            mount -t vfat $storage_device $dir/cv$(($i-1)) 2> /dev/null
            mount -t ext2 $storage_device $dir/cv$(($i-1)) 2> /dev/null
            mount -t ext4 $storage_device $dir/cv$(($i-1)) 2> /dev/null
            
            sleep 2
            mnt=$(df | grep $dir/cv$(($i-1)))
            if [ -z "$mnt" ]; then
                echo "Failed in $storage_device device mounted to ./cv$(($i-1)), $(date)" | tee -a $dir/logfile.log
                echo "Failed in $storage_device device mounted to ./cv$(($i-1)), $(date)" >> $dir/$FAIL_LOG_FILE
                exit 1
            fi

            rm -r $dir/cv$(($i-1))/tempfile* 2> /dev/null
            sync

            echo $storage_device >> $dir/devices.txt
        fi
        
        i=$(($i+1))
    done
fi
export size=0
export size_rank=0
export last_size=0
export stdir=$dir
export stloop=$(($i-2))
export mediapath=$dir/media/2GBFiles
tested_time=0
starting=$(date -u +%s)
run_time=0
time_left=0
size_change=0
cvloop=1
duration=0
duration_in_seconds=0
while [ $tested_time -lt $test_time ]
do
    echo " " | tee -a $dir/logfile.log
    echo " " | tee -a $dir/logfile.log
    echo `date` | tee -a $dir/logfile.log
    echo "========= loop: $cvloop =========" | tee -a $dir/logfile.log
    if [ $tokenring -eq 1 ]; then
        echo -n "   Running token_ring.sh..." | tee -a $dir/logfile.log
        start=$(date +%s)
        sh $dir/token_ring.sh 1
        stop=$(date +%s)
        duration=$((($stop-$start)/60+1))
        echo "$duration minutes" | tee -a $dir/logfile.log
    fi

    checktime

    if [ $touchmount -eq 1 ]; then
        echo -n "   Running touchmount.sh..." | tee -a $dir/logfile.log
        start=$(date +%s)
        sh $dir/touchmount.sh
        stop=$(date +%s)
        duration=$((($stop-$start)/60+1))
        echo "$duration minutes" | tee -a $dir/logfile.log
    fi

    checktime

    if [ $many2one -eq 1 ]; then
        echo -n "   Running many2one.sh..." | tee -a $dir/logfile.log
        start=$(date +%s)
        sh $dir/many2one.sh 1
        stop=$(date +%s)
        duration=$((($stop-$start)/60+1))
        echo "$duration minutes" | tee -a $dir/logfile.log
    fi

    checktime

    if [ $one2many -eq 1 ]; then
        echo -n "   Running one2many.sh..." | tee -a $dir/logfile.log
        start=$(date +%s)
        sh $dir/one2many.sh 1
        stop=$(date +%s)
        duration=$((($stop-$start)/60+1))
        echo "$duration minutes" | tee -a $dir/logfile.log
    fi

    checktime

    if [ $concurrency -eq 1 ]; then
        echo -n "   Running concurrency.sh..." | tee -a $dir/logfile.log
        clear_content
        set_size_rank
        duration_in_seconds=$(($duration*60))
        if [ $size_rank -eq $last_size ] && [ $last_size -ne 0 ]; then
            #same size available as last time
            time_left=$(($duration_in_seconds + $tested_time))
            #if time to run it again, do it
            if [ $time_left -lt $test_time ]; then
                start=$(date +%s)
                sh $dir/concurrency.sh 1
            else
                #must not have enough time left to run same size as last time
                time_left=$(($test_time - $tested_time))
                run_time=$(($duration_in_seconds/$time_left))
                if [ $size_rank -eq 1 ]; then
                    #if it was 2GB last time do we have time to do something at 1/2, 1/4 or 1/8 the size and therefore something taking 1/2, 1/4 or 1/8 the time, respectively?
                    if [ $run_time -le 2 ]; then
                        size_rank=2
                        start=$(date +%s)
                        sh $dir/concurrency.sh 1
                    elif [ $run_time -gt 2 ] && [ $run_time -le 4 ]; then
                        size_rank=4
                        start=$(date +%s)
                        sh $dir/concurrency.sh 1
                    elif [ $run_time -gt 4 ] && [ $run_time -le 8 ]; then
                        size_rank=8
                        start=$(date +%s)
                        sh $dir/concurrency.sh 1
                    else
                        break
                    fi
                elif [ $size_rank -eq 2 ]; then
                    #if it was 1GB last time do we have time to do something at 1/2 or 1/4 the size and therefore something taking 1/2 or 1/4 the time, respectively?
                    if [ $run_time -le 2 ]; then
                        size_rank=4
                        start=$(date +%s)
                        sh $dir/concurrency.sh 1
                    elif [ $run_time -gt 2 ] && [ $run_time -le 4 ]; then
                        size_rank=8
                        start=$(date +%s)
                        sh $dir/concurrency.sh 1
                    else
                        break
                    fi
                elif [ $size_rank -eq 4 ]; then
                    #if it was 500MB last time do we have time to do something at 1/2 the size and therefore something taking 1/2 the time?
                    if [ $run_time -le 2 ]; then
                        size_rank=8
                        start=$(date +%s)
                        sh $dir/concurrency.sh 1
                    else
                        break
                    fi
                else
                    #Must have been 256MB last time and we don't even have time to do something that size again.
                    break
                fi
            fi
        elif [ $size_rank -lt $last_size ] && [ $last_size -ne 0 ]; then
            #smaller size available than last time
            time_left=$(($duration_in_seconds + $tested_time))
            #if time to run the bigger size, then time to run smaller size.
            if [ $time_left -lt $test_time ]; then
                start=$(date +%s)
                sh $dir/concurrency.sh 1
            else
                time_left=$(($test_time - $tested_time))
                run_time=$(($duration_in_seconds/$time_left))
                size_change=$(($last_size/$size_rank))
                if [ $size_change -eq 2 ] && [ $run_time -le 2 ]; then
                    #size is half of last time and we have time to run something up to half the size as last time.
                    start=$(date +%s)
                    sh $dir/concurrency.sh 1
                elif [ $size_change -eq 2 ] && [ $run_time -gt 2 ] && [ $run_time -le 4 ]; then
                    #size is half of last time but time left is enough for something at 1/4 size.
                    if [ $size_rank -eq 2 ]; then
                        #we'll do 1/4 of 2GB (2GB==last size)
                        size_rank=4
                        start=$(date +%s)
                        sh $dir/concurrency.sh 1
                    elif [ $size_rank -eq 4 ]; then
                        #we'll do 1/4 of 1GB (1GB==last size)
                        size_rank=8
                        start=$(date +%s)
                        sh $dir/concurrency.sh 1
                    else
                        #not doing 1/4 or 512MB, so quit.
                        break
                    fi
                elif [ $size_change -eq 2 ] && [ $run_time -gt 4 ] && [ $run_time -le 8 ]; then
                    #size is half of last time but time left is only enough for something 1/8 size.
                    if [ $size_rank -eq 2 ]; then
                        #we'll do 1/8 of 2GB.
                        size_rank=8
                        start=$(date +%s)
                        sh $dir/concurrency.sh 1
                    else
                        #we aren't going smaller than 256MB, so quit.
                        break
                    fi
                elif [ $size_change -eq 4 ] && [ $run_time -le 4 ]; then
                    #size is 1/4 that of last time and we have time to do that.
                    start=$(date +%s)
                    sh $dir/concurrency.sh 1
                elif [ $size_change -eq 4 ] && [ $run_time -gt 4 ] && [ $run_time -le 8 ]; then
                    #size is 1/4 that of last time but we have time for 1/8 size of last time only.
                    if [ $size_rank -eq 4 ]; then
                        #so, we have time to do 256MB
                        size_rank=8
                        start=$(date +%s)
                        sh $dir/concurrency.sh 1
                    else
                        #but we aren't doing smaller than 256MB
                        break
                    fi
                elif [ $size_change -eq 8 ] && [ $run_time -le 8 ]; then
                    #size is 1/8 of last time, so we need enough time to do 256MB.
                    start=$(date +%s)
                    sh $dir/concurrency.sh 1
                else
                    #if there wasn't enough time for 256MB, then quit.
                    break
                fi
            fi
        elif [ $size_rank -gt $last_size ] && [ $last_size -ne 0 ]; then
            #bigger size available than last time (2GB, 1GB or 512MB)
            time_left=$(($duration_in_seconds + $tested_time))
            if [ $time_left -gt $test_time ]; then
                #bigger size and less time than before.
                time_left=$(($test_time - $tested_time))
                run_time=$(($duration_in_seconds/$time_left))
                if [ $run_time -lt 2 ]; then
                    #We have enough time to do 1/2 size as last time.
                    if [ $last_size -eq 8 ]; then
                        #So, we definitely aren't doing less than 512MB
                        break
                    else
                        #We can do half of last_size
                        size_rank=$(($last_size/2))
                        start=$(date +%s)
                        sh $dir/concurrency.sh 1
                    fi
                elif [ $run_time -ge 2 ] && [ $run_time -lt 4 ]; then
                    #time to do 1/4 size of last time
                    if [ $last_size -ge 4 ]; then
                        #nothing to do if last size was 512MB or 256MB
                        break
                    else
                        #We can do 1/4 or 2GB or 1GB though.
                        size_rank=$(($last_size/4))
                        start=$(date +%s)
                        sh $dir/concurrency.sh 1
                    fi
                elif [ $run_time -ge 4 ] && [ $run_time -lt 8 ]; then
                    #time to do 1/8 size of last time
                    if [ $last_size -ge 2 ]; then
                        #nothing to do if last size was 1GB, 512MB or 256MB
                        break
                    else
                        #We can do 1/4 or 2GB though.
                        size_rank=8
                        start=$(date +%s)
                        sh $dir/concurrency.sh 1
                    fi
                else
                    break
                fi
            else
                #bigger size and at least enough time to do same size as last time...
                time_left=$(($test_time - $tested_time))
                #so, let's use run_time for having time_left greater than duration_in_seconds.
                run_time=$(($time_left/$duration_in_seconds))
                if [ $run_time -lt 2 ]; then
                    #Only enough time to do same as last time.
                    size_rank=$last_size
                    start=$(date +%s)
                    sh $dir/concurrency.sh 1
                elif [ $run_time -ge 2 ] && [ $run_time -lt 4 ]; then
                    #Time to do 2x but less than 4x last time.
                    size_rank=$(($last_size/2))
                    start=$(date +%s)
                    sh $dir/concurrency.sh 1
                elif [ $run_time -ge 4 ] && [ $run_time -lt 8 ]; then
                    #Time to do 4x but less than 8x last time.
                    if [ $size_rank -eq 1 ] && [ $last_size -eq 8]; then
                        #Can't do 2GB if we were 1/8 size last time, so reduce to 1GB.
                        size_rank=2
                    fi
                    start=$(date +%s)
                    sh $dir/concurrency.sh 1
                else
                    #Lots of time, let's do whatever we had space for.
                    start=$(date +%s)
                    sh $dir/concurrency.sh 1
                fi
            fi
        else
            #first time through
            start=$(date +%s)
            sh $dir/concurrency.sh 1
        fi
        stop=$(date +%s)
        duration=$((($stop-$start)/60+1))
        last_size=$size_rank
        echo "$duration minutes" | tee -a $dir/logfile.log
    fi

    checktime

    if [ $dding -eq 1 ]; then
        echo -n "   Running dd-ing.sh...   " | tee -a $dir/logfile.log
        start=$(date +%s)
        $dir/bash $dir/dd-ing.sh 1
        stop=$(date +%s)
        duration=$((($stop-$start)/60+1))
        echo "$duration minutes" | tee -a $dir/logfile.log
    fi

    checktime

    if [ $compress -eq 1 ]; then
        echo -n "   Running compress.sh..." | tee -a $dir/logfile.log
        start=$(date +%s)
        sh $dir/compression.sh 1
        stop=$(date +%s)
        duration=$((($stop-$start)/60+1))
        echo "$duration minutes" | tee -a $dir/logfile.log
    fi
        
    cvloop=$(($cvloop+1))
    ending=$(date -u +%s)
    tested_time=$(($ending-$starting))
done

echo "End Time: $(date)" | tee -a $dir/logfile.log

i=2
while [ $i -le $# ]; do
    umount $dir/cv$(($i-1)) 2> /dev/null
    wait
    rm -r $dir/cv$(($i-1)) 2> /dev/null
    i=$(($i+1))
done

rm -r $dir/devices.txt 2> /dev/null
cat $dir/logfile.log >> $dir/pass.log

# Storage Device test: End ~
