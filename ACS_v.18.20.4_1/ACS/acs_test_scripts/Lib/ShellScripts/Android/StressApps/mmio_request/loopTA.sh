#!/system/bin/sh

dir=`dirname $0`
rm -r $dir/*.log $dir/*.txt  2> /dev/null

if [ -z $1 ]; then
	echo "	TA: Need test duration" | tee -a $dir/fail.log
        echo "	Usage: sh $dir/loopTA.sh <duration in minutes>" | tee -a $dir/fail.log
	exit 1
else
        test_time=$(($1*60))
        echo "TA: Test Duration : $1 minute(s)" | tee -a $dir/logfile.txt
fi

tested_time=0
starting=$(date -u +%s)

echo "Start time : $(date)" | tee -a $dir/logfile.txt
while [ $tested_time -lt $test_time ]
do
	

	        j=1
		echo "This is loop number $j" | tee -a $dir/logfile.txt
		$dir/mmio_read_write_aligned | tee -a $dir/logfile.txt
                $dir/mmio_read_write_unaligned | tee -a $dir/logfile.txt 
                 
			if [ $? != 0 ]
			then
				echo "*************ERROR OCCURS****************" | tee -a $dir/logfile.txt
				echo "Recent Time : $(date)" | tee -a $dir/logfile.txt
				mv $dir/logfile.txt $dir/fail.log
				exit -1
			fi
			ending=$(date -u +%s)
                	tested_time=$(($ending-$starting))
                	if [ $tested_time -gt $test_time ]; then
				echo "End Time : $(date)" | tee -a $dir/logfile.txt
	                        mv $dir/logfile.txt $dir/pass.log
				exit 0
	                fi
			
		
		j=$(($j+1))
	

	
	

ending=$(date -u +%s)
tested_time=$(($ending-$starting))

done
echo "End Time : $(date)" | tee -a $dir/logfile.txt
cat $dir/logfile.txt >> $dir/pass.log
