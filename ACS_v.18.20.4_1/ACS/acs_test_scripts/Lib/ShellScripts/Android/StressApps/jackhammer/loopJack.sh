#!/system/bin/sh

dir=`dirname $0`
rm -r $dir/*.log $dir/*.txt Jackhammer0.log jackhammer.log 2> /dev/null

if [ -z $1 ]; then
	echo "	Jackhammer: Need test duration" | tee -a $dir/fail.log
        echo "	Usage: sh $dir/loopJackhammer.sh <duration in minutes>" | tee -a $dir/fail.log
	exit 1
else
        test_time=$(($1*60))
        echo "Jackhammer: Test Duration : $1 minute(s)" | tee -a $dir/logfile.txt
fi

tested_time=0
starting=$(date -u +%s)

echo "Start time : $(date)" | tee -a $dir/logfile.txt
while [ $tested_time -lt $test_time ]
do
	x=$(grep -c "processor" /proc/cpuinfo)

	j=1
	while [ $j -lt 4 ]; do
		echo "This is loop number $j" | tee -a $dir/logfile.txt
		y=1
		while [ $y -lt 17 ]; do
			echo "$dir/Jackhammer 1 70 $y 300" | tee -a $dir/logfile.txt 
			$dir/Jackhammer 1 70 $y 300 
			if [ $? != 0 ]
			then
				echo "*************ERROR OCCURS****************" | tee -a $dir/logfile.txt
				echo "Recent Time : $(date)" | tee -a $dir/logfile.txt
				y=17
				j=4
				i=500
				a=17
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
			y=$(($y+1))
		done
		j=$(($j+1))
	done

	a=1
	while [ $a -lt 17 ]; do
		p=$[`($RANDOM % 10489) + 1`]
		z=$[`($RANDOM % $x) + 1`]
		d=$[`($RANDOM % 16) + 1`]
		echo "$dir/Jackhammer $z $p $d 300" | tee -a $dir/logfile.txt
		$dir/Jackhammer $z $p $d 300 
		if [ $? != 0 ]
		then
			echo "*************ERROR OCCURS****************" | tee -a $dir/logfile.txt
			echo "Recent Time : $(date)" | tee -a $dir/logfile.txt
			y=17
			j=4
			i=500
			a=17
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
		a=$(($a+1))
	done

ending=$(date -u +%s)
tested_time=$(($ending-$starting))

done
echo "End Time : $(date)" | tee -a $dir/logfile.txt
cat $dir/logfile.txt >> $dir/pass.log
