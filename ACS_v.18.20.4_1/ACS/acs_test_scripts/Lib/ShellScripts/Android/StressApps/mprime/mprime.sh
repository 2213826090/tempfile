#!/system/bin/sh

dir=`dirname $0`
rm -f $dir/*.ini $dir/resu*.txt $dir/*.log 2> /dev/null

start=$(date)

if [ -z $1 -o -z $2 ]; then
	echo "command incomplete"
	echo "sh $dir/mprime.sh <duration in minutes> <memory size to test> <cpu speed>"
	echo "Eg. .$dir/mprime.sh 60 256 1600"
	echo "mprime: Test setup failed. $(date)" | tee -a $dir/fail.log
	exit -1
else
	

	echo "OldCpuSpeed=$3" > $dir/loca0000.txt
	echo "NewCpuSpeedCount=0" >> $dir/loca0000.txt
	echo "NewCpuSpeed=0"  >> $dir/loca0000.txt
	echo "ComputerGUID=89c0cfd62a11a02ca8135801cd7bd52f" >> $dir/loca0000.txt
	echo "Memory=$2 during 7:30-23:30 else 100" >> $dir/loca0000.txt

	echo "24OptionsConverted=1" > $dir/prim0000.txt
	echo "WGUID_version=2" >> $dir/prim0000.txt
	echo "StressTester=1" >> $dir/prim0000.txt
	echo "UsePrimenet=0" >> $dir/prim0000.txt
	echo "MinTortureFFT=8" >> $dir/prim0000.txt
	echo "MaxTortureFFT=4096" >> $dir/prim0000.txt
	echo "TortureMem=$2" >> $dir/prim0000.txt
	echo "TortureTime=$1" >> $dir/prim0000.txt
	echo "[PrimeNet]" >> $dir/prim0000.txt
	echo "Debug=0" >> $dir/prim0000.txt

	cp $dir/loca0000.txt $dir/loca0001.txt
	cp $dir/prim0000.txt $dir/prim0001.txt
fi

test_time=$(($1*60))

tested_time=0
starting=$(date -u +%s)

($dir/mprime_277_atom_static -a0 -T | tee -a $dir/a0.log &)
($dir/mprime_277_atom_static -a1 -T | tee -a $dir/a1.log &)

while [ $tested_time -lt $test_time ]
do
	end=$(date)
	if [ $? -ne 0 ]; then
		echo "mprime: start time: $start". $(date) | tee -a $dir/fail.log
		echo "*******a0.log**********" >> $dir/fail.log
		cat $dir/a0.log >> $dir/fail.log
		echo "*******a1.log**********" >> $dir/fail.log
		cat $dir/a1.log >> $dir/fail.log
		echo "End time: $end" | tee -a $dir/fail.log
		killall mprime
		exit 1
	fi
	ending=$(date -u +%s)
	tested_time=$(($ending-$starting))
done
	
echo "start time: $start" | tee -a $dir/pass.log
echo "*******a0.log**********" >> $dir/pass.log
       cat $dir/a0.log >> $dir/pass.log
       echo "*******a1.log**********" >> $dir/pass.log
       cat $dir/a1.log >> $dir/pass.log
echo "End time: $end" | tee -a $dir/pass.log

killall mprime_277_atom_static
