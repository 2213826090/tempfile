#!/system/bin/sh

dir=`dirname $0`

test_time=$(($1*60))
tested_time=0
starting=$(date -u +%s)
set clobber
cb=cachebench
procs=4
cat /proc/cpuinfo > $dir/results/run
date >> $dir/results/run

mem=24
res=3
dur=5

mkdir $dir/results 2> /dev/null
sleep 3
function timecheck
{
	ending=$1
	tested_time=$(($ending-$starting))
	if [ $tested_time -gt $test_time ]; then
		echo "	Cachebench Tests Completed. $(date)" | tee -a $dir/results/run
		exit 1
	fi
}

while [ $tested_time -lt $test_time ] 
do
	$dir/cachebench -b -d${dur} -x${res} -e1 -m${mem} > $dir/results/rmw1.1 &
	wait
	timecheck `date -u +%s`
	$dir/cachebench -b -d${dur} -x${res} -e1 -m${mem} > $dir/results/rmw2.1 &
	$dir/cachebench -b -d${dur} -x${res} -e1 -m${mem} > $dir/results/rmw2.2 &
	wait
	timecheck `date -u +%s`
 
	$dir/cachebench -b -t -d${dur} -x${res} -e1 -m${mem} > $dir/results/rmwt1.1 &
	wait
	timecheck `date -u +%s`
	$dir/cachebench -b -t -d${dur} -x${res} -e1 -m${mem} > $dir/results/rmwt2.1 &
	$dir/cachebench -b -t -d${dur} -x${res} -e1 -m${mem} > $dir/results/rmwt2.2 &
	wait
	timecheck `date -u +%s`
                                                                              
	$dir/cachebench -r -d${dur} -x${res} -e1 -m${mem} > $dir/results/read1.1 &
	wait
	timecheck `date -u +%s`
	$dir/cachebench -r -d${dur} -x${res} -e1 -m${mem} > $dir/results/read2.1 &
	$dir/cachebench -r -d${dur} -x${res} -e1 -m${mem} > $dir/results/read2.2 &
	wait
	timecheck `date -u +%s`
	                                                                             
	$dir/cachebench -r -t -d${dur} -x${res} -e1 -m${mem} > $dir/results/readt1.1 &
	wait
	timecheck `date -u +%s`
	$dir/cachebench -r -t -d${dur} -x${res} -e1 -m${mem} > $dir/results/readt2.1 &
	$dir/cachebench -r -t -d${dur} -x${res} -e1 -m${mem} > $dir/results/readt2.2 &
	wait
	timecheck `date -u +%s`
	                                                                               
	$dir/cachebench -p -d${dur} -x${res} -e1 -m${mem} > $dir/results/mcpy1.1 &
	wait
	timecheck `date -u +%s`
	$dir/cachebench -p -d${dur} -x${res} -e1 -m${mem} > $dir/results/mcpy2.1 &
	$dir/cachebench -p -d${dur} -x${res} -e1 -m${mem} > $dir/results/mcpy2.2 &
	wait
	timecheck `date -u +%s`
	
	if [ $procs == 4 ]; then
		$dir/cachebench -b -d${dur} -x${res} -e1 -m${mem} > $dir/results/rmw4.1 &
		$dir/cachebench -b -d${dur} -x${res} -e1 -m${mem} > $dir/results/rmw4.2 &
		$dir/cachebench -b -d${dur} -x${res} -e1 -m${mem} > $dir/results/rmw4.3 &
		$dir/cachebench -b -d${dur} -x${res} -e1 -m${mem} > $dir/results/rmw4.4 &
		wait
		timecheck `date -u +%s`
		$dir/cachebench -b -t -d${dur} -x${res} -e1 -m${mem} > $dir/results/rmwt4.1 &
		$dir/cachebench -b -t -d${dur} -x${res} -e1 -m${mem} > $dir/results/rmwt4.2 &
		$dir/cachebench -b -t -d${dur} -x${res} -e1 -m${mem} > $dir/results/rmwt4.3 &
		$dir/cachebench -b -t -d${dur} -x${res} -e1 -m${mem} > $dir/results/rmwt4.4 &
		wait
		timecheck `date -u +%s`
		$dir/cachebench -r -d${dur} -x${res} -e1 -m${mem} > $dir/results/read4.1 &
		$dir/cachebench -r -d${dur} -x${res} -e1 -m${mem} > $dir/results/read4.2 &
		$dir/cachebench -r -d${dur} -x${res} -e1 -m${mem} > $dir/results/read4.3 &
		$dir/cachebench -r -d${dur} -x${res} -e1 -m${mem} > $dir/results/read4.4 &
		wait
		timecheck `date -u +%s`
		$dir/cachebench -r -t -d${dur} -x${res} -e1 -m${mem} > $dir/results/readt4.1 &
		$dir/cachebench -r -t -d${dur} -x${res} -e1 -m${mem} > $dir/results/readt4.2 &
		$dir/cachebench -r -t -d${dur} -x${res} -e1 -m${mem} > $dir/results/readt4.3 &
		$dir/cachebench -r -t -d${dur} -x${res} -e1 -m${mem} > $dir/results/readt4.4 &
		wait
		timecheck `date -u +%s`
		$dir/cachebench -p -d${dur} -x${res} -e1 -m${mem} > $dir/results/mcpy4.1 &
		$dir/cachebench -p -d${dur} -x${res} -e1 -m${mem} > $dir/results/mcpy4.2 &
		$dir/cachebench -p -d${dur} -x${res} -e1 -m${mem} > $dir/results/mcpy4.3 &
		$dir/cachebench -p -d${dur} -x${res} -e1 -m${mem} > $dir/results/mcpy4.4 &
		wait
	fi 

done
