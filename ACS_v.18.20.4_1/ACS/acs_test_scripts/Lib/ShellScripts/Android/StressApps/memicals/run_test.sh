dir=`dirname $0`
duration=$1
number_of_tests=$2

chmod 777 $dir/memicals

if [ "$3" = "1" ]
then
    $dir/memicals -logPath=$dir -verbose=9 -changeTargets -changeDirection -algorithm=fastRandom -time=$duration
fi

if [ "$4" = "1" ]
then
    $dir/memicals -logPath=$dir -verbose=9 -changeTargets -changeStrideStart -algorithm=lfsr -time=$duration
fi

if [ "$5" = "1" ]
then
    $dir/memicals -logPath=$dir -verbose=9 -changeTargets -changeCritical -algorithm=randomAddr -time=$duration
fi

if [ "$6" = "1" ]
then
    $dir/memicals -logPath=$dir -verbose=9 -changeTargets -changeDirection -algorithm=switchAddr -time=$duration
fi

if [ "$7" = "1" ]
then
    $dir/memicals -logPath=$dir -verbose=9 -changeTargets -algorithm=walkingBits -lag=2 -time=$duration
fi

numCmdlines=$number_of_tests
numLogs=$(ls memi*log | wc -l)

if [ numCmdlines -ne numLogs ]
then
  echo "Too few log files; one or more instances did not execute." > fail.log
  ls memi*log | xargs cat >> fail.log
else
  var=0
  var=$(grep ERR memi*log | wc -l)
  if [ var -ne 0 ]
  then
    ls memi*log | xargs cat > fail.log
  else
    ls memi*log | xargs cat > pass.log
  fi
fi
rm -f memi*log
