#!/system/bin/sh

#Killing the run_em scripts
pawn=`ps -e|grep run_em |awk '{print $1}'`
kill -TERM $pawn 2> /dev/null

#Killing the cachebench processes which still active
bunuh=`ps -e|grep cachebench |awk '{print $1}'`
for i in $bunuh
do
    kill -TERM $i 2> /dev/null
    echo "cachebench processes killed"
done

#Killing the memorytest.sh process
bunuh=`ps -e|grep memorytest.sh |awk '{print $1}'`
    kill -TERM $bunuh 2> /dev/null

bunuh=`ps -e|grep "tar" |awk '{print $1}'`
for i in $bunuh
do
    kill -TERM $i 2> /dev/null
    echo "tar processes killed"
done

bunuh=`ps -e|grep "gzip" |awk '{print $1}'`
for i in $bunuh
do
    kill -TERM $i 2> /dev/null
    echo "gzip processes killed"
done

bunuh=`ps -e|grep "diff" |awk '{print $1}'`
for i in $bunuh
do
    kill -TERM $i 2> /dev/null
    echo "diff processes killed"
done

echo "All previous cacheMem related processes killed."
