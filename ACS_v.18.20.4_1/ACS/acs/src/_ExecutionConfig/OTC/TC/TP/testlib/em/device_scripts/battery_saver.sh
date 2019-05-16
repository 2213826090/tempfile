sleep $1
input tap $2 $3
sleep 2
#dumpsys display | grep mActualBacklight=  > $4
dumpsys display | tee $4
