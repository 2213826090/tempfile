#! /bash/sh
event_num=$1
ave_x=$2
ave_y=$3
scale_x=$4
scale_y=$5
fingers=$6

sendevent /dev/input/event${event_num} 3 47 0
sendevent /dev/input/event${event_num} 3 57 0
#sendevent /dev/input/event${event_num} 3 57 0
finger1_x=$((${ave_x}*${scale_x}))
finger1_y=$((${ave_y}*${scale_y}))
sendevent /dev/input/event${event_num} 3 53 ${finger1_x}
sendevent /dev/input/event${event_num} 3 54 ${finger1_y}

#touch
sendevent /dev/input/event${event_num} 1 330 1
sendevent /dev/input/event${event_num} 0 0 0
sleep 5
#add finger
i=1
x=${ave_x}
y=${ave_y}
while [ $i -lt ${fingers} ]
do 
    echo $i
    sendevent /dev/input/event${event_num} 3 47 $i
    sendevent /dev/input/event${event_num} 3 57 $i
    #sendevent /dev/input/${event_num} 0 0 0

    x=$(($x+50))
    y=$(($y+50))
    x1=$(($x*(${scale_x})))
    y1=$(($y*(${scale_y})))

    echo $x, $y, $x1, $y1
    sendevent /dev/input/event${event_num} 3 53 $x1
    sendevent /dev/input/event${event_num} 3 54 $y1
    sendevent /dev/input/event${event_num} 0 0 0
    i=$(($i+1))
done
sleep 5
#untouch
#sendevent /dev/input/event${event_num} 3 57 4294967295
#sendevent /dev/input/event${event_num} 3 47 0
#sendevent /dev/input/event${event_num} 3 57 0
#sendevent /dev/input/event${event_num} 0 0 0
#sendevent /dev/input/event${event_num} 1 330 0
#sendevent /dev/input/event${event_num} 0 0 0
