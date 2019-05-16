#! /bash/sh
event_num=$1
finger1_x=$2
finger1_y=$3


sendevent /dev/input/event${event_num} 3 47 0
sendevent /dev/input/event${event_num} 3 57 0

sendevent /dev/input/event${event_num} 3 53 ${finger1_x}
sendevent /dev/input/event${event_num} 3 54 ${finger1_y}

#touch
sendevent /dev/input/event${event_num} 1 330 1
sendevent /dev/input/event${event_num} 0 0 0
#sleep 1
#add finger


#untouch
sendevent /dev/input/event${event_num} 3 57 4294967295
sendevent /dev/input/event${event_num} 3 47 0
sendevent /dev/input/event${event_num} 3 57 0
sendevent /dev/input/event${event_num} 1 330 0
sendevent /dev/input/event${event_num} 0 0 0
