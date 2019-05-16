#!/bin/bash

unset http_proxy
IP=10.102.160.240

cookie=/tmp/outlet.cookie

function login() {
    challenge=`curl -s http://$IP/ | grep '"Challenge"' | sed 's/.*value=//' | tr -d '">'`
    pass=`echo -n $challenge"admin1234"$challenge | md5sum | cut -d' ' -f1`
    curl -s "http://$IP/login.tgi" --data-urlencode 'Username=admin' --data-urlencode "Password=$pass" -c $cookie 
}

function is_logged() {
    curl -s "http://$IP/" -b $cookie | grep Challenge 2>/dev/null
    if [ $? == 0 ]
    then
	return 1
    else
	return 0
    fi
}

function action() {
    action=$1
    is_logged
    if [ $? == "1" ]
    then
	login
    fi
    curl -s "http://$IP/outlet?$action" -b $cookie 
}

if [ "$#" != "2" ]
then
    echo "Usage: `basename $0` <number> <OFF|ON>"
    exit 1
fi

action $1=$2 >/dev/null
