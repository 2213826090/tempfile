#!/bin/bash
name=""
email=""
gitStatus=""
commitName=""
mailList=".mail_list"
nameList=""
mailTxt=""
workSpace=`pwd | awk -F "/" '{print $(NF)}'`

ECHO_PREFIX_INFO="\033[1;32;40mINFO\033[0;0m"
ECHO_PREFIX_ERROR="\033[1;31;40mError\033[0;0m"

function gitConfig(){
        [[ ! -f ~/.gitconfig ]] && touch ~/.gitconfig
        git config --global credential.helper "store"
        name=`cat ~/.gitconfig | grep -Po '(?<=name \= ).*'`
        email=`cat ~/.gitconfig | grep -Po '(?<=email \= ).*'`
        if [[ ! -n $name ]] || [[ ! -n $email ]] ; then
                echo -e $ECHO_PREFIX_INFO "your name:"
                read name
                echo -e $ECHO_PREFIX_INFO "your email:"
                read email
                git config --global user.name "$name"
                git config --global user.email "$email"
        fi
        [[ ! -n $name ]] || [[ ! -n $email ]] && echo -e $ECHO_PREFIX_ERROR "unknow name or email!!" && exit 1
}

function commitChange(){
        gitStatus=`git status | grep -i "nothing" | grep "clean"`
        if [[ ! -n $gitStatus ]] ; then
                echo -e $ECHO_PREFIX_INFO "Please name your change:"
                read commitName
                git add -A
                git commit -am "$commitName"
                sendMail #send mail after commit successfully
                git push
        else
                echo -e $ECHO_PREFIX_INFO "nothing has changed, your work space is clean"
        fi
}

function sendMail(){
        mailTxt=`git log  | head -n 5`
        nameList=`cat .mail_list   | awk '{print $(NF)}' | grep -Po '(?<=\<).*?(?=\>)'`
        for line in `echo $nameList` ; do
                git log | head -n 5 | mail -s "[vpg_git][${name}] committed a change to ${workSpace}" $line
        done
}


gitConfig
commitChange
