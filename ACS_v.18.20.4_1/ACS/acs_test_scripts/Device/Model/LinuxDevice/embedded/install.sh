#!/bin/sh

[ -z $(uname -r | grep "edison") ] && { \
   USER="root";\
   ADDR=$1;\
   ADDR=${ADDR:-"192.168.2.15"};\
   cd $(dirname $0);\
   DIRNAME=$(basename $(pwd));\
   echo $DIRNAME;\
   echo "remote install => use ssh connection (user=$USER, address=$ADDR)";\
   chmod a+x $0;\
   echo "copying $DIRNAME to /tmp/$DIRNAME";\
   $(which scp) -r ../$DIRNAME $USER@$ADDR:/tmp/;\
   echo "remote executing /tmp/$DIRNAME/install.sh";\
   $(which ssh) $USER@$ADDR "cd /tmp/$DIRNAME && ./install.sh; cd ../; rm -r /tmp/$DIRNAME";\
} || {\
   echo "local install";\
   cd $(dirname $0)
   mkdir -p /usr/local/bin;\
   mv command_server.service /etc/systemd/system/;\
   cp -r ./* /usr/local/bin/;\
   echo -n "relaunching systemd... "
   $(which systemctl) daemon-reload && echo done;\
   echo -n "relaunching command server... "
   $(which systemctl) restart command_server && echo done;}


