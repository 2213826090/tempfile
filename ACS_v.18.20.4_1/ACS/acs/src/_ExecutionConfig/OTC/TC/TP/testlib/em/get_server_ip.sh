#!/usr/bin/expect
set HOST [lindex $argv 0]
set USER [lindex $argv 1]
set PASSWD [lindex $argv 2]
set timeout 10
spawn ssh $USER@$HOST ifconfig
expect {
    "*yes/no" {
        send "yes\r";
        exp_continue
    }
    "*password:" {
        send "$PASSWD\r"
        expect eof {exit}
    }
}

