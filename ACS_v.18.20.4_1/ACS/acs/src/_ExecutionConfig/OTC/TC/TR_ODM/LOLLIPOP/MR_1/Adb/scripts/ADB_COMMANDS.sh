#!/bin/bash
adb install -r $HOME/.acs/Artifacts/APPLICATIONS/ChessARM.apk
if [ $? -eq 1 ]; then
    echo "Success"
else
    echo "FAIL"
fi