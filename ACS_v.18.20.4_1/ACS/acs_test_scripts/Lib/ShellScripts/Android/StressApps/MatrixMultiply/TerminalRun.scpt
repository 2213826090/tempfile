-- $Id: TerminalRun.scpt,v 1.1 2005/10/19 23:37:49 tfcreela Exp $
-- Wrapper to execute commands in new Terminal under OS X

on run argv
ignoring application responses
   tell application "Terminal"
      activate
	do script with command Â
	item 1 of argv
   end tell
end ignoring
end run
