#!/bin/sh

declare OLD_VER=$( grep CURR_VER= install.sh | cut -d '=' -f 2 )
declare NEW_VER=$1

sed -i '' "s/$OLD_VER/$NEW_VER/" install.sh dkms.conf
