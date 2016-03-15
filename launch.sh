#!/bin/bash

 
# Copyright (C) 2015  TELECOM Nancy
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.


# See https://github.com/KonradIT/goprowifihack/blob/master/HERO4.md
# for available GoPro command

# On signal, stop video of gopro
trap "{ curl http://10.5.5.9/gp/gpControl/command/shutter?p=0 ; exit 0; }" SIGINT SIGTERM


#set gopro settings
#video mode
curl "http://10.5.5.9/gp/gpControl/command/mode?p=0"
curl "http://10.5.5.9/gp/gpControl/setting/68/0"
#30fps
curl "http://10.5.5.9/gp/gpControl/setting/3/8"
#1080p wide
curl "http://10.5.5.9/gp/gpControl/setting/2/8"

#trigger video
curl "http://10.5.5.9/gp/gpControl/command/shutter?p=1" &

#launch ros and pass arguments
roslaunch twizy save.launch $@

