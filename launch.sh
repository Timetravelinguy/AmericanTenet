#!/bin/bash
#Run the command below to give your terminal read and write permissions so that it can run properly
# chmod +x launch.sh
# Open QGroundControl in current terminal
cd ~/Downloads
./QGroundControl-x86_64.AppImage &

# Give QGC a second to open
sleep 2

# Open PX4 SITL in a new tab
gnome-terminal --tab -- bash -c "
cd ~/PX4-Autopilot;
make px4_sitl gz_rc_cessna;
exec bash"

# Open VSCode in another tab
gnome-terminal --tab -- bash -c "
cd ~/PX4-Autopilot;
code .;
exec bash"

# Opens plot juggler
gnome-terminal --tab -- bash -c "
ros2 run plotjuggler plotjuggler
exec bash"