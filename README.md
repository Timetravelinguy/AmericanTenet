<p align="center">
  <strong><font size="6">FlightLink</font></strong><br>
  <font size="4">American Tenet</font>
</p>

<h4 align="center">
  <a href="https://americantenet.com/">
    <img src="https://img.shields.io/badge/website-americantenet.com-blue?style=fla" alt="continuous integration" style="height: 20px;">
  </a>
</h4>



<p align="center">
    <img src="https://i0.wp.com/americantenet.com/wp-content/uploads/2024/08/cropped-AT_Logo-inverted.jpg?w=1301&ssl=1"/>
</p>


## Introduction

**American Tenet** is a project focused on building innovative tools and workflows for autonomous drone development and simulation. Our work integrates open source technologies such as **PX4** and **QGroundControl** to enable robust flight testing, mission planning, and real-time telemetry streaming.

This repository serves as a hub for everything we've built so far — from simulation setups to companion tools and scripts that enhance the PX4 development pipeline.

FlightLink is a real time telemetry monotoring and data visualization tool deisgned for PX4 based drones. 
We use this tool to decode and sort **Mavlink** messages into different flight phase data. The data is then displayed in real time on a graph using **plotjuggler** showing relevant flight phase data during the mission. 
## Usage 

To get started with setting up your virtual environment, first you need to decide wether you are going to use [WSL](https://learn.microsoft.com/en-us/windows/wsl/install) or a computer running [Ubuntu Linux](https://ubuntu.com/).

This will determine how your setup process looks like but will also affect your development stage because there is more documentation on Linux than for WSL. 


> **Note**
> We reccomend If you want to use [plotjuggler using ROS2 topics](https://docs.px4.io/main/en/debug/plotting_realtime_uorb_data.html) you need to use Ubuntu 22.04 and NOT 24.04. 



<details>
<summary>
  Tutorials
</summary> <br />

- [How to install WSL](https://www.google.com/url?sa=t&source=web&rct=j&opi=89978449&url=https://www.youtube.com/watch%3Fv%3Dwz0QBNy9i7w&ved=2ahUKEwiYxYbsoICOAxX-48kDHTBHLTAQwqsBegQIJhAG&usg=AOvVaw0MArqOTHan-iu-6JF5DHvA)
- [How to dual-boot Linux](https://www.google.com/url?sa=t&source=web&rct=j&opi=89978449&url=https://www.youtube.com/watch%3Fv%3DGXxTxBPKecQ&ved=2ahUKEwiEvImBoYCOAxXx4MkDHYByBE4QwqsBegQIFhAF&usg=AOvVaw3GWCywFtSxQoZC3Ex_BnXg)
</details>

### Development 

We are going to set up a Developer Environment (Toolchain). <br> 
The supported platforms for PX4 development are:
###
-    Ubuntu Linux (22.04/20.04/18.04) — Recommended
-    Windows (10/11) — via WSL2
-   Mac OS





## Ubuntu Development Environment
The following instructions use a bash script to set up the PX4 development environment on the Ubuntu Linux LTS versions supported by PX4: Ubuntu 24.04 (Nimble Numbat) and Ubuntu 22.04 (Jammy Jellyfish). <br> 
##
To install the toolchain: <br>
###
 ```shell
 cd ~  
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
``` 
###
What you installed is the PX4-Autopilot source code. Next you will run a script to install everything needed to build the source code. 
###

```shell
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```
- Acknowledge any prompts as they come up. <br>

- Restart your computer upon completion. 


 Open up the command prompt to build your first `PX4` instance 
```
cd ~/PX4-Autopilot
make px4_sitl
```
### Next Steps
1. Install [VSCode](https://docs.px4.io/main/en/dev_setup/vscode.html) (if you prefer using an IDE to the command line) 
2. Download the [QGroundControl](https://d176tv9ibo4jno.cloudfront.net/builds/master/QGroundControl-x86_64.AppImage). <br>

###
Before installing QGroundControl for the first time: <br>
3. On the command prompt enter: <br>

```
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y  
```
4. Logout and login again to enable the changes to user permissions.

5. Install (and run) using the terminal commands. 

``` 
chmod +x ./QGroundControl-x86_64.AppImage
./QGroundControl-x86_64.AppImage  (or double click)
```
### Flow
You have now installed the source code to make virtual drones yay pat yourself on the back. 
You have also downloaded QGroundControl and VSCode. 

Now everytime you start a new session you will need to 

1. Make an instance of a virtual fixed wing plane
```
cd ~/PX4_Autopilot/
make px4_sitl gz_rc_cessna 
```
2. Open up QGroundControl
```
/path/to/./QGroundControl
```
3. Open VSCode. <br>
What I like to do is open up VSCode from the terminal while in whatever directory I need to edit. 
```
cd path/to/whatever/
code . 
```
This will open up VSCode in whatever directory you are in. 

This is the most basic setup, but dont worry it gets more complicated when you start doing harder stuff lol. 

## Windows Subsystem Linux 

The Windows Subsystem for Linux (WSL2) allows users to install and run the Ubuntu Development Environment on Windows, almost as though we were running it on a Linux computer. <br>
The following instructions explain how to set up a PX4 development environment on Windows 10 or 11, running on Ubuntu Linux within WSL2.

With this environment developers can:

- Build any simulator or hardware target supported by Ubuntu Development Environment in the WSL Shell. (Ubuntu is the best supported and tested PX4 development platform).
- Debug code in Visual Studio Code running on Windows.
Monitor a simulation using QGroundControl for Linux running in WSL. QGC for Linux connects automatically to the simulation.
>**Note**: Connecting to a USB device from within WSL is not supported, so you can't update firmware using the upload option when building on the command line, or from QGroundControl for Linux.

### Installation

Install WSL2

To install WSL2 with Ubuntu on a new installation of Windows 10 or 11:

1. Make sure your computer your computer's virtualization feature is enabled in the BIOS. It's usually referred as "Virtualization Technology", "Intel VT-x" or "AMD-V" respectively

2. Open cmd.exe as administrator. This can be done by pressing the start key, typing cmd, right-clicking on the Command prompt entry and selecting Run as administrator.

3. Execute the following commands to install WSL2 and a particular Ubuntu version:

- Default version (22.04) 
```
wsl --install
```
- Ubuntu 20.04 (Gazebo-Classic Simulation) 
```shell
wsl --install -d Ubuntu-20.04
```
>**Note**: You can also install Ubuntu 20.04 and Ubuntu 22.04 from the store, which allows you to delete the application using the normal Windows Add/Remove settings:

4. WSL will prompt for a username and password for the Ubuntu installation. Record these credentials as you will need them later on. 
###
The command Prompt is now a terminal within the newly install Ubuntu environment. 

### Opening a WSL Shell 
All operations to install and build `PX4 `must be done within a `WSL Shell` (you can use the same shell that was used to install `WSL2` or open a new one).

If you're using `Windows Terminal` you can open a shell into an installed `WSL` environment as shown, and exit it by closing the tab.

<p align="center">
    <img src="https://docs.px4.io/main/assets/wsl_windows_terminal.qIuwqDTO.png"/>
</p>

To open a WSL shell using a command prompt:

Open a command prompt:

1. Press the Windows Start key.
2. Type cmd and press Enter to open the prompt
3. To start WSL and access the WSL shell, execute the command:
```
wsl -d <distribution_name>
```
For example:
```
wsl -d Ubuntu
```
or
```
wsl -d Ubuntu-20.04
```

>**Note**: If you only have one version of Ubuntu, you can just use wsl.

Enter the following commands to first close the WSL shell, and then shut down WSL:
```
exit
wsl -d <distribution_name> --shutdown
```
Alternatively, after entering exit you can just close the prompt.

### Install PX4 Toolchain 
Next we download the PX4 source code within the WSL2 environment, and use the normal Ubuntu installer script to set up the developer environment. This will install the toolchain for Gazebo Classic simulation and Pixhawk/NuttX hardware.

To install the development toolchain:
1. Open a WSL2 Shell (if it is still open you can use the same one that was used to install WSL2).
2. Execute the command `cd ~` to switch to the home folder of WSL for the next steps.
> **Warning**: This is important! If you work from a location outside of the WSL file system you'll run into issues such as very slow execution and access right/permission errors.
3. Downlaod the PX4 source code using `git` (which is already installed in WSL2)
```shell
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```
4. Run the **ubuntu.sh** installer script and acknowledge any prompts as the script progresses: 
``` shell
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```
5. Restart the "WSL Computer" after the script completes (exit the shell, shutdown WSL, and restart WSL) 
```shell
exit
wsl --shutdown
wsl
```
6. Switch to the PX4 repository in the WSL home folder: 
``` 
cd ~/PX4-Autopilot 
```
7. Build the PX4 Repository in the WSL home folder:
```
make px4_sitl
```
for more build options see [here](https://docs.px4.io/main/en/dev_setup/building_px4.html). 

## Visual Studio Integration
VS Code running on Windows is well integrated with WSL. <br> 
To set up the integration:
1. Download and install VIsual Studio Code on windows. 

2. Open VS Code. 

3. Install the extension called Remote- WSL
4. Open a WSL Shell
5. In the WSL Shell, switch to the PX4 Folder: 
```
cd ~/PX4-Autopilot
```
6. In the WSL Shell, start VS Code:
```
code .
```
This will open the IDE fully integrated with the WSL shell. <br>
Make sure you always open the PX4 repository in the remote WSL mode. <br>

7. Next time you want to develop WSL2 you can very easily open it again in Remote WSL mode by selection **Open Recent**. This will start WSL for you. 
<p align="center">
    <img src="https://docs.px4.io/main/assets/vscode_wsl.DUUO0DI2.png"/>
</p>

> **Note**: however that the IP address of the WSL virtual machine will have changed, so you won't be able to monitor simulation from QGC for Windows (you can still monitor using QGC for Linux)

### QGroundControl in WSL
The easiest way to set up and use QGroundControl is to donwload the Lnux version into WSL. <br>

You can do this from within the WSL shell. 
1. Right click [here](https://d176tv9ibo4jno.cloudfront.net/builds/master/QGroundControl-x86_64.AppImage) and copy the link to the download. 

2. Open a WSL shell and enter the following commands to download the appimage and make it executable (replace the AppImage URL where indicated):
``` shell
cd ~
wget <the_copied_AppImage_URL>
chmod +x QgroundControl.AppImage
```
3. Run QGroundControl 
```shell
./QGroundControl.AppImage
```
QGroundControl will launch and automatically connect to a running simulation and allow you to monitor and control your vehicle(s).

You will not be able to use it to install PX4 firmware because WSL does not allow access to serial devices.

## QGroundControl on Windows
Install [QGroundControl on Windows](https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-installer.exe) if you want to be able to update hardware with firmware created within PX4. 

These steps descibe how you can connect to the simulation running in the WSL: 
1. Open a WSL Shell

2. Check the IP adress of the WSL virtual machine by running the command `ip addr | grep eth0` : 
```shell
ip addr | grep eth0

6: eth0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc mq state UP group default qlen 1000
    inet 172.18.46.131/20 brd 172.18.47.255 scope global eth0
```
copy the first part of the `eth0` interface `inet` address to the clipboard In this case: `172.18.46.131` 

3. In QGC go to Q > Application settings > Comms Links

4. Add a UDP Link called "WSL" to port 18570 lof the IP address copied above
5. Save it and connect to it. 
>**Note**: You will have to uodate the WSL comm link in QGC every time WSL restarts (because it gets a dynamic IP address).


# Networking 
- how it works 
# MAVLink Router Installation
QGC and scripts at the same time wow
- modify qgc
- main.conf
- px shell

# Plot Juggler 
How to install plotjuggler 
- ros2 
- mavlink router
## Resources

- **[Website](https://amplication.com)** overview of the product.
- **[Docs](https://docs.amplication.com)** for comprehensive documentation.
- **[Blog](https://amplication.com/blog)** for guides and technical comparisons.
- **[Discord](https://amplication.com/discord)** for support and discussions with the community and the team.
- **[GitHub](https://github.com/amplication/amplication)** for source code, project board, issues, and pull requests.
- **[Twitter](https://twitter.com/amplication)** for the latest updates on the product and published blogs.
- **[YouTube](https://www.youtube.com/c/Amplicationcom)** for guides and technical talks.


## License

A large part of this project is licensed under the [Apache 2.0](./LICENSE) license. The only exception are the components under the `ee` (enterprise edition) directory, these are licensed under the [Amplication Enterprise Edition](./ee/LICENSE) license.
