# Control Baxter robot with robonomics

Example of how it works is available on [YouTube.][db1]

## Requirements:

 - ROS Melodic + Gazebo (installation manual [here][db2])  
 - extra packages:
```sh
sudo apt-get install ros-melodic-qt-build ros-melodic-driver-common ros-melodic-gazebo-ros-control ros-melodic-gazebo-ros-pkgs ros-melodic-ros-control ros-melodic-control-toolbox ros-melodic-realtime-tools ros-melodic-ros-controllers ros-melodic-xacro python-wstool ros-melodic-tf-conversions ros-melodic-kdl-parser python-wstool python-catkin-tools qt4-default
```
- IPFS up to 0.6.0 (download from [here][db3] and install)
- python packages:
```sh
sudo apt-get -y install python3-pip
pip3 install --upgrade pip
```
 - Robonomics node (binary file) (download latest [release][db4] here)
 - Create __Baxter__ and __Employer__ accounts  on **Robonomics Portal**  
 (you can find tutorial ["Create an Account on Robonomics Portal"][db8] here).
 - IPFS browser extension (not necessary)

## 0. install CV Bridge extension for python3

 - Create catkin workspace
```shell
mkdir -p catkin_workspace/src
cd catkin_workspace
catkin init
```

 - Instruct catkin to set cmake variables. Use your current version of `python`. For me, it is `python3.6`:
```sh
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
catkin config --install
```

 - Clone cv_bridge src:
```shell
git clone https://github.com/ros-perception/vision_opencv.git src/vision_opencv
```

 - Find version of cv_bridge in your repository:
```shell
apt-cache show ros-melodic-cv-bridge | grep Version
    Version: 1.12.8-0xenial-20180416-143935-0800
```

 - Checkout right version in git repo. In our case it is 1.12.8:
```shell
cd src/vision_opencv/
git checkout 1.12.8
cd ../../
```

 - Build:
```shell
catkin build cv_bridge
```

 - Extend environment with new package:

```shell
source install/setup.bash --extend
``` 
 - Test:
```shell
$ python3
Python 3.6.9 (default, Jan 26 2021, 15:33:00) 
[GCC 8.4.0] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> from cv_bridge.boost.cv_bridge_boost import getCvType
>>>
```

## 1. Download simulation and controller packages
We will need to create 2 workspaces - one for main Baxter's packages and other for main control programme.
First workspace. It's main control programme. It will run under python3.

```sh
cd ~
mkdir -p robonomics_ws/src
cd robonomics_ws/src/
git clone https://github.com/nakata5321/Baxter_simulation_controller.git
cd Baxter_simulation_controller
pip3 install -r requirements.txt
```
Second workspace. There will be all Baxter's packages. Simulation is very old, so it could run only under python2.
```shell
cd ~
mkdir -p robot_ws/src
cd robot_ws/src/
wstool init .
wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter_simulator/master/baxter_simulator.rosinstall
wstool update
```
These packages were created for ROS indigo. We have to change some files to run them on ROS melodic.
We will use **patch** files.
```sh
patch ./baxter_simulator/baxter_sim_io/include/baxter_sim_io/qnode.hpp ~/robonomics_ws/src/Baxter_simulation_controller/patch/qnode_patch
patch ./baxter_simulator/baxter_sim_kinematics/src/arm_kinematics.cpp ~/robonomics_ws/src/Baxter_simulation_controller/patch/arm_patch
patch ./baxter_interface/src/baxter_interface/robot_enable.py ~/robonomics_ws/src/Baxter_simulation_controller/patch/interface_patch
```
And let's build  all our packages:  
First build Baxter's packages
```sh
cd ../
catkin build
```
Then return to first workspace and build it too:
```sh
cd ../
catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3
```
Don't forget to add source command:

```sh
echo "source /home/$USER/robot_ws/devel/setup.bash" >> ~/.bashrc
echo "source /home/$USER/robonomics_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```  


## 2. Start simulation
### Let's start our simulation:
At first go to `robot_ws` and copy and edit baxter.sh
```sh
cd ~/robot_ws/
cp src/baxter/baxter.sh .
```
Find your local ip address with command:
```
ip a
```
![ip_a][im14]

Edit the following values in `baxter.sh` :
```
nano baxter.sh
```

- your_ip - put your local ip address. See `ip a`
- ros_version - for example "melodic"

![baxtersh][im15]

Run the baxter shell script with sim specified:
```sh
./baxter.sh sim
roslaunch baxter_gazebo baxter_world.launch
```
You can put some models in front of our baxter. It will be more interesting.
![baxter][im2]

## 3.Manage accounts in DAPP

Since we are testing, let us create a local robonomics network with robonomics binary file. Go to folder with robonomics file and run:
```sh
./robonomics --dev --tmp
```
![robonomics][im3]

Go to [https://parachain.robonomics.network][db5] and switch to local node
![local node][im4]

Go to Accounts and create __Baxter__ and __Employer__ accounts.

You can find The manual "Create an Account on Robonomics Portal" [here][db8]

__Important!__ Copy each account's **Mnemonic** and **address** (to copy address click on account's icon). **Mnemonic** is the private key for account.

Transfer some money (units) to these accounts:

![create account][im5]
![create account2][im16]
![accounts][im6]

Add Baxter's **Mnemonic** and **address** to `config.yaml` in `robonomics_ws/src/Baxter_simulation_controller/config/`

## 4.Start simulation

In new window run:
```sh
ifps init #you only need to do this once
ipfs daemon
```
Open separate terminal and start *controller package*:
```sh
rosrun robot_controller robot_control.py
```
![waiting][im7]

Now you can send a transaction triggering the Baxter to start moving and collecting data. To do so, you can use the same portal [https://parachain.robonomics.network][db5]. Go to **Developer->Extrinsics** and select Baxter's employer account, `launch` extrinsic, Baxter's account as a target account and `yes` as a parameter. Submit the extrinsic.


![rob_message][im8]

The robot should start moving. It won't accept commands from other accounts neither commands with `no` parameter.
You should see the following:

![working][im9]

When the work is over go to the Robonomics Portal to `Developer > Chain state`. Choose *datalog.datalogItem(AccountId,u64)* in **state query**.If you want to show all datalog's, then turn off `include option` add view Baxter's datalog message using "+" button.

![datalog][im10]

Now the IPFS hash of the telemetry and photos is saved in the blockchain. To see the data simply copy the hash and insert it in the search bar with URL:  
#### gateway.ipfs.io/ipfs/< put your hash here>



That's all!

![result1][im12]
![result2][im13]

[db1]: <https://youtu.be/2AQGFVzkGdg>
[db2]: <http://wiki.ros.org/melodic/Installation>
[db3]: <https://dist.ipfs.io/go-ipfs/v0.6.0/go-ipfs_v0.6.0_linux-386.tar.gz>
[db4]: <https://github.com/airalab/robonomics/releases>
[im1]: <./docs/images/empty_world.jpg>
[im2]: <./docs/images/baxter_simulation.jpg>
[im3]: <./docs/images/robonomics.jpg>
[db5]: <https://parachain.robonomics.network>
[im4]: <./docs/images/local_node.jpg>
[im5]: <./docs/images/create_account.jpg>
[im6]: <./docs/images/accounts.jpg>
[im7]: <./docs/images/waiting.jpg>
[db6]: <https://wiki.robonomics.network/docs/rio-overview/>
[im8]: <./docs/images/rob_message.jpg>
[im9]: <./docs/images/working.jpg>
[im10]: <./docs/images/datalog.jpg>
[im11]: <./docs/images/ipfs.jpg>
[im12]: <./docs/images/result1.jpg>
[im13]: <./docs/images/result2.jpg>
[im14]: <./docs/images/ip_a.png>
[im15]: <./docs/images/baxter_sh.jpg>
[im16]: <./docs/images/create_account2.jpg>
[db8]: <https://wiki.robonomics.network/docs/create-account-in-dapp/>
