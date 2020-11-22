# Control Baxter robot with robonomics

Example of how it works is available on [YouTube.][db1]

## Requirements:

 - ROS Melodic + Gazebo (installation manual [here][db2])  
 - extra packages:
```sh
sudo apt-get install ros-melodic-qt-build ros-melodic-driver-common ros-melodic-gazebo-ros-control ros-melodic-gazebo-ros-pkgs ros-melodic-ros-control ros-melodic-control-toolbox ros-melodic-realtime-tools ros-melodic-ros-controllers ros-melodic-xacro python-wstool ros-melodic-tf-conversions ros-melodic-kdl-parser python-wstool python-catkin-tools qt4-default
```
- IPFS 0.4.22 (download from [here][db3] and install)
- pip:
```sh
sudo apt install python-pip
```

- ipfshttpclient:
```sh
pip install ipfshttpclient
```
 - Robonomics node (binary file) (download latest [release][db4] here)
 - IPFS browser extension (not necessary)

## 1. Download simulation and controller packages
Download packages:
```sh
cd ~
mkdir -p robot_ws/src
cd robot_ws/src/
wstool init .
wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter_simulator/master/baxter_simulator.rosinstall
wstool update
git clone https://github.com/nakata5321/Baxter_simulation_controller.git
```
This packages were created for ROS indigo. We have to change some files to run them on ROS melodic.
We will use **patch** files.
```sh
patch ./baxter_simulator/baxter_sim_io/include/baxter_sim_io/qnode.hpp ./Baxter_simulation_controller/patch/qnode_patch
patch ./baxter_simulator/baxter_sim_kinematics/src/arm_kinematics.cpp ./Baxter_simulation_controller/patch/arm_patch
```
And let's build  all our packages:
```sh
cd ..
catkin build
```
Dont forget to add source command:
```sh
echo "source /home/$USER/robot_ws/devel/setup.bash" >> ~/.bashrc
```  
At the end save *Robonomics node (binary file)* in **robot_ws** directory.

## 2. Start simulation
### Let's start our simulation:
At first copy and edit baxter.sh
```sh
cp src/baxter/baxter.sh .
```
Edit the following values in `baxter.sh` :
- your_ip value - put your local ip address
- ros_version

Run the baxter shell script with sim specified:
```sh
./baxter.sh sim
roslaunch baxter_gazebo baxter_world.launch
```
You can put some models in front of our baxter. It will be more intresting.
![baxter][im2]

## 3.Manage accounts in DAPP

Since we are testing, let us create a local robonomics network with robonomics binary file. Go to folder with robonomics file and run:
```sh
./robonomics --dev --rpc-cors all
```
![robonomics][im3]

Don't forget to remove `db` folder after every launch:
```sh
rm -rf /home/$USER/.local/share/robonomics/chains/dev/db
```

Go to [https://parachain.robonomics.network][db5] and switch to local node
![local node][im4]

Go to Accounts and create __Baxter__ and __Employer__ accounts (__Robot__ is not necessary)

You can find The manual "Create an Account on Robonomics Portal" [here][db6]

__Important!__ Copy each account's key and address (to copy address click on account's icon).You should change value from **Mnemonic** to **Raw seed** in second raw. It will be the private key for account.
Transfer some money (units) to these accounts:

![create account][im5]
![accounts][im6]

Add Baxter's secret key and adress to `config.yaml` in `robot_ws/src/Baxter_simulation_controller/config/`

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

Return to the first terminal, open new window and send command to [**robonomics io**][db6]. This command will turn ON your robot:
```sh
echo "ON" | ./robonomics io write launch -r <BAXTER ADDRESS> -s <EMPLOYER’S KEY>
```
Where `<BAXTER ADDRESS>`  and `<EMPLOYER’S KEY>` are replaced with previously saved strings accordingly.

![rob_message][im8]

You should see the following:

![working][im9]

when the work is over go to the Robonomics Portal to `Developer > Chain state`. Choose *datalog* in **state query** and add Baxter datalog message using "+" button.

![datalog][im10]

Now the IPFS hash of the telemetry and photos is saved in the blockchain. To see the data simply copy the hash and insert it in the search bar with URL:  
#### gateway.ipfs.io/ipfs/<put your hash here>



That's all!

![result1][im12]
![result2][im13]

[db1]: <https://youtu.be/2AQGFVzkGdg>
[db2]: <http://wiki.ros.org/melodic/Installation>
[db3]: <https://dist.ipfs.io/go-ipfs/v0.4.22/go-ipfs_v0.4.22_linux-386.tar.gz>
[db4]: <https://github.com/airalab/robonomics/releases>
[im1]: <https://github.com/nakata5321/Baxter_simulation_controller/blob/master/docs/images/empty_world.jpg>
[im2]: <https://github.com/nakata5321/Baxter_simulation_controller/blob/master/docs/images/baxter_simulation.jpg>
[im3]: <https://github.com/nakata5321/Baxter_simulation_controller/blob/master/docs/images/robonomics.jpg>
[db5]: <https://parachain.robonomics.network>
[im4]: <https://github.com/nakata5321/Baxter_simulation_controller/blob/master/docs/images/local_node.jpg>
[im5]: <https://github.com/nakata5321/Baxter_simulation_controller/blob/master/docs/images/create_account.jpg>
[im6]: <https://github.com/nakata5321/Baxter_simulation_controller/blob/master/docs/images/accounts.jpg>
[im7]: <https://github.com/nakata5321/Baxter_simulation_controller/blob/master/docs/images/waiting.jpg>
[db6]: <https://wiki.robonomics.network/docs/rio-overview/>
[im8]: <https://github.com/nakata5321/Baxter_simulation_controller/blob/master/docs/images/rob_message.jpg>
[im9]: <https://github.com/nakata5321/Baxter_simulation_controller/blob/master/docs/images/working.jpg>
[im10]: <https://github.com/nakata5321/Baxter_simulation_controller/blob/master/docs/images/datalog.jpg>
[im11]: <https://github.com/nakata5321/Baxter_simulation_controller/blob/master/docs/images/ipfs.jpg>
[im12]: <https://github.com/nakata5321/Baxter_simulation_controller/blob/master/docs/images/result1.jpg>
[im13]: <https://github.com/nakata5321/Baxter_simulation_controller/blob/master/docs/images/result2.jpg>
[db6]: <https://wiki.robonomics.network/docs/create-accounts-in-dapp/>
