# Control Baxter robot with robonomics

Example of how it works is available on [YouTube.][db1]

## Requirements:

 - ROS Melodic + Gazebo (installation manual [here][db2])  
 - extra packages:
```sh
sudo apt-get install ros-melodic-gazebo-ros-control ros-melodic-effort-controllers ros-melodic-joint-state-controller python3-rospkg rosbash
```
- IPFS up to 0.6.0 (download from [here][db3] and install)
- python packages:
```sh
pip3 install -r requirements.txt
```
 - Robonomics node (binary file) (download latest [release][db4] here)
 - IPFS browser extension (not necessary)

## 1. Download simulation and controller packages
Download packages:
```sh
cd ~
mkdir -p robot_ws/src
cd robot_ws/src
git clone https://github.com/nakata5321/Baxter_simulation_controller.git
cd Baxter_simulation_controller
git checkout old_version
cd ../..
catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3
```
Dont forget to add source command:
```sh
echo "source /home/$USER/robot_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 2. Start simulation
Let's start gazebo world and put our baxter in it:
```sh
roslaunch gazebo_ros empty_world.launch
```
![empty world][im1]

Open one more window in terminal:
```sh
rosrun gazebo_ros spawn_model -file `rospack find baxter_description`/urdf/baxter.urdf -urdf -z 1 -model baxter
```
You can put some models in front of our baxter. It will be more intresting.

![baxter][im2]

## 3.Manage accounts in DAPP

Since we are testing, let us create a local robonomics network with robonomics binary file. Go to folder with robonomics file and run:
```sh
./robonomics --dev --tmp --rpc-cors all
```
![robonomics][im3]


Go to [https://parachain.robonomics.network][db5] and switch to local node

![local node][im4]

Go to Accounts and create __Baxter__ and __Employer__ accounts (__Robot__ is not necessary)

__Important!__ Copy each account's key and address (to copy address click on account's icon).You should change value from **Mnemonic** to **Raw seed** in second raw. It will be the private key for account.
Transfer some money (units) to these accounts:

![create account][im5]
![accounts][im6]

Add Baxter's secret key and adress to `config.yaml` in `robot_ws/src/robot_controller/config/`

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
Where *<BAXTER ADDRESS>* and *<EMPLOYER’S KEY>* are replaced with previously saved strings accordingly

![rob_message][im8]

You should see the following:

![working][im9]

when the work is over go to the Robonomics Portal to `Developer > Chain state`. Choose *datalog* in **state query** and add Baxter datalog message using "+" button.

![datalog][im10]

Now the IPFS hash of the telemetry and photos is saved in the blockchain. To see the data simply copy the hash and insert it in IPFS Companion:

![ipfs][im11]

Click  __View on Gateway__ and that's all!

![result1][im12]
![result2][im13]

[db1]: <https://youtu.be/AeufQmaNRWk>
[db2]: <http://wiki.ros.org/melodic/Installation>
[db3]: <https://dist.ipfs.io/go-ipfs/v0.6.0/go-ipfs_v0.6.0_linux-386.tar.gz>
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
