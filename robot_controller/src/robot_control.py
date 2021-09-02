#!/usr/bin/env python3

# from cv_bridge.boost.cv_bridge_boost import getCvType

import ipfshttpclient
import typing as tp
import threading
# import subprocess
import rospy
import yaml
import time
import cv2
import os

from substrateinterface import SubstrateInterface, Keypair
from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from threading import Thread


def read_config(path: str) -> tp.Dict[str, str]:
    try:
        with open(path) as f:
            content = f.read()
            config = yaml.load(content)
            rospy.logdebug(f"Configuration dict: {content}")
            return config
    except Exception as e:
        while True:
            rospy.logerr("Configuration file is broken or not readable!")
            rospy.logerr(e)
            rospy.sleep(5)


def push_to_ipfs(result_picture: list, dirname: str) -> list:
    rospy.loginfo("Push to IPFS")
    client = ipfshttpclient.connect()
    hash_result = []
    for x in result_picture:
        res = client.add(x)
        hash_result.append(res.values()[0].encode('utf8'))
        rospy.loginfo("Pushed, the IPFS hash is " + res.values()[0].encode('utf8'))
    res = client.add(dirname + '/' + "result.txt")
    hash_result.append(res.values()[0].encode('utf8'))
    rospy.loginfo("Pushed, the IPFS hash is " + res.values()[0].encode('utf8'))
    return hash_result


def substrate_connection(url: str) -> tp.Any:
    """
    establish connection to a specified substrate node
    """
    try:
        rospy.loginfo("Establishing connection to substrate node")
        substrate = SubstrateInterface(
            url=url,
            ss58_format=32,
            type_registry_preset="substrate-node-template",
            type_registry={
                "types": {
                    "Record": "Vec<u8>",
                    "Parameter": "Bool",
                    "LaunchParameter": "Bool",
                    "<T as frame_system::Config>::AccountId": "AccountId",
                    "RingBufferItem": {
                        "type": "struct",
                        "type_mapping": [
                            ["timestamp", "Compact<u64>"],
                            ["payload", "Vec<u8>"],
                        ],
                    },
                    "RingBufferIndex": {
                        "type": "struct",
                        "type_mapping": [
                            ["start", "Compact<u64>"],
                            ["end", "Compact<u64>"],
                        ],
                    }
                }
            },
        )
        rospy.loginfo("Successfully established connection to substrate node")
        return substrate
    except Exception as e:
        rospy.logerr(f"Failed to connect to substrate: {e}")
        return None


def write_datalog(substrate, seed: str, data: str) -> str or None:
    """
    Write any string to datalog
    Parameters
    ----------
    substrate : substrate connection instance
    seed : mnemonic seed of account which writes datalog
    data : data to be stored as datalog
    Returns
    -------
    Hash of the datalog transaction
    """

    # create keypair
    try:
        keypair = Keypair.create_from_mnemonic(seed, ss58_format=32)
    except Exception as e:
        rospy.logerr(f"Failed to create keypair for recording datalog: \n{e}")
        return None

    try:
        rospy.loginfo("Creating substrate call for recording datalog")
        call = substrate.compose_call(
            call_module="Datalog",
            call_function="record",
            call_params={
                'record': data
            }
        )
        rospy.loginfo(f"Successfully created a call for recording datalog:\n{call}")
        rospy.loginfo("Creating extrinsic for recording datalog")
        extrinsic = substrate.create_signed_extrinsic(call=call, keypair=keypair)
    except Exception as e:
        rospy.logerr(f"Failed to create an extrinsic for recording datalog: {e}")
        return None

    try:
        rospy.loginfo("Submitting extrinsic for recording datalog")
        receipt = substrate.submit_extrinsic(extrinsic, wait_for_inclusion=True)
        rospy.loginfo(f"Extrinsic {receipt.extrinsic_hash} for recording datalog sent and included in block"
                      f" {receipt.extrinsic_hash}")
        return receipt.extrinsic_hash
    except Exception as e:
        rospy.logerr(f"Failed to submit extrinsic for recording datalog: {e}")
        return None


class Baxter:
    """
    Robot control class with activation and moving functions
    """

    def __init__(self):
        """
        init all constants and start working
        """
        # all constants
        self.hash = ""
        self.br = CvBridge()
        self.dirname = os.path.dirname(__file__)
        self.path_result = self.dirname + "/result.txt"
        self.result = []
        self.i = time.time()
        self.result_picture = []
        self.stop_publish = False
        self.happy_picture = self.dirname + "/happy_smile.jpg"
        self.done_picture = self.dirname + "/accept.png"
        self.publish = Thread(target=self.listener)
        self.result_file = open(self.path_result, "w")

        # parsing config
        self.config = read_config(self.dirname + "/../../config/config.yaml")
        self.baxter_address = self.config["baxter_address"]
        self.baxter_seed = self.config["baxter_seed"]
        self.employer_address = self.config["employer_address"]
        self.node_address = self.config["node_address"]

        rospy.loginfo("Activation")
        rospy.init_node('robot_control', anonymous=False)

        self.face_publisher = rospy.Publisher('/robot/xdisplay', Image, queue_size=1)
        self.sad_picture = self.dirname + "/sad_face.png"
        self.face = cv2.imread(self.sad_picture, 1)
        self.face_msg = self.br.cv2_to_imgmsg(self.face, "bgr8")
        self.face_publisher.publish(self.face_msg)
        rospy.loginfo("Activation complete. Ready for a job")

        rospy.loginfo('Initiating substrate connection for launch tracking and datalogs writing')
        self.substrate_launch = substrate_connection(self.node_address)
        self.substrate_datalog = substrate_connection(self.node_address)
        launch_tracker = LaunchTracker(self.substrate_launch, self.employer_address, self.baxter_address)

        rospy.loginfo('Waiting job command from employer, press Ctrl+\\ to interrupt')

        while True:
            launch_tracker.launch_command_event.wait()
            self.work()
            self.hash_result = push_to_ipfs(self.result_picture, self.dirname)
            for self.hash in self.hash_result:
                self.tr_hash = write_datalog(self.substrate_datalog, self.baxter_seed, self.hash)
            launch_tracker.launch_command_event.clear()
            rospy.loginfo("Published to chain! Transaction hash is " + self.tr_hash)
            rospy.loginfo("Job Done. Check DAPP for IPFS data hash")

    def work(self) -> None:
        rospy.loginfo("Start working")
        self.face = cv2.imread(self.happy_picture, 1)
        self.face_msg = self.br.cv2_to_imgmsg(self.face, "bgr8")
        self.face_publisher.publish(self.face_msg)
        self.publish.start()
        rospy.sleep(7)
        self.stop_publish = True
        self.publish.join()

        for f in self.result:
            self.result_file.write(f)
        self.result_file.close()

        self.face = cv2.imread(self.done_picture, 1)
        self.face_msg = self.br.cv2_to_imgmsg(self.face, "bgr8")
        self.face_publisher.publish(self.face_msg)

    def callback_head(self, data: Image) -> None:
        if not self.stop_publish:
            if time.time() - self.i > 2:
                self.path_pic = self.dirname + "/scrennshot" + str(int(self.i)) + ".png"
                self.result_picture.append(self.path_pic)
                self.image = self.br.imgmsg_to_cv2(data)
                cv2.imwrite(self.path_pic, self.image)
                self.i = time.time()

    def callback(self, data: LaserScan) -> None:
        self.result.append(str(data))

    def listener(self):
        self.rate = rospy.Rate(2)
        rospy.Subscriber('/cameras/head_camera/image', Image, self.callback_head)
        rospy.Subscriber('/sim/laserscan/left_hand_range/state', LaserScan, self.callback)
        while not rospy.is_shutdown():
            self.face_publisher.publish(self.face_msg)
            if self.stop_publish:
                break
            self.rate.sleep()


class LaunchTracker:
    """
    Parse new blocks in chain to search for Launch commands
    """

    def __init__(self, substrate: tp.Any, source_address: str, target_address: str):
        """
        Starting subscriber and creating Python event
        :param substrate: instance of substrate connection
        :param source_address: address from which tre launch transaction is supposed
        :param target_address: robot address
        """
        rospy.loginfo("Creating an instance of a LaunchTracker class")
        self.substrate = substrate
        self.employer_address = source_address
        self.robot_address: str = target_address

        rospy.loginfo(f"Initiating new blocks subscriber for launch commands tracking")
        self.launch_command_event = threading.Event()
        self.subscriber = threading.Thread(target=self._obtain_launch_commands)
        self.subscriber.start()

        rospy.loginfo("Block subscriber started. Waiting for launch commands")

    def _subscription_handler(self, obj, update_nr, subscription_id):
        """
        parse block events and trigger python Event on launch command sending to the robot address
        params info:
        https://github.com/polkascan/py-substrate-interface/blob/65f247f129016f5bb3a9a572579033f35fd385ea/substrateinterface/base.py#L2588
        """
        ch: str = self.substrate.get_chain_head()
        chain_events = self.substrate.get_events(ch)
        for ce in chain_events:
            if ce.value["event_id"] == "NewLaunch":
                print(ce.params)
            if ce.value["event_id"] == "NewLaunch" and ce.params[0]["value"] == self.employer_address \
                    and ce.params[1]["value"] == self.robot_address and ce.params[2]["value"] is True:  # yes/no
                rospy.loginfo(f"\"ON\" launch command from employer.")
                self.launch_command_event.set()  # trigger python Event in main loop

            elif ce.value["event_id"] == "NewLaunch" and ce.params[0]["value"] != self.employer_address \
                    and ce.params[1]["value"] == self.robot_address:
                rospy.loginfo(f"Launch command not from employer. Idle")

    def _obtain_launch_commands(self):
        """
        Subscribe to new block headers as soon as they are available. The callable `subscription_handler` will be
        executed when a new block is available and execution will block until `subscription_handler` will return
        a result other than `None`
        """

        self.substrate.subscribe_block_headers(self._subscription_handler)


if __name__ == '__main__':
    baxter = Baxter()


'''
rospy.init_node('robot_control', anonymous=False)
rospy.loginfo("Activation")
br = CvBridge()
dirname = os.path.dirname(__file__)


path = dirname + "/configuration.txt"
conf = open(path, 'r')
my_private_key = conf.readline()
my_adress = conf.readline()
conf.close()

face_publisher = rospy.Publisher('/robot/xdisplay', Image, queue_size=1)
sad_picture = dirname + "/sad_face.png"
face = cv2.imread(sad_picture, 1)
face_msg = br.cv2_to_imgmsg(face,  "bgr8")
rospy.loginfo("Activation complete. Ready for a job")
face_publisher.publish(face_msg)

# end of activation

# waiting for a job

program = '/home/$USER/robot_ws/' + 'robonomics io read launch'
rob_read = subprocess.Popen(program, shell=True, stdout=subprocess.PIPE)
while True:
    try:
        while not rospy.is_shutdown():
            face_publisher.publish(face_msg)
            break
        output = rob_read.stdout.readline()
        rospy.loginfo("Find a payment")
        out = output.split(" >> ")
        check_state = my_adress.rstrip() + " : true"
        if(out[1].rstrip() == check_state):
            rospy.loginfo("Job paid")
            rob_read.kill()
            break
        if(output):
            rospy.loginfo("Not my job")
    except KeyboardInterrupt:
        exit()


# start working

rospy.loginfo("Start working")
result = ""
happy_picture = dirname + "/happy_smile.jpg"
face = cv2.imread(happy_picture, 1)
face_msg = br.cv2_to_imgmsg(face,  "bgr8")
face_publisher.publish(face_msg)
i = time.time()
result_picture = []
global stop_publish
stop_publish = False


def callback_head(data):
    global i
    global result_picture
    global stop_publish
    if(not stop_publish):
        if(time.time() - i > 2):
            path = dirname + "/scrennshot" + str(int(i)) + ".png"
            result_picture.append(path)
            image = br.imgmsg_to_cv2(data)
            cv2.imwrite(path, image)
            i = time.time()


def callback(data):
    global result
    result = result + (str(data) + "\n")


def listener():
    global stop_publish
    rate = rospy.Rate(2)
    rospy.Subscriber('/cameras/head_camera/image', Image, callback_head)
    rospy.Subscriber('/sim/laserscan/left_hand_range/state', LaserScan, callback)

    while not rospy.is_shutdown():
        face_publisher.publish(face_msg)
        if stop_publish:
            break
        rate.sleep()    

publish = Thread(target=listener)
publish.start()
rospy.sleep(7)
stop_publish = True
publish.join()


try:
    path = dirname + "/result.txt"
    result_file = open(path, "w")
    for f in result:
        result_file.write(f)
finally:
    result_file.close()
rospy.loginfo("End of work")


done_picture = dirname + "/accept.png"
face = cv2.imread(done_picture, 1)
face_msg = br.cv2_to_imgmsg(face,  "bgr8")

# push to ipfs
rospy.loginfo("Push to IPFS")
client = ipfshttpclient.connect()
hash_result = []
for x in result_picture:
    while not rospy.is_shutdown():
        # face_publisher.publish(face_msg)
        break
    res = client.add(x)
    hash_result.append(res.values()[0].encode('utf8'))
    rospy.loginfo("Pushed, the IPFS hash is " + res.values()[0].encode('utf8'))
res = client.add(dirname + '/' + "result.txt")
hash_result.append(res.values()[0].encode('utf8'))
rospy.loginfo("Pushed, the IPFS hash is " + res.values()[0].encode('utf8'))
    
# push to robonomics
rospy.loginfo("Push hash to robonomics")
for r in hash_result:
    print("echo \"" + r + "\" | " + '/home/$USER/robot_ws/' + "robonomics io write datalog -s " + my_private_key)
    program = "echo \"" + r + "\" | " + '/home/$USER/robot_ws/' + "robonomics io write datalog -s " + my_private_key
    process = subprocess.Popen(program, shell=True, stdout=subprocess.PIPE)
    rospy.sleep(4)
rospy.loginfo("Job finished")
'''
