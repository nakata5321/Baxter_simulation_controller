#!/usr/bin/env python

import ipfshttpclient
import subprocess
import rospy
import time
import cv2
import os
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from threading import Thread

# write about activation
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

publish = Thread(target = listener)
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

#push to ipfs
rospy.loginfo("Push to IPFS")
client = ipfshttpclient.connect()
hash_result = []
for x in result_picture:
    while not rospy.is_shutdown():
        face_publisher.publish(face_msg)
        break
    res = client.add(x)
    hash_result.append(res.values()[0].encode('utf8'))
    rospy.loginfo("Pushed, the IPFS hash is " + res.values()[0].encode('utf8'))
res = client.add(dirname + '/' + "result.txt")
hash_result.append(res.values()[0].encode('utf8'))
rospy.loginfo("Pushed, the IPFS hash is " + res.values()[0].encode('utf8'))

#push to robonomics
rospy.loginfo("Push hash to robonomics")
for r in hash_result:
    print("echo \"" + r + "\" | " + '/home/$USER/robot_ws/' + "robonomics io write datalog -s " + my_private_key)
    program = "echo \"" + r + "\" | " + '/home/$USER/robot_ws/' + "robonomics io write datalog -s " + my_private_key
    process = subprocess.Popen(program, shell=True, stdout=subprocess.PIPE)
    rospy.sleep(4)
rospy.loginfo("Job finished")
