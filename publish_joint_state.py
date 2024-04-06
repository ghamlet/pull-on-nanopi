#!/usr/bin/env python
import math

fromEncToRadFor1066428 = 4096/(2*math.pi)
fromRncToRadFor12 = (1024*180)/(300*math.pi)
minEncPoseForGripper = 390
fromEncToLinGripper = 7.75
zeroPose = [2048, 2048, 2048, 2048, 512, 680]

currentState = {
        'name': [],
        'position':[]
    }
currentState['position'] = [0,0,0,0,0,0]
countOfJoint = 6



def parse_msg(msg):
    jointState = {
        "name": [],
        "position":[]
    }

    msgList = msg.split()
    jointState['name'] = msgList[0:countOfJoint]
    jointState['position'] = msgList[countOfJoint:]
    jointState['position'] = [float(el) for el in jointState['position']] 
    
    return jointState


def convert_pose(msg):
    pose_cmd = ''

    msg = parse_msg(msg)
    name = msg['name']
    poseList = msg['position']

    jointcmd = {
        'position': [] }

    jointcmd['name'] = name
    poseListPub = []
   

    for i in range(len(name)):
        if(name[i] == 'ang_joint_5'): 
            poseListPub.append(int((poseList[i]*fromRncToRadFor12)+zeroPose[i]))

        elif(name[i] == 'gripper'):
            poseListPub.append(int(poseList[i]*fromEncToLinGripper + minEncPoseForGripper))
            
        else:
            poseListPub.append(int((poseList[i]*fromEncToRadFor1066428)+zeroPose[i]))
           

    jointcmd['position'] = poseListPub
    print(jointcmd)
    
    pose_cmd = 'g:' + ':'.join(str(pose) for pose in poseListPub) + "#"
    return pose_cmd    

