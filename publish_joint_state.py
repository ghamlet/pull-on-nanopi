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



def parse_msg(msg: str):
    jointState = {
        "name": [],
        "position":[]
    }

    msgList = msg.split()
    jointState['name'] = msgList[0:countOfJoint]
    jointState['position'] = msgList[countOfJoint:]
    jointState['position'] = [float(el) for el in jointState['position']] 
    
    return jointState


def convert_pose(msg: str):

    msg = parse_msg(msg)
    name = msg['name']
    poseList = msg['position']

    jointcmd = {
        'position': [] }

    jointcmd['name'] = name
    poseListPub = []
   

    for i in range(len(name)):
        if(name[i] == 'ang_joint_5'): 
            poseListPub.append(round((poseList[i]*fromRncToRadFor12)+zeroPose[i]))

        elif(name[i] == 'gripper'):
            poseListPub.append(round(poseList[i]*fromEncToLinGripper + minEncPoseForGripper))
            
        else:
            poseListPub.append(round((poseList[i]*fromEncToRadFor1066428)+zeroPose[i]))
           

    jointcmd['position'] = poseListPub
    print(jointcmd)

    return jointcmd    

