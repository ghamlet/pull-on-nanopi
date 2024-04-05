#!/usr/bin/env python
 
import socket
import NanoPi2openCM
from RoboticArmClass import RoboticArm
import publish_joint_state 

servos_name = ['ang_joint_1','ang_joint_2','ang_joint_3','ang_joint_4','ang_joint_5','gripper']
gripperPose = '0'
curJointState = [0,0,0,0,0]


def init_server(ip_center, port):

    SERVICE_CENTER = (ip_center, port)
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(SERVICE_CENTER) 
    server.listen()   
    print("Waiting for connection...")
    client, address = server.accept()

    print("Connection from", address)
    return client


def main_loop(client):
    while True:
        msg = ''
        while True:
            symbol = client.recv(1).decode()
            if symbol =='#': break  #stop bit
            msg += symbol
        print("I recive message: ",msg)    
        
        MoveToPointCallback(msg)


def ParseMsg(msg):
    try:
        coord_list = msg.split(':')
        print(coord_list)
        x = float(coord_list[0])
        y = float(coord_list[1])
        z = float(coord_list[2])
        pith = float(coord_list[3])
        roll = float(coord_list[4])
        return x,y,z,pith,roll
    except ValueError:
        pass


def MoveToPointCallback(msg):
    x,y,z,pitch,roll = ParseMsg(msg)
    roboticArm = RoboticArm()
    availJointState,goalJointState = roboticArm.InversProblem(x,y,z,pitch,roll)

    if (not availJointState):
        print('Point cannot be reached')
    else:
        goalJointState = [str(el) for el in goalJointState]
        strName = ' '.join(servos_name)
        strJS = ' '.join(goalJointState) + ' ' + gripperPose
        strCmd = strName + ' ' + strJS

        print(strCmd)
        print('Point can be reached')

        joint_Cmd = publish_joint_state.convert_pose(strCmd)
        NanoPi2openCM.send_data(joint_Cmd)



if __name__=='__main__':

    ip_addr, port = "127.0.0.1", 8000
    client = init_server(ip_addr, port)
    main_loop(client)

   
    

       