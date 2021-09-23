import rospy 
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from arm_node.srv import *
from ws4py.client.threadedclient import WebSocketClient
import json
import time


class BasicClient(WebSocketClient):
    def opened(self):
        self.operation_mode = None
        self.responded = True
        # self.arm_pose = None

        privilege_request = ['request-privilege', 
                                {'privilege' : 'change-action-command',
                                 'priority' : 0}]
        self.send(json.dumps(privilege_request))


    def closed(self, code, reason):
        print(("Closed", code, reason))


    def received_message(self, m):
        dataloaded = json.loads(m.data)
        message_type = str(dataloaded[0])
        message_dict = dataloaded[1]

        if message_type == 'privileges':
            self.done = message_dict['privileges'][0]['has']
            if self.done:
                print("Privilege request executed successfully.")

        elif message_type == 'robot-status':
            self.responded = True
            self.operation_mode = str(message_dict['operation-mode'])

        elif message_type == 'error':
            self.error_info = str(message_dict['info'])
            print('Error: ', self.error_info)

        elif message_type == 'action-status-changed':
            if message_dict['status'] == 'running':
                # print('Command received and is running')
                self.completed = False

            elif message_dict['status'] == 'success':
                # print('Command finished successfully executing. ', str(message_dict['info']))
                self.completed = True
        elif message_type == 'object-kinematics':  
            self.arm_pose = message_dict['transform']['rpyxyz'] 



class Digit_Control:
    def __init__(self, client):
        self.client = client 
        self.x_offset = 0.06
        self.z_offset = 0.16
        self.none = 999
        self.vertical_confs = [150, 150]
        self.armname = {'right':'right-hand', 'left':'left-hand'}
        self.arm_id = {'right':0, 'left':1}

        rospy.init_node("digit_control")

    def send_wrist_msg(self, pose, dur, armname):
        arm = self.armname[armname]
        msg = ["action-end-effector-move",
          {
            "end-effector": arm,
            "waypoints": [
                          {"rpyxyz":[0.3,0.8128,0.1109,pose[0], pose[1], pose[2]]},
                          {"rpyxyz":[0.3,0.8128,0.1109,pose[0], pose[1], pose[2]]}],
            "reference-frame": {
              "robot-frame": "base"
            },
            "stall-threshold": None,
            "cyclic": False,
            "max-speed": 0.5,
            "duration": dur,
            "transition-duration": None
          }, 0] 
        self.client.send(json.dumps(msg))


    def move_both_arms(self, lpose, rpose, dur):
        msg = ["action-concurrent",
                {
                    "actions": [
                        ["action-end-effector-move",
                              {
                                "end-effector": 'left-hand',
                                "waypoints": [
                                              {"rpyxyz":[0.3,0.8128,0.1109,lpose[0], lpose[1], lpose[2]]},
                                              {"rpyxyz":[0.3,0.8128,0.1109,lpose[0], lpose[1], lpose[2]]}],
                                "reference-frame": {
                                  "robot-frame": "base"
                                },
                                "stall-threshold": None,
                                "cyclic": False,
                                "max-speed": 0.5,
                                "duration": dur,
                                "transition-duration": None
                              }, 0],
                        ["action-end-effector-move",
                            {
                                "end-effector": 'right-hand',
                                "waypoints": [
                                              {"rpyxyz":[0.3,0.8128,0.1109,rpose[0], rpose[1], rpose[2]]},
                                              {"rpyxyz":[0.3,0.8128,0.1109,rpose[0], rpose[1], rpose[2]]}],
                                "reference-frame": {
                                  "robot-frame": "base"
                                },
                                "stall-threshold": None,
                                "cyclic": False,
                                "max-speed": 0.5,
                                "duration": dur,
                                "transition-duration": None
                              }, 0]  
                    ]
                }
        ]
        self.client.send(json.dumps(msg))


    def get_wrist_pose(self, armname):
        arm = self.armname[armname]
        msg = [
              "get-object-kinematics",
              {
                "object": {"robot-frame":arm},
                "relative-to": {
                  "robot-frame":"base"
                },
                "in-coordinates-of": {}
              }
            ]
        self.client.send(json.dumps(msg))
        time.sleep(0.05)
        pose = self.client.arm_pose
        print(pose)
        return pose


    def move_ee_to_pose(self, pose, armname, dur):
        #compute relative pose of stub
        #offset from gripper: z=-16cm, x=6cm
        wrist_x = pose[0]# - self.x_offset
        wrist_y = pose[1]
        wrist_z = pose[2] #+ self.z_offset
        self.send_wrist_msg([wrist_x, wrist_y, wrist_z],dur, armname) 
        # time.sleep(10)
        # arm_pose = self.get_wrist_pose(armname)
        # confs = self.compute_gripper_confs(pose, arm_pose)
        # self.move_gripper_to_conf(confs, armname) 


    def compute_gripper_confs(self, pose):
        pass


    def move_gripper_to_conf(self, confs, armname):
        try:
            channel = rospy.ServiceProxy("conf_channel", Conf)
            send_request = ConfRequest()
            arm = self.arm_id[armname]
            send_request.arm.data = arm
            send_request.conf.x = self.none
            send_request.conf.y = confs[0]
            send_request.conf.z = confs[1]

            response = channel(send_request)
            if response.status:
               print("successfully sent conf ") 
            else:
                print("failed to send conf ")

        except rospy.ServiceException as e:
            print(e)
            sys.exit()


    def close_gripper(self, armname):
        # print('Gripper closed')
        try:
            channel = rospy.ServiceProxy("conf_channel", Conf)
            send_request = ConfRequest()
            arm = self.arm_id[armname]
            send_request.arm.data = arm
            send_request.conf.x = 1.0
            send_request.conf.y = self.none
            send_request.conf.z = self.none

            response = channel(send_request)
            if response.status:
               print("successfully sent conf ") 
            else:
                print("failed to send conf ")

        except rospy.ServiceException as e:
            print(e)
            sys.exit()


    def open_gripper(self, armname):
        # print("Gripper open")
        try:
            channel = rospy.ServiceProxy("conf_channel", Conf)
            send_request = ConfRequest()
            arm = self.arm_id[armname]
            send_request.arm.data = arm
            send_request.conf.x = 0.0
            send_request.conf.y = self.none
            send_request.conf.z = self.none

            response = channel(send_request)
            if response.status:
               print("successfully sent conf ") 
            else:
                print("failed to send conf ")

        except rospy.ServiceException as e:
            print(e)
            sys.exit()


if __name__ == '__main__':
    # ws = BasicClient('ws://10.10.2.1:8080', protocols=['json-v1-agility'])
    ws = BasicClient('ws://127.0.0.1:8080', protocols=['json-v1-agility'])
    ws.daemon = False

    while True:
        try:
            ws.connect()
            print("WS connection established")
            time.sleep(1)
            break
        except:
            print('WS connection NOT established')
            time.sleep(1)

# -0.0136,0.8128,0.1109,0.4,-0.25,0.15
    
    dc = Digit_Control(ws)
    dt = 3

    

    # dc.close_gripper(armname='left')
    # time.sleep(dt)

    # dc.open_gripper(armname='left')
    # time.sleep(dt)

    dc.move_gripper_to_conf([100,150], armname='left')
'''
    pose = [0.2,-0.25,0.15]
    dc.move_ee_to_pose(pose, armname = 'right', dur=5)
    time.sleep(dt)

    pose = [0.4,-0.25,0.1]    
    dc.move_ee_to_pose(pose, armname = 'right', dur=2.5)
    time.sleep(dt)

    pose = [0.4,-0.25,0.025]    
    dc.move_ee_to_pose(pose, armname = 'right', dur=2.5)
    time.sleep(dt)

    dc.close_gripper(armname='right')
    time.sleep(dt)

    

    pose = [0.4,-0.25,0.1]    
    dc.move_ee_to_pose(pose, armname = 'right', dur=2.5)
    time.sleep(dt)

    pose = [0.2,-0.25,0.15] 
    dc.move_ee_to_pose(pose, armname = 'right', dur=5)
    time.sleep(dt)

    dc.open_gripper(armname='right')
    time.sleep(dt)

'''

    # print("Done")