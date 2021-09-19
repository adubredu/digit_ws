import rospy
import sys, os, time, copy
sys.path.append('../../digit_control/src')
from digit_control import Digit_Control
from digit_control import BasicClient

class Digit_Manipulation:
    def __init__(self, dc):
        self.dc = dc
        self.dt = 3
        self.none = 999

    def pick_object(self, poses, armname): 
        self.dc.open_gripper(armname=armname)
        time.sleep(self.dt) 
        # '''
        p = [0.2,-0.25,0.15]
        self.dc.move_ee_to_pose(p, armname = armname, dur=5)
        time.sleep(self.dt)

        pose = poses[:-1]
        prepose = copy.deepcopy(pose)
        prepose[2]+=0.075 
        self.dc.move_ee_to_pose(prepose, armname =armname, dur=2.5)
        time.sleep(self.dt)
        print('At prepose ',prepose)

        confs = [poses[-1], self.none]
        self.dc.move_gripper_to_conf(confs, 'right')
        time.sleep(self.dt)

        self.dc.move_ee_to_pose(pose, armname = armname, dur=2.5)
        time.sleep(self.dt)
        print('At pose ',pose)

        self.dc.close_gripper(armname=armname)
        time.sleep(self.dt+5)

        self.dc.move_ee_to_pose(prepose, armname = armname, dur=2.5)
        time.sleep(self.dt)

        p = [0.2,-0.25,0.15] 
        self.dc.move_ee_to_pose(p, armname = armname, dur=5)
        time.sleep(self.dt)

        # self.dc.open_gripper(armname=armname)
        # time.sleep(self.dt)


    def place_object(self, poses, armname):
        p = [0.2,-0.25,0.4]
        self.dc.move_ee_to_pose(p, armname = armname, dur=5)
        time.sleep(self.dt)

        pose = poses[:-1]
        prepose = copy.deepcopy(pose)
        prepose[2]+=0.15
        self.dc.move_ee_to_pose(prepose, armname =armname, dur=7)
        time.sleep(self.dt+5)
        print('At prepose ',prepose)

        confs = [self.none, poses[-1]]
        self.dc.move_gripper_to_conf(confs, 'right')
        time.sleep(self.dt)

        self.dc.move_ee_to_pose(pose, armname = armname, dur=2.5)
        time.sleep(self.dt)
        print('At pose ',pose)

        self.dc.open_gripper(armname=armname)
        time.sleep(self.dt+5)

        self.dc.move_ee_to_pose(prepose, armname = armname, dur=2.5)
        time.sleep(self.dt)

        p = [0.2,-0.25,0.15] 
        self.dc.move_ee_to_pose(p, armname = armname, dur=7)
        time.sleep(self.dt)

        confs = [self.none, 270]
        self.dc.move_gripper_to_conf(confs, 'right')
        time.sleep(self.dt)


    def raise_arm(self, armname):
        p = [0.2,-0.25,0.15]
        if armname == 'right':
            p = [0.2,-0.25,0.15]
        else:
            p = [0.2, 0.25, 0.15]
        self.dc.move_ee_to_pose(p, armname = armname, dur=5)
        time.sleep(self.dt) 


    def raise_both_arms(self):
        rp = [0.2,-0.25,0.15]
        lp = [0.2,0.25,0.15]
        self.dc.move_both_arms(lp, rp, dur=5)


    def hello_demo(self):
        self.raise_both_arms()
        time.sleep(10)
        dt = 5

        confs=[self.none, 90]
        self.dc.move_gripper_to_conf(confs, 'right')
        time.sleep(dt)

        confs=[self.none, 60]
        self.dc.move_gripper_to_conf(confs, 'left')
        time.sleep(dt)

        confs=[60, self.none]
        self.dc.move_gripper_to_conf(confs, 'right')
        time.sleep(dt)

        confs=[60, self.none]
        self.dc.move_gripper_to_conf(confs, 'left')
        time.sleep(dt)

        confs=[240, self.none]
        self.dc.move_gripper_to_conf(confs, 'right')
        time.sleep(dt)

        confs=[240, self.none]
        self.dc.move_gripper_to_conf(confs, 'left')
        time.sleep(dt)

        confs=[150, self.none]
        self.dc.move_gripper_to_conf(confs, 'right')
        time.sleep(dt)

        confs=[150, self.none]
        self.dc.move_gripper_to_conf(confs, 'left')
        time.sleep(dt)

        confs=[self.none, 180]
        self.dc.move_gripper_to_conf(confs, 'right')
        time.sleep(dt)

        confs=[self.none, 150]
        self.dc.move_gripper_to_conf(confs, 'left')
        time.sleep(dt)

        confs=[self.none, 270]
        self.dc.move_gripper_to_conf(confs, 'right')
        time.sleep(dt)

        confs=[self.none, 240]
        self.dc.move_gripper_to_conf(confs, 'left')
        time.sleep(dt)


    def pack_groceries(self):
        sponge =  [0.5,-0.2,0.025, 150] 
        chlorox =  [0.4,-0.3,0.025, 135]
        tea =  [0.4,-0.15,0.022, 135] 
        cube =  [0.32,-0.24,0.018, 170]
        banana =  [0.31,-0.17,0.016, 150] 
        lion =   [0.38,-0.22,0.025, 150]

        placepose1 = [0.35,0.13,0.05, 260]   
        placepose2 = [0.35,0.05,0.05,270]  
        placepose3 = [0.35,0.0,0.05,270] 
        placepose4 = [0.35,0.13,0.08, 260]   
        placepose5 = [0.35,0.05,0.08,270]  
        placepose6 = [0.35,0.0,0.08,270] 
        dt = 5

        self.pick_object(sponge, 'right')
        time.sleep(dt)
        self.place_object(placepose1, 'right')
        time.sleep(dt) 

        self.pick_object(chlorox, 'right')
        time.sleep(dt)
        self.place_object(placepose2, 'right')
        time.sleep(dt) 

        self.pick_object(tea, 'right')
        time.sleep(dt)
        self.place_object(placepose3, 'right')
        time.sleep(dt) 

        self.pick_object(cube, 'right')
        time.sleep(dt)
        self.place_object(placepose4, 'right')
        time.sleep(dt) 

        self.pick_object(banana, 'right')
        time.sleep(dt)
        self.place_object(placepose5, 'right')
        time.sleep(dt) 

        self.pick_object(lion, 'right')
        time.sleep(dt)
        self.place_object(placepose6, 'right')
        time.sleep(dt) 




















        # '''
if __name__ == '__main__':
    ws = BasicClient('ws://10.10.2.1:8080', protocols=['json-v1-agility'])
    # ws = BasicClient('ws://127.0.0.1:8080', protocols=['json-v1-agility'])
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
 #right :=> flatyaw = 150
 #right :=>pitch = horizontal=180 verticalup=90 verticaldown=270

#left :=> flatyaw = 150
 #left :=> pitch = horizontal=150 verticalup=60 verticaldown=240
    
    dc = Digit_Control(ws)
    # armname='left'
    # dc.close_gripper(armname=armname)
    # time.sleep(5)
    # dc.open_gripper(armname=armname)
    # time.sleep(5)
    # dc.close_gripper(armname=armname)
    # time.sleep(5)
    # dc.open_gripper(armname=armname)

    # '''
    dm = Digit_Manipulation(dc)
    sponge =  [0.5,-0.2,0.025, 150] 
    chlorox =  [0.4,-0.3,0.025, 135]
    tea =  [0.4,-0.15,0.022, 135] 
    cube =  [0.32,-0.24,0.018, 170]
    banana =  [0.31,-0.17,0.016, 150]
    # banana2 =  [0.3,-0.35,0.02, 150] 
    lion =   [0.38,-0.22,0.025, 150]

    placepose1 = [0.35,0.13,0.05, 260]   
    placepose2 = [0.35,0.05,0.05,270]  
    placepose3 = [0.35,0.0,0.05,270] 

    repick1 = [0.4,0.1,0.04, 150]  

    # dm.hello_demo()
    dm.pack_groceries()

    # dm.raise_both_arms()
    # dm.raise_arm('left')
    # confs=[150, dm.none]
    # dm.dc.move_gripper_to_conf(confs, 'left')

    # print("Picking object 1...")
    # dm.pick_object(lion, 'right')
    # time.sleep(5)
    # dm.place_object(placepose3, 'right')
    time.sleep(10)

    # print("Picking object 2...")
    # dm.pick_object(pickpose2, 'right')
    # time.sleep(5)
    # dm.place_object(placepose2, 'right')
    # time.sleep(10)

    # print("Picking object 3...")
    # dm.pick_object(pickpose3, 'right')
    # time.sleep(5)
    # dm.place_object(placepose3, 'right')
    # time.sleep(15)

    # print("Re-picking object 1...")
    # repick1 = [0.4,0.1,0.04]
    # dm.pick_object(repick1, 'right')
    # time.sleep(5)
    # dm.place_object(pickpose1, 'right')
    # time.sleep(10)

    # print("Picking object 4...")
    # dm.pick_object(pickpose4, 'right')
    # time.sleep(5)
    # dm.place_object(placepose1, 'right')
    # time.sleep(10)

    # print("Picking object 5...")
    # dm.pick_object(pickpose6, 'right')
    # time.sleep(5)
    # dm.place_object(placepose2, 'right')
    # time.sleep(10)

    # print("Picking object 1 again...")
    # dm.pick_object(pickpose1, 'right')
    # time.sleep(5)
    # dm.place_object(placepose1, 'right')
    # time.sleep(10)
    # '''


#picks
    # 1. [0.4,-0.25,0.025] 
    # 2. [0.4,-0.3,0.025]
    # 3. [0.4,-0.2,0.022] 
    # 4. [0.3,-0.25,0.018]
    # 5. [0.3,-0.2,0.016]
    # 6. [0.3,-0.3,0.02]   


#places
    # 1. [0.4,0.1,0.07]   
    # 2. [0.4,0.05,0.07]  
    # 3. [0.4,0.0,0.07]