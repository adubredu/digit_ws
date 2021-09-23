import rospy
import sys
import time
from lesample import Lesample
sys.path.append('../../digit_manipulation/src')
sys.path.append('../../digit_perception/src')
sys.path.append('../../digit_control/src')
from digit_control import Digit_Control
from digit_control import BasicClient
from digit_perception import Digit_Perception
from digit_manipulation import Digit_Manipulation


if __name__ == "__main__":
    ws = BasicClient('ws://10.10.2.1:8080', protocols=['json-v1-agility'])
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
 
    
    dc = Digit_Control(ws)
    dp = Digit_Perception()
    dm = Digit_Manipulation(dc)
    domain_path = '/home/alphonsus/research/digit/digit_ws/src/digit_planning/pddl/domain.pddl'
    print('Starting')
    ls = Lesample(dm, dp, domain_path)
    ls.perform_lesample()
    print('\nComplete')