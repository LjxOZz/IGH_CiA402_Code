# import sys
# sys.path.insert(0, "../")

import time
from nmxrt import Subscriber
from nmxrt import Stamp

def str1_handler(msg: str, stamp: Stamp):
    print(f'receive spe: {msg} at {stamp.time}')

def str2_handler(msg: str, stamp: Stamp):
    print(f'receive pos: {msg} at {stamp.time}')

def str3_handler(msg: str, stamp: Stamp):
    print(f'receive tor: {msg} at {stamp.time}')


if __name__ == "__main__":
    sub1 = Subscriber("ct_motor0/state/speed", str, str1_handler)
    sub2 = Subscriber("ct_motor0/state/postion", str, str2_handler)
    sub3 = Subscriber("ct_motor0/state/torque", str, str3_handler)

    while True:
        time.sleep(1)
