
import time
import threading
import numpy as np

def wait_for_input(ans):
    global inp
    print(ans)
    inp = float(input())
    while not (inp in ans):
        inp = float(input())


def ff():
    global inp
    inp = 2
    ans = [1, 2]
    x = threading.Thread(target=wait_for_input, args = (np.linspace(0,2,int((2-0)*10+1)),))
    x.start()
    while True:
        time.sleep(1)
        print('',inp)
# x.join()
while True:
    ff()