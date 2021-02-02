from multiprocessing import Queue, Process
from queue import Empty as QueueEmpty
import random
from joystick_lib import joystick_value_get
from indy_joystick_sim import indy_joystick_sim

def getter(name, queue):
    #print 'Son process %s' % name
    while True:
        try:
            value = queue.get(True, 10)
            # block为True,就是如果队列中无数据了。
            #   |—————— 若timeout默认是None，那么会一直等待下去。
            #   |—————— 若timeout设置了时间，那么会等待timeout秒后才会抛出Queue.Empty异常
            # block 为False，如果队列中无数据，就抛出Queue.Empty异常
            #print "Process getter get: %f" % value
        except QueueEmpty:
            break


def putter(name, queue):
    #print "Son process %s" % name
    for i in range(0, 1000):
        value = random.random()
        queue.put(value)
        # 放入数据 put(obj[, block[, timeout]])
        # 若block为True，如队列是满的：
        #  |—————— 若timeout是默认None，那么就会一直等下去
        #  |—————— 若timeout设置了等待时间，那么会等待timeout秒后，如果还是满的，那么就抛出Queue.Full.
        # 若block是False，如果队列满了，直接抛出Queue.Full
        #print "Process putter put: %f" % value


if __name__ == '__main__':
    queue = Queue()
    getter_process = Process(target=indy_joystick_sim, args=("Getter", queue))
    putter_process = Process(target=joystick_value_get, args=("Putter", queue))
    getter_process.start()
    putter_process.start()