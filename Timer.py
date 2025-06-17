import time

class Timer(object):
    def __init__(self, name=None, printFlag = True):
        self.name = name
        self.printFlag = printFlag

    def __enter__(self):
        self.tstart = time.time()

    def __exit__(self, type, value, traceback):
        if self.printFlag:
            if self.name:
                print('[%s]' % self.name,)
            print('Elapsed: %s' % (time.time() - self.tstart))