import time
import logging


logger = logging.getLogger(__name__)

class Timer(object):
    def __init__(self, name=None, printFlag = True):
        self.name = name
        self.printFlag = printFlag

    def __enter__(self):
        self.tstart = time.time()

    def __exit__(self, type, value, traceback):
        if self.printFlag:
            elapsed_s = time.time() - self.tstart
            if self.name:
                logger.debug("%s | elapsed=%0.6fs", self.name, elapsed_s)
            else:
                logger.debug("unknown_name | elapsed=%0.6fs", elapsed_s)