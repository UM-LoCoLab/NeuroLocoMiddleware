import time
from math import sqrt

class StatProfiler():
    """ Profiler class with statistics"""
    def __init__(self, name):
        self.N, self.agg, self.aggvar = 0, 0.0, 0.0
        self.name=name

    def __del__(self):
        try:
            mean = self.agg/self.N
            stddev = sqrt((self.aggvar - self.N*mean**2)/(self.N-1)) 
        except ZeroDivisionError:
            mean = float('NaN')
            stddev=float('NaN')
        print(f"StatProfiler {self.name}: {self.N} reps, avg: {mean*1e3} ms, stddev: {stddev*1e3} ms, total: {self.agg} s")

    def tic(self):
        """ Matlab style """
        self.t0=time.time()

    def toc(self):
        """ Matlab style """
        t = time.time()-self.t0
        self.N+=1
        self.agg+=t
        self.aggvar+=t**2

    def profile(self, func):
        """ lambda style """
        self.tic()
        x = func()
        self.toc()
        return x

    def decorate(self, func):
        """ Decorator style"""
        def ret(*args, **kwargs):
            self.tic()
            x = func(*args, **kwargs)
            self.toc()
            return x
        return ret


class SSProfile(StatProfiler):
    """ Singleton Stat Profilers """
    _instances = dict()
    def __new__(cls, name, **kwargs):
        if name not in cls._instances:
            print("instantiating singleton StatProfiler %s"%name)
            cls._instances[name] = StatProfiler(name)
        return cls._instances[name]


def test_no_runs():
    SSProfile("test_none")

def test_decorator():
    @SSProfile("decorator").decorate
    def my_decorated_function():
        time.sleep(0.0001)
    for i in range(1000):
        my_decorated_function()

def test_lambda():
    for i in range(1000):
        SSProfile("lambda").profile(lambda : time.sleep(0.0001))

def test_tic_toc():
    for i in range(1000):
        SSProfile("tic_toc").tic()
        time.sleep(0.0001)
        SSProfile("tic_toc").toc()

if __name__ == '__main__':
    test_no_runs()
    test_decorator()
    test_lambda()
    test_tic_toc()
