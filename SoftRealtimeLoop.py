"""
Soft Realtime Loop---a class designed to allow clean exits from infinite loops
with the potential for post-loop cleanup operations executing.

The Loop Killer object watches for the key shutdown signals on the UNIX operating system (which runs on the PI)
when it detects a shutdown signal, it sets a flag, which is used by the Soft Realtime Loop to stop iterating.
Typically, it detects the CTRL-C from your keyboard, which sends a SIGTERM signal.

the function_in_loop argument to the Soft Realtime Loop's blocking_loop method is the function to be run every loop.
A typical usage would set function_in_loop to be a method of an object, so that the object could store program state.
See the 'ifmain' for two examples.
"""

import signal
import time
from math import sqrt
import heapq
import os
import sys


# Calculate the precision of sleep. Python 3.11 has nanosecond sleep
# Older versions have microsecond sleep
python_version = sys.version_info.minor
if python_version >= 11:
    PRECISION_OF_SLEEP = 1e-9 
else:
    PRECISION_OF_SLEEP = 1e-6

# Version of the SoftRealtimeLoop library
__version__="2.0.0"

class LoopKiller:
    def __init__(self, fade_time=0.0):
        if os.name == 'posix':
            self.signals = [signal.SIGTERM, signal.SIGINT, signal.SIGHUP]
        else:
            self.signals = [signal.SIGTERM, signal.SIGINT]

        for sig in self.signals:
            signal.signal(sig, self.handle_signal)
        
        self._fade_time = fade_time
        self._soft_kill_time = None

    def handle_signal(self,signum, frame):
        self.kill_now=True

    def get_fade(self):
        # interpolates from 1 to zero with soft fade out
        if self._kill_soon:
            t = time.monotonic()-self._soft_kill_time
            if t>=self._fade_time:
                return 0.0
            return 1.0-(t/self._fade_time)
        return 1.0

    _kill_now = False
    _kill_soon = False
    @property
    def kill_now(self):
        if self._kill_now:
            return True
        if self._kill_soon:
            t = time.monotonic()-self._soft_kill_time
            if t>self._fade_time:
                self._kill_now=True
        return self._kill_now

    @kill_now.setter
    def kill_now(self, val):
        if val:
            if self._kill_soon: # if you kill twice, then it becomes immediate
                self._kill_now = True
            else:
                if self._fade_time > 0.0:
                    self._kill_soon = True
                    self._soft_kill_time = time.monotonic()
                else:
                    self._kill_now = True
        else:
            self._kill_now = False
            self._kill_soon = False
            self._soft_kill_time = None

class SoftRealtimeLoop(object):
    def __init__(self, dt=0.001, report=False, fade=0.0, 
                             max_error_trigger_value: float = float('inf'), 
                             max_error_trigger_kill: bool = False,
                             track_naive_time: bool = True,
                             increase_scheduler_priority: bool = False):
        """
        The SoftRealtimeLoop object is a class designed to allow perform smart
        loops that can approximate a real time operating system. It also allows
        clean exits from infinite loops with the potential for post-loop cleanup 
        operations executing. You can also kill the loop if it exceeds a certain
        error threshold.

        Parameters
        ----------
        dt : float
                The time step of the loop in seconds.
        report : bool
                If True, the loop will print a report at the end of the loop.
        fade : float
                The time in seconds to fade out the loop when it is killed.
        max_error_trigger_value : float
                The maximum error value in seconds that the loop can have before it is 
                killed. The default value is infinity (i.e. it is never triggered).
        max_error_trigger_kill : bool
                If True, the loop will be killed if a loop error exceeds the max error. 
                The default value is False. 
        track_naive_time : bool
                If True, the iterator object will try to keep the time elapsed equal 
                to (loop_number*dt). Therefore, if is strays too far from this number
                it will run at full speed to catch up. If False, the iterator object
                will only look at the difference between the current loop and the 
                previous loop. The original behaviour is the same as 
                track_naive_time=True. The default value is True.
        increase_scheduler_priority : bool
            if True, this will increase the scheduler priority of the real time loop
            so that the time that it wakes up from sleep is increased. Having a 
            higher priority can potentially starve other processes. Setting it to 
            false keeps the normal priority. Default value is false. 
        """
        ## Original variables for the next_track_naive_time method
        self.t0 = self.t1 = time.monotonic()
        self.ttarg = None 
        self.sleep_t_agg = 0.0
        
        # Max error trigger
        self.max_error_trigger_value = max_error_trigger_value
        self.max_error_trigger_kill = max_error_trigger_kill
        self.adjacent_error_counter = 0
        self.max_adjacent_error = 0

        ## New variables for the next_prev_loop_independent method
        self.initial_time = None
        self.prev_loop_time = None
        
        # Common variables
        self.sum_err = 0.0
        self.sum_var = 0.0
        self.n = 0
        self.dt = dt
        self.killer = LoopKiller(fade_time=fade)
        self.report = report
        self.max_errors = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Set the corresponding next function
        self.track_naive_time = track_naive_time

        # If dt is smaller than the precision of sleep, then raise an error
        if dt < PRECISION_OF_SLEEP:
            raise ValueError(f"The dt ({dt}s) has to be larger than the precision of"
                                             f" sleep ({PRECISION_OF_SLEEP}s)")
        if dt < 0:
            raise ValueError(f"The dt ({dt}s) has to be positive")
        
        # Increase the scheduler priority if the user wants to
        self.increase_scheduler_priority = increase_scheduler_priority
        if self.increase_scheduler_priority:
            pid = os.getpid()
            sched_priority = os.sched_get_priority_max(os.SCHED_FIFO)
            # Set the scheduler policy to be SCHED_FIFO. FIFO and RR work well, 
            # I haven't gotten the deadline scheduler to work yet.
            os.system(f'sudo chrt -f -p {sched_priority} {pid}')
        
    def __del__(self):
        if self.report:
            # Calculate the total time you ran
            if self.track_naive_time:
                total_time = self.t1-self.t0
            else:
                total_time = self.prev_loop_time-self.initial_time
            print('In %d cycles at %.2f Hz:'%(self.n, 1./self.dt))
            print('\tavg error: %.3f milliseconds'% (1e3*self.sum_err/max(self.n,1)))
            print('\tstddev error: %.3f milliseconds'% (1e3*sqrt((self.sum_var-self.sum_err**2/max(self.n,1))/(self.n-1))))
            print('\tpercent of time sleeping: %.1f %%' % (self.sleep_t_agg/total_time*100.))
            print('\tfive max cycle errors: %.3f, %.3f, %.3f, %.3f, %.3f milliseconds'% (1e3*self.max_errors[0], 1e3*self.max_errors[1], 1e3*self.max_errors[2], 1e3*self.max_errors[3], 1e3*self.max_errors[4]))

    @property
    def fade(self):
        return self.killer.get_fade()

    def stop(self):
        self.killer.kill_now=True

    def __iter__(self):
        self.t0 = self.t1 = time.monotonic()+self.dt
        self.prev_loop_time = None
        return self

    def __next__(self):
        """
        This is the main method that will be called when the SoftRealtimeLoop
        object is used as an iterator. It will select the appropriate next 
        method based on the self.track_naive_time variable.
        """
        if self.track_naive_time:
            return self._next_track_naive_time()
        else:
            return self._next_track_dt()

    def _next_track_dt(self):
        """
        This method will prioritize having a consistent dt over each loop iteration.
        In contrast, the next_track_naive_time method will attempt "catch up" 
        on time lost in previous loops by sleeping less in the current loop.
        
        This is not the default mode and neets to be enabled via setting 
        track_naive_time parameter in the SoftRealtimeLoop initializer to False.
        
        This object will return the time since the iterator object started.
        """

        # If the loop is killed, raise a StopIteration
        if self.killer.kill_now:
            raise StopIteration
        
        ## Sleep the amount we need to satisfy the dt.
        # Verify if we are in the first loop, if so, just sleep the dt
        if self.prev_loop_time is None:
            self.initial_time = self.prev_loop_time = time.monotonic()
            sleep_time = self.dt
        # If we are not in the first loop, calculate the time we need to sleep
        # based on the remaining time to satisfy the dt
        else:
            time_since_last_loop = time.monotonic()-self.prev_loop_time
            sleep_time = max(self.dt - time_since_last_loop - 2*PRECISION_OF_SLEEP,0)
        time.sleep(max(PRECISION_OF_SLEEP,sleep_time))
        # Update the time slept
        self.sleep_t_agg+=sleep_time

        # Busy wait to compensate for sleep durations precision
        # We don't busy wait all the time since that gives problem with dephy
        time_to_busy_wait = time.monotonic() + PRECISION_OF_SLEEP
        while time.monotonic() < time_to_busy_wait and not self.killer.kill_now:
            if os.name == 'posix':
                if signal.sigtimedwait(self.killer.signals, 0):
                    self.stop()
                    raise StopIteration
            
        ## Store the current time
        current_time = time.monotonic()

        ## Handle how much error that we have in a given loop
        # Calculate the error for the loop and update the max errors
        error = (current_time - self.prev_loop_time) - self.dt
        # Update the statistics for the error
        self.sum_err += abs(error)
        self.sum_var += abs(error)**2
        self.n+=1
        # Update the max errors
        if abs(error) > self.max_errors[0]:
            heap = self.max_errors + [error]
            heapq.heapify(heap)
            heapified_heap = [heapq.heappop(heap) for _ in range(len(heap))]
            self.max_errors = heapified_heap[1:]
        # If the error exceeds the max error trigger value
        self._report_error(error,sleep_time)

        # Calculate the time since the iterator object started
        time_since_start = current_time - self.initial_time

        # Update the previous loop time
        self.prev_loop_time = current_time

        return time_since_start

    def _report_error(self, error, time_slept):
        """
        This method will report the error of the loop if the error exceeds the
        max error trigger value. It will also kill the loop if the user set it up
        that way.
        """
        # If the error exceeds the max error trigger value, inform the user
        if abs(error) > self.max_error_trigger_value:
            # Keep track of errors that happen next to each other
            self.adjacent_error_counter += 1 
            if self.adjacent_error_counter == 2:
                self.max_adjacent_error = error
            error_pct = 100*abs(error)/self.dt
        
            print(f"SoftRealTimeLoop: loop error {1e3*error:3.3f}ms / {error_pct:3.0f}% | "
                        f"dt = {1e3*self.dt} ms | Time slept {1e3*time_slept:.3f}ms | "
                        f"Adjacent errors: {self.adjacent_error_counter}/{1e3*self.max_adjacent_error:3.3f}ms")
    
            # Kill the loop if the user set it up that way
            if self.max_error_trigger_kill:
                self.stop()
        else:
            # Reset the adjacent error counter
            self.adjacent_error_counter = 0
            self.max_adjacent_error = 0


    def _next_track_naive_time(self):
        """
        This method will prioritize "catching up" on time lost in previous loops
        by sleeping less in the current loop. In contrast, the
        next_prev_loop_independent method will prioritize having a consistent dt
        over each loop iteration.

        This object will return the time that we should be running at (i.e. not
        the real time).
        """

        # If the loop is killed, raise a StopIteration
        if self.killer.kill_now:
            raise StopIteration

        ## Sleep the amount we need to satisfy the dt.
        sleep_curr_loop = 0
        # Calculate the time we need to sleep
        sleep_time = self.t1 - 2*PRECISION_OF_SLEEP - time.monotonic()
        
        while sleep_time > 0 and not self.killer.kill_now:
            # Calculate the time spent sleeping
            t_pre_sleep = time.monotonic()
            # Sleep for the time we need to satisfy the dt
            time.sleep(max(PRECISION_OF_SLEEP,sleep_time + PRECISION_OF_SLEEP))
            # Update the time spent sleeping to calculate the sleep percentage
            sleep_curr_loop += time.monotonic()-t_pre_sleep
            # Recalculate if we still need to sleep
            sleep_time = self.t1 - 2*PRECISION_OF_SLEEP - time.monotonic()

        # Update the time slept
        self.sleep_t_agg+=sleep_curr_loop

        # Busy wait until the time we should be running at
        while time.monotonic()<self.t1 and not self.killer.kill_now:
            if os.name == 'posix':
                if signal.sigtimedwait(self.killer.signals, 0):
                    self.stop()

        # If the loop is killed while we were waiting, raise a StopIteration
        if self.killer.kill_now:
            raise StopIteration
        
        # Increase the dt naively based on the time that we should have slept
        self.t1+=self.dt
        
        # Initialize ttarg on first call
        if self.ttarg is None: 
            self.ttarg = time.monotonic()+self.dt
            # then skips the first loop
            return self.t1-self.t0

        # Calculate the error for the loop and update the max errors
        error = time.monotonic()-self.ttarg # seconds
        self.sum_err += error
        self.sum_var += error**2
        self.n+=1
        self.ttarg+=self.dt

        # Update the max errors
        if error > self.max_errors[0]:
            heap = self.max_errors + [error]
            heapq.heapify(heap)
            heapified_heap = [heapq.heappop(heap) for _ in range(len(heap))]
            self.max_errors = heapified_heap[1:]

        # If the error exceeds the max error trigger value, either inform the
        # user or kill the loop
        self._report_error(error, sleep_curr_loop)

        return self.t1-self.t0



